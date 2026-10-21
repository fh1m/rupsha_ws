"""Planar right-invariant EKF on SE(2): the shadow estimate of where the rover is.

WHAT THIS IS FOR. The Pixhawk's EKF3 is the navigation authority. This filter
exists to (a) cross-check the board and (b) carry a GNSS-denied leg when the
operator opts in to ExtNav. It is not trying to beat EKF3 at EKF3's job; it is
trying to be an INDEPENDENT second opinion, which is why it propagates on raw
gyro and odometry rather than on the board's fused attitude.

WHY INVARIANT. Barrau & Bonnabel, "The Invariant Extended Kalman Filter as a
Stable Observer", IEEE Trans. Automatic Control 62(4), 2017. For dynamics of
the form T_{k+1} = T_k exp(u dt) on a matrix Lie group, the invariant error
obeys a LOG-LINEAR recursion whose Jacobian does not depend on the estimate.
A standard EKF linearised about a wrong yaw uses a Jacobian that is wrong in
exactly the direction of the error; this one does not, which is why it
converges from a 90 degree wrong heading where a naive EKF can lock in.

THE HANDEDNESS, stated precisely because it is where planar filters go wrong.
The error used here is xi with

    T_wb = T_hat_wb * exp(xi)            (xi = rho_x, rho_y, theta)

Written on the state X = T_bw = T_wb^-1 (world-to-body), that is exactly
Barrau & Bonnabel's RIGHT-invariant error eta = X_hat X^-1. In their
terminology a GNSS fix of the antenna position is then a right-invariant
observation Y = X^-1 b + V, b = (0, 0, 1), and its error Jacobian is the
constant H = [I_2 0] once the innovation is rotated into the body frame.
Writing the same filter on T_wb would make it the "left-invariant" one; the
algebra is identical, only the name moves. What must NOT change is the pair:
error and observation of the same invariance class, so that H is
state-independent -- the property the filter is chosen for.

TWO CAVEATS, carried not glossed:
  1. The gyro bias is an additive Euclidean augmentation outside the group. It
     couples into the error dynamics and the log-linear property is then only
     approximate, as Barrau & Bonnabel note for any biased system.
  2. Heading (compass/board yaw) and zero-velocity are not group observations
     of the invariant form. Heading is still linear in theta on SE(2) because
     planar rotations commute; zero-velocity constrains only the bias, because
     velocity is an INPUT here, not a state.

GATING. Every update computes its normalised innovation squared (NIS) and is
refused when it exceeds the chi-square 95% point for its dimension. At 95% a
correct filter refuses about 1 in 20 legitimate measurements -- that is the
price of a gate, and it is counted, not hidden. It has a second price: the
covariance does not know which fixes were refused, so the errors those fixes
would have corrected linger. Measured on 48 seeded figure-eight runs the
gated filter's ANEES is 3.37 against 3.0 for 3 dof (12% overconfident);
ungated it is 3.04. Each kind keeps accepted,
rejected and a consecutive-reject streak. A long streak means the FILTER is
probably the thing that is wrong: for GNSS position, REJECT_STREAK_LIMIT
consecutive refusals relocalise the filter (see `relocalise`); for heading
and zero-velocity the streak is only reported, because neither is the
absolute reference the filter can restart from.

Pure numpy. No ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from rupsha_localization import se2

# Chi-square 95% upper points. Hard-coded so the core needs no scipy.
CHI2_95 = {1: 3.841458820694124, 2: 5.991464547107979, 3: 7.814727903251178}

# Consecutive rejections of one kind after which the filter, not the sensor, is
# the prime suspect. 5 GNSS fixes is 5 s at 1 Hz: longer than a multipath
# excursion usually lasts, short enough that a lost filter does not drive on.
REJECT_STREAK_LIMIT = 5

# Eigenvalue floor applied when conditioning P. Below 1e-12 a variance is
# numerical noise; a negative one is a bug that must not reach a gate.
_EIG_FLOOR = 1e-12


@dataclass
class ProcessNoise:
    """Continuous-time noise DENSITIES (value * sqrt(dt) is the std of one step).

    A density, not a per-sample std, so the filter's confidence does not
    change when the loop rate does. A per-sample std sigma at rate f is the
    density sigma / sqrt(f).
    """
    speed: float = 0.02         # m/s/sqrt(Hz), along-track (v_x)
    lateral: float = 0.02       # m/s/sqrt(Hz), v_y; nominally 0 on a skid-steer
    gyro: float = 0.003         # rad/s/sqrt(Hz)
    bias_walk: float = 1e-4     # rad/s^2/sqrt(Hz)
    # Multiplier on speed and lateral densities while the slip gate is up.
    # 10 is a 100x variance: odometry still constrains, but barely.
    slip_inflation: float = 10.0


@dataclass
class UpdateResult:
    kind: str
    accepted: bool
    nis: float
    dof: int
    relocalised: bool = False


@dataclass
class _Counter:
    accepted: int = 0
    rejected: int = 0
    relocalisations: int = 0
    streak: int = 0


class RIEKF:
    """Pose T_wb in SE(2), plus an optional additive gyro bias."""

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                 pos_std: float = 1.0, yaw_std: float = math.radians(10.0),
                 estimate_bias: bool = True, bias: float = 0.0,
                 bias_std: float = 0.02,
                 noise: Optional[ProcessNoise] = None):
        for name, v in (('x', x), ('y', y), ('yaw', yaw), ('pos_std', pos_std),
                        ('yaw_std', yaw_std), ('bias', bias), ('bias_std', bias_std)):
            if not math.isfinite(float(v)):
                raise ValueError(f'RIEKF init: {name} is not finite ({v}); '
                                 'an initial state cannot be absent')
        self.T = se2.make(x, y, yaw)
        self.estimate_bias = bool(estimate_bias)
        self.b = float(bias)
        self.noise = noise or ProcessNoise()
        n = 4 if self.estimate_bias else 3
        P = np.zeros((n, n))
        # Body-frame position error equals world-frame for an isotropic prior.
        P[0, 0] = P[1, 1] = pos_std ** 2
        P[2, 2] = yaw_std ** 2
        if self.estimate_bias:
            P[3, 3] = bias_std ** 2
        self.P = P
        self.counters: Dict[str, _Counter] = {}
        self.propagations = 0
        self.slip_steps = 0

    # ------------------------------------------------------------ properties
    @property
    def dim(self) -> int:
        return self.P.shape[0]

    @property
    def x(self) -> float:
        return float(self.T[0, 2])

    @property
    def y(self) -> float:
        return float(self.T[1, 2])

    @property
    def yaw(self) -> float:
        return se2.yaw_of(self.T)

    @property
    def bias(self) -> float:
        """Gyro bias estimate, rad/s. NaN when the bias is not estimated."""
        return self.b if self.estimate_bias else float('nan')

    def world_covariance(self) -> np.ndarray:
        """3x3 covariance of (x, y, yaw) in the WORLD frame.

        The state error is body-frame (p = p_hat + R_hat rho), so position
        covariance is rotated out; yaw is frame-invariant in the plane.
        """
        J = np.eye(3)
        J[:2, :2] = self.T[:2, :2]
        Pp = self.P[:3, :3]
        C = J @ Pp @ J.T
        return 0.5 * (C + C.T)

    def position_std(self) -> float:
        """Worst-axis 1-sigma horizontal position error, m."""
        ev = np.linalg.eigvalsh(self.world_covariance()[:2, :2])
        return float(math.sqrt(max(ev.max(), 0.0)))

    def yaw_std(self) -> float:
        return float(math.sqrt(max(self.P[2, 2], 0.0)))

    def counter(self, kind: str) -> _Counter:
        return self.counters.setdefault(kind, _Counter())

    def report(self) -> Dict[str, Dict[str, int]]:
        """Accepted / rejected / relocalisations / streak per kind -- the gate's ledger."""
        return {k: {'accepted': c.accepted, 'rejected': c.rejected,
                    'relocalisations': c.relocalisations, 'streak': c.streak}
                for k, c in sorted(self.counters.items())}

    # ------------------------------------------------------------ propagate
    def propagate(self, v_x: float, v_y: float, omega_meas: float, dt: float,
                  slip: bool = False) -> None:
        """Advance by one body-twist step.

        `omega_meas` is the raw gyro z (FLU, CCW positive); the bias estimate
        is removed here. Any NaN input is REFUSED: a filter that substitutes
        zero for an absent speed is dead-reckoning a parked rover while the
        real one drives away, and it still reports a confident pose.
        """
        for name, v in (('v_x', v_x), ('v_y', v_y), ('omega', omega_meas), ('dt', dt)):
            if not math.isfinite(float(v)):
                raise ValueError(f'RIEKF.propagate: {name} is absent ({v}); '
                                 'the caller must not propagate on it')
        if dt <= 0.0:
            raise ValueError(f'RIEKF.propagate: dt must be positive, got {dt}')
        b = self.b if self.estimate_bias else 0.0
        u = np.array([v_x, v_y, omega_meas - b]) * dt
        step = se2.exp(u)
        self.T = se2.compose(self.T, step)

        # xi_{k+1} = Ad_{exp(-u dt)} xi_k - dt (dv, dw + db): exact in xi for
        # the noise-free part, first order in the bias coupling.
        n = self.dim
        F = np.eye(n)
        F[:3, :3] = se2.adjoint(se2.exp(-u))
        if self.estimate_bias:
            F[:3, 3] = -dt * np.array([0.0, 0.0, 1.0])
        k = self.noise.slip_inflation if slip else 1.0
        q = np.zeros(n)
        q[0] = (k * self.noise.speed) ** 2
        q[1] = (k * self.noise.lateral) ** 2
        q[2] = self.noise.gyro ** 2
        if self.estimate_bias:
            q[3] = self.noise.bias_walk ** 2
        self.P = F @ self.P @ F.T + np.diag(q * dt)
        self._condition()
        self.propagations += 1
        if slip:
            self.slip_steps += 1

    # ------------------------------------------------------------ updates
    def update_position(self, east: float, north: float,
                        cov: np.ndarray) -> UpdateResult:
        """GNSS antenna position in the local ENU plane, `cov` 2x2 (m^2), WORLD frame.

        Innovation is rotated into the body frame: z = R_hat^T (y - p_hat),
        so H = [I 0] with no dependence on the estimate.
        """
        y = np.array([east, north], dtype=float)
        cov = np.asarray(cov, dtype=float).reshape(2, 2)
        if not (np.all(np.isfinite(y)) and np.all(np.isfinite(cov))):
            raise ValueError('RIEKF.update_position: fix or covariance is absent '
                             f'({y}, {cov.tolist()})')
        R = self.T[:2, :2]
        z = R.T @ (y - self.T[:2, 2])
        Rm = R.T @ cov @ R
        H = np.zeros((2, self.dim))
        H[0, 0] = H[1, 1] = 1.0
        res = self._update('position', z, H, Rm)
        if not res.accepted and self.counter('position').streak >= REJECT_STREAK_LIMIT:
            self.relocalise(east, north, cov)
            res.relocalised = True
        return res

    def relocalise(self, east: float, north: float, cov: np.ndarray) -> None:
        """Declare the filter lost: position from the fix, heading UNKNOWN.

        A gate that can lock out forever turns one bad linearisation into
        permanent divergence: measured on the seeded figure-eight, a filter
        started 90 deg wrong swung the wrong way on its first 1 m-baseline
        fix, shrank its yaw std to 38 deg at 170 deg of true error, and then
        refused every fix for the rest of the run. After REJECT_STREAK_LIMIT
        consecutive refusals the joint estimate is inconsistent and nothing
        says which component is wrong, so both position and heading are
        re-opened; the invariant error dynamics re-acquire from there. Yaw
        keeps its value (it is as good a seed as any) and the bias keeps its
        own variance, which no GNSS streak is evidence against.
        """
        cov = np.asarray(cov, dtype=float).reshape(2, 2)
        R = self.T[:2, :2]
        self.T[0, 2], self.T[1, 2] = east, north
        self.P[:3, :] = 0.0
        self.P[:, :3] = 0.0
        self.P[:2, :2] = R.T @ cov @ R
        self.P[2, 2] = math.pi ** 2
        self._condition()
        c = self.counter('position')
        c.streak = 0
        c.relocalisations += 1

    def update_heading(self, yaw: float, std: float) -> UpdateResult:
        """Absolute ENU yaw (rad, CCW from east) with 1-sigma `std` (rad)."""
        if not (math.isfinite(yaw) and math.isfinite(std)) or std <= 0.0:
            raise ValueError(f'RIEKF.update_heading: yaw/std absent or invalid ({yaw}, {std})')
        z = np.array([se2.wrap_pi(yaw - self.yaw)])
        H = np.zeros((1, self.dim))
        H[0, 2] = 1.0
        return self._update('heading', z, H, np.array([[std * std]]))

    def update_zero_velocity(self, omega_meas: float, std: float) -> UpdateResult:
        """The rover is known to be stationary: the gyro reads its own bias.

        Velocity is an input in this filter, not a state, so the only thing a
        zero-velocity pseudo-measurement can constrain is the gyro bias. The
        caller is responsible for propagating with zero twist meanwhile.
        """
        if not self.estimate_bias:
            raise ValueError('RIEKF.update_zero_velocity: bias is not estimated, '
                             'so a zero-velocity update has nothing to constrain')
        if not (math.isfinite(omega_meas) and math.isfinite(std)) or std <= 0.0:
            raise ValueError(f'RIEKF.update_zero_velocity: input absent ({omega_meas}, {std})')
        z = np.array([omega_meas - self.b])
        H = np.zeros((1, self.dim))
        H[0, 3] = 1.0
        return self._update('zero_velocity', z, H, np.array([[std * std]]))

    def inflate(self, pos_std: float, yaw_std: float) -> None:
        """Add independent uncertainty for an interval nothing was integrated over."""
        if not (math.isfinite(pos_std) and math.isfinite(yaw_std)):
            raise ValueError('RIEKF.inflate: absent input')
        self.P[0, 0] += pos_std ** 2
        self.P[1, 1] += pos_std ** 2
        self.P[2, 2] += yaw_std ** 2
        self._condition()

    # ------------------------------------------------------------ internals
    def _update(self, kind: str, z: np.ndarray, H: np.ndarray,
                Rm: np.ndarray) -> UpdateResult:
        S = H @ self.P @ H.T + Rm
        S = 0.5 * (S + S.T)
        Sinv = np.linalg.inv(S)
        nis = float(z @ Sinv @ z)
        dof = z.shape[0]
        c = self.counter(kind)
        if nis > CHI2_95[dof]:
            c.rejected += 1
            c.streak += 1
            return UpdateResult(kind, False, nis, dof)
        K = self.P @ H.T @ Sinv
        d = K @ z
        self.T = se2.compose(self.T, se2.exp(d[:3]))
        if self.estimate_bias:
            self.b += float(d[3])
        I_KH = np.eye(self.dim) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self._condition()
        c.accepted += 1
        c.streak = 0
        return UpdateResult(kind, True, nis, dof)

    def _condition(self) -> None:
        P = 0.5 * (self.P + self.P.T)
        w, V = np.linalg.eigh(P)
        if w.min() < _EIG_FLOOR:
            w = np.maximum(w, _EIG_FLOOR)
            P = (V * w) @ V.T
            P = 0.5 * (P + P.T)
        self.P = P
