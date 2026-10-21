"""The shadow estimator's decisions, without ROS: which sample reaches the filter.

A localization node goes wrong in its wiring far more often than in its
maths: a filter fed a NaN, a zero, or a 3-second dt still publishes a
confident pose. Every such decision lives here so it is tested without rclpy;
`localization_node.py` only converts messages and calls these methods.

TIME is whatever the caller passes, in seconds, from one clock. The node uses
its own clock for every input, because the board-side publishers do not yet
promise board-time stamps.

WHAT THE SHADOW IS AND IS NOT INDEPENDENT OF. On the current hardware the GNSS
fix comes from the same receiver the board uses, and with no wheel encoders
the speed is the board's own groundspeed. The shadow is therefore independent
of EKF3's LOGIC (its tuning, lane switching, innovation handling), not of its
sensors. A divergence still means one of the two fusions is wrong; agreement
does not prove the shared sensors right.

GROUNDSPEED IS UNSIGNED. VFR_HUD groundspeed is a magnitude, so without wheel
speeds a reversing rover is propagated forwards. The GNSS gate then rejects,
the streak trips, and position is re-seeded -- visible in the ledger, not
silent. Wheel speeds remove the problem.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from rupsha_localization import health, se2
from rupsha_localization.geodesy import LocalTangentPlane
from rupsha_localization.icr_odometry import IcrParams, SlipGate, icr_twist
from rupsha_localization.riekf import REJECT_STREAK_LIMIT, RIEKF, ProcessNoise  # noqa: F401

# ArduPilot GPS_RAW_INT fix_type: 3 = 3D fix. A 2D fix has no vertical and a
# degraded horizontal; it is not an origin anyone should anchor a course to.
MIN_FIX_TYPE = 3

# A dt longer than this is a GAP, not a step. It is not integrated (one 2 s
# Euler step is not the same estimate as 40 steps of 50 ms); covariance is
# inflated by worst-case motion instead.
MAX_STEP_DT_S = 0.25


@dataclass
class ShadowConfig:
    rate_hz: float = 20.0
    track_m: float = 0.60
    icr_chi: float = 1.0
    slip_threshold_dps: float = 15.0
    slip_k: int = 3
    extnav_enabled: bool = False
    extnav_max_std_m: float = 2.0
    gnss_std_m: float = 1.5               # used when NavSatFix carries no covariance
    heading_from_board: bool = True
    heading_std_deg: float = 5.0
    heading_period_s: float = 1.0
    health_max_position_m: float = 5.0
    health_max_yaw_deg: float = 15.0
    imu_timeout_s: float = 0.5
    velocity_timeout_s: float = 0.5
    state_timeout_s: float = 2.0
    zupt_speed_mps: float = 0.03
    zupt_gyro_rad_s: float = 0.02
    max_speed_mps: float = 3.0            # worst case for gap inflation
    max_yaw_rate_dps: float = 120.0
    noise: ProcessNoise = field(default_factory=ProcessNoise)


@dataclass
class ShadowOutput:
    t: float
    x: float                  # ENU east, m, from the origin
    y: float                  # ENU north, m
    yaw: float                # ENU yaw, rad, CCW from east
    cov: np.ndarray           # 3x3 world covariance of (x, y, yaw)
    v_x: float                # body forward, m/s
    v_y: float
    omega: float              # bias-corrected yaw rate, rad/s
    lat: float
    lon: float
    compass_deg: float        # yaw in the board's convention
    position_std_m: float
    slipping: bool


class Shadow:
    def __init__(self, cfg: Optional[ShadowConfig] = None,
                 log: Optional[Callable[[str], None]] = None):
        self.cfg = cfg or ShadowConfig()
        self.log = log or (lambda _m: None)
        self.icr = IcrParams.from_track(self.cfg.track_m, self.cfg.icr_chi)
        self.slip = SlipGate(math.radians(self.cfg.slip_threshold_dps), self.cfg.slip_k)
        self.filter: Optional[RIEKF] = None
        self.plane: Optional[LocalTangentPlane] = None
        self._t: Optional[float] = None
        self._gyro = (None, float('nan'))          # (t, rad/s FLU)
        self._wheels = (None, float('nan'), float('nan'))
        self._state = None                         # dict of the last RupshaState
        self._state_t: Optional[float] = None
        self._last_heading_t = -math.inf
        self._twist = (float('nan'), float('nan'), float('nan'))
        self.stale_reason = 'no origin yet'
        self.gaps = 0
        self.resets = 0
        self.zupts = 0
        self.extnav_refusals = 0
        self.extnav_last_refusal = ''
        self.imu_sign_checks = 0
        self.imu_sign_disagreements = 0

    # ------------------------------------------------------------ inputs
    def on_imu(self, t: float, omega_z: float) -> None:
        self._gyro = (t, float(omega_z))

    def on_wheels(self, t: float, v_left: float, v_right: float) -> None:
        self._wheels = (t, float(v_left), float(v_right))

    def on_state(self, t: float, lat: float, lon: float, yaw_deg: float,
                 groundspeed: float, gps_fix: int, ekf_ok: bool,
                 yaw_rate_dps: float = float('nan')) -> None:
        self._state = dict(lat=float(lat), lon=float(lon), yaw_deg=float(yaw_deg),
                           groundspeed=float(groundspeed), gps_fix=int(gps_fix),
                           ekf_ok=bool(ekf_ok), yaw_rate_dps=float(yaw_rate_dps))
        self._state_t = t
        self._check_imu_sign(t)
        if self.filter is not None and self.cfg.heading_from_board:
            self._maybe_heading(t)

    def on_navsat(self, t: float, lat: float, lon: float, alt: float,
                  status_ok: bool, cov_en: Optional[np.ndarray] = None):
        """A GNSS fix. Returns the filter's UpdateResult, or None if not used."""
        if not (status_ok and math.isfinite(lat) and math.isfinite(lon)):
            return None
        if not self._board_fix_ok(t):
            return None
        alt = alt if math.isfinite(alt) else 0.0
        if self.plane is None:
            self._initialise(t, lat, lon, alt)
            return None
        e, n, _ = self.plane.to_enu(lat, lon, self.plane.h0)
        cov = self._fix_cov(cov_en)
        res = self.filter.update_position(e, n, cov)
        if res.relocalised:
            self.resets += 1
            self.log(f'shadow: {REJECT_STREAK_LIMIT} consecutive GNSS fixes rejected '
                     f'(last NIS {res.nis:.1f}); the filter is the suspect -- '
                     'RELOCALISED: position re-seeded to the fix, heading re-opened '
                     f'({self.resets} so far)')
        return res

    # ------------------------------------------------------------ clock
    def step(self, t: float) -> Optional[ShadowOutput]:
        """Propagate to `t`. Returns the estimate, or None when it must not be published."""
        if self.filter is None:
            self.stale_reason = 'no origin yet (waiting for a 3D GNSS fix)'
            return None
        prev, self._t = self._t, t
        if prev is None or t <= prev:
            return None
        dt = t - prev
        omega = self._fresh_gyro(t)
        twist = self._velocity(t)
        if omega is None or twist is None:
            self.stale_reason = 'no gyro' if omega is None else 'no velocity source'
            self._inflate_gap(dt)
            return None
        if dt > MAX_STEP_DT_S:
            self.gaps += 1
            self.log(f'shadow: {dt:.2f} s gap not integrated; covariance inflated')
            self._inflate_gap(dt)
            return None
        v_x, v_y, odom_omega = twist
        slipping = False
        if math.isfinite(odom_omega):
            b = self.filter.bias if self.filter.estimate_bias else 0.0
            slipping = self.slip.update(omega - b, odom_omega)
        if self._stationary(v_x, odom_omega, omega):
            self.filter.propagate(0.0, 0.0, omega, dt)
            r = self.filter.update_zero_velocity(omega, self._gyro_sample_std())
            if r.accepted:
                self.zupts += 1
            v_x = v_y = 0.0
        else:
            self.filter.propagate(v_x, v_y, omega, dt, slip=slipping)
        self.stale_reason = ''
        b = self.filter.bias if self.filter.estimate_bias else 0.0
        self._twist = (v_x, v_y, omega - b)
        return self.output(t, slipping)

    # ------------------------------------------------------------ outputs
    def output(self, t: float, slipping: bool = False) -> Optional[ShadowOutput]:
        if self.filter is None or self.stale_reason:
            return None
        f = self.filter
        lat, lon = self.plane.to_geodetic_on_plane(f.x, f.y)
        return ShadowOutput(t=t, x=f.x, y=f.y, yaw=f.yaw, cov=f.world_covariance(),
                            v_x=self._twist[0], v_y=self._twist[1], omega=self._twist[2],
                            lat=lat, lon=lon, compass_deg=se2.compass_deg_from_yaw(f.yaw),
                            position_std_m=f.position_std(), slipping=slipping)

    def extnav(self, out: Optional[ShadowOutput]) -> Optional[ShadowOutput]:
        """The estimate to uplink, or None with the refusal counted and named."""
        if not self.cfg.extnav_enabled:
            return None
        why = ''
        if out is None:
            why = f'no estimate ({self.stale_reason})'
        elif not out.position_std_m < self.cfg.extnav_max_std_m:
            why = (f'position std {out.position_std_m:.2f} m >= gate '
                   f'{self.cfg.extnav_max_std_m:.2f} m')
        elif out.slipping:
            why = 'track slip in progress'
        if why:
            self.extnav_refusals += 1
            self.extnav_last_refusal = why
            return None
        self.extnav_last_refusal = ''
        return out

    def health(self, t: float, out: Optional[ShadowOutput]) -> health.Verdict:
        nan = float('nan')
        s = self._state if self._state_fresh(t) else None
        return health.compare(
            out.lat if out else nan, out.lon if out else nan,
            out.compass_deg if out else nan,
            s['lat'] if s else nan, s['lon'] if s else nan, s['yaw_deg'] if s else nan,
            self.cfg.health_max_position_m, self.cfg.health_max_yaw_deg)

    def imu_sign_suspect(self) -> bool:
        """True when the IMU's yaw rate mostly disagrees in sign with the board's.

        The board's yaw rate is NED (clockwise positive); REP-103 gyro z is
        counter-clockwise positive, so agreement means opposite signs. A
        publisher that forwards FRD instead of FLU flips every turn this filter
        makes, and nothing else would catch it.
        """
        return self.imu_sign_checks >= 20 and \
            self.imu_sign_disagreements > 0.5 * self.imu_sign_checks

    # ------------------------------------------------------------ internals
    def _board_fix_ok(self, t: float) -> bool:
        return self._state_fresh(t) and self._state['gps_fix'] >= MIN_FIX_TYPE

    def _state_fresh(self, t: float) -> bool:
        return self._state is not None and t - self._state_t <= self.cfg.state_timeout_s

    def _fix_cov(self, cov_en) -> np.ndarray:
        if cov_en is not None:
            c = np.asarray(cov_en, float).reshape(2, 2)
            if np.all(np.isfinite(c)) and c[0, 0] > 0.0 and c[1, 1] > 0.0:
                return 0.5 * (c + c.T)
        return np.eye(2) * self.cfg.gnss_std_m ** 2

    def _initialise(self, t: float, lat: float, lon: float, alt: float) -> None:
        self.plane = LocalTangentPlane(lat, lon, alt)
        yaw_deg = self._state['yaw_deg'] if self._state else float('nan')
        if self.cfg.heading_from_board and math.isfinite(yaw_deg):
            yaw0, yaw_std = se2.yaw_from_compass_deg(yaw_deg), math.radians(self.cfg.heading_std_deg)
            src = f'board yaw {yaw_deg:.1f} deg'
        else:
            # Heading unknown: start at 0 with a std that admits any heading. The
            # invariant filter converges from there on GNSS + motion alone.
            yaw0, yaw_std = 0.0, math.pi
            src = 'unknown (std pi; converges on motion)'
        self.filter = RIEKF(0.0, 0.0, yaw0, pos_std=self.cfg.gnss_std_m,
                            yaw_std=yaw_std, noise=self.cfg.noise)
        self._t = t
        self.stale_reason = ''
        self.log(f'shadow: ORIGIN set at lat {lat:.8f} lon {lon:.8f} alt {alt:.2f} m '
                 f'(first fix with board fix_type >= {MIN_FIX_TYPE}); initial heading {src}')

    def _maybe_heading(self, t: float) -> None:
        s = self._state
        if not (s['ekf_ok'] and math.isfinite(s['yaw_deg'])):
            return
        if t - self._last_heading_t < self.cfg.heading_period_s:
            return
        self._last_heading_t = t
        self.filter.update_heading(se2.yaw_from_compass_deg(s['yaw_deg']),
                                   math.radians(self.cfg.heading_std_deg))

    def _fresh_gyro(self, t: float) -> Optional[float]:
        tg, w = self._gyro
        if tg is None or t - tg > self.cfg.imu_timeout_s or not math.isfinite(w):
            return None
        return w

    def _velocity(self, t: float):
        """(v_x, v_y, odom_omega) from wheels if fresh, else board groundspeed."""
        tw, vl, vr = self._wheels
        if tw is not None and t - tw <= self.cfg.velocity_timeout_s:
            v_x, v_y, w = icr_twist(vl, vr, self.icr)
            if math.isfinite(v_x):
                return v_x, v_y, w
        s = self._state
        if s is not None and t - self._state_t <= self.cfg.velocity_timeout_s \
                and math.isfinite(s['groundspeed']):
            return s['groundspeed'], 0.0, float('nan')
        return None

    def _stationary(self, v_x: float, odom_omega: float, gyro: float) -> bool:
        b = self.filter.bias if self.filter.estimate_bias else 0.0
        # A slow pivot under the gyro threshold would be read as bias; the
        # threshold sits at about twice a hobby gyro's noise floor for that reason.
        if abs(v_x) >= self.cfg.zupt_speed_mps or abs(gyro - b) >= self.cfg.zupt_gyro_rad_s:
            return False
        return not (math.isfinite(odom_omega) and abs(odom_omega) >= self.cfg.zupt_gyro_rad_s)

    def _gyro_sample_std(self) -> float:
        return self.cfg.noise.gyro * math.sqrt(self.cfg.rate_hz)

    def _inflate_gap(self, dt: float) -> None:
        self.filter.inflate(self.cfg.max_speed_mps * dt,
                            math.radians(self.cfg.max_yaw_rate_dps) * dt)

    def _check_imu_sign(self, t: float) -> None:
        rate = self._state['yaw_rate_dps']
        w = self._fresh_gyro(t)
        if w is None or not math.isfinite(rate) or abs(rate) < 10.0:
            return
        self.imu_sign_checks += 1
        if math.copysign(1.0, w) == math.copysign(1.0, rate):
            self.imu_sign_disagreements += 1
