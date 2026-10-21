"""Skid-steer odometry through instantaneous centres of rotation, and a slip gate.

THE MODEL. Mandow, Martinez, Morales, Blanco, Garcia-Cerezo & Gonzalez,
"Experimental kinematics for wheeled skid-steer mobile robots", IROS 2007
(after Martinez et al., "Approximating kinematics for tracked mobile robots",
IJRR 2005). A skid-steer vehicle does not turn about the midpoint of its
tracks: the tracks slide, and the effective left/right contact lines sit
further out than the geometric ones. Three numbers capture that -- the lateral
coordinates y_l, y_r of the left and right track ICRs and the longitudinal
coordinate x_v of the vehicle ICR -- and they are fitted, not measured.

SIGNS, derived rather than copied. Body frame is REP-103 FLU (y LEFT). A point
of the rigid body at lateral offset y moves forward at v_x - omega*y, so each
track's ground-contact speed is vL = v_x - omega*y_l, vR = v_x - omega*y_r.
Solving:

    v_x   = (vR*y_l - vL*y_r) / (y_l - y_r)
    omega = (vR - vL) / (y_l - y_r)
    v_y   = -x_v * omega              (x_v = -v_y/omega, Martinez 2005)

The commonly quoted form "omega = -(vR - vL)/(y_l - y_r)" belongs to a frame
with y to the RIGHT; with y_l > 0 here it turns the rover the wrong way. The
diff-drive truth test pins the sign: right track faster must be CCW.

DEFAULTS ARE UNMEASURED. y_l = +chi*track/2, y_r = -chi*track/2, x_v = 0 with
chi = 1.0 is ideal differential drive. Real skid-steer chi is typically
1.2..2 and depends on surface. Until `fit_icr` has been run on logged ground
truth for THIS rover on THIS surface, the parameters are BUILT, not FIELD.

Pure numpy. No ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class IcrParams:
    y_l: float
    y_r: float
    x_v: float = 0.0

    @classmethod
    def from_track(cls, track_m: float, chi: float = 1.0) -> 'IcrParams':
        if not (math.isfinite(track_m) and track_m > 0.0):
            raise ValueError(f'IcrParams: track must be positive, got {track_m}')
        if not (math.isfinite(chi) and chi >= 1.0):
            raise ValueError(f'IcrParams: expansion factor chi must be >= 1, got {chi} '
                             '(tracks cannot turn tighter than their geometry)')
        half = 0.5 * track_m * chi
        return cls(y_l=half, y_r=-half, x_v=0.0)

    def __post_init__(self):
        if not (self.y_l - self.y_r > 0.0):
            raise ValueError(f'IcrParams: need y_l > y_r (FLU), got {self.y_l}, {self.y_r}')


def icr_twist(v_left: float, v_right: float,
              p: IcrParams) -> Tuple[float, float, float]:
    """Track speeds (m/s) -> body twist (v_x, v_y, omega). NaN in, NaN out."""
    if not (math.isfinite(v_left) and math.isfinite(v_right)):
        nan = float('nan')
        return nan, nan, nan
    d = p.y_l - p.y_r
    v_x = (v_right * p.y_l - v_left * p.y_r) / d
    omega = (v_right - v_left) / d
    v_y = -p.x_v * omega
    return v_x, v_y, omega


@dataclass(frozen=True)
class IcrFit:
    params: IcrParams
    n_used: int
    rms_residual: float      # m/s, over the track-speed equations


# Samples turning slower than this carry no information about y_l, y_r, x_v
# (each enters multiplied by omega) and only add noise to the fit.
MIN_FIT_OMEGA = 0.05         # rad/s
MIN_FIT_SAMPLES = 20


def fit_icr(v_left: Sequence[float], v_right: Sequence[float],
            v_x: Sequence[float], omega: Sequence[float],
            v_y: Optional[Sequence[float]] = None) -> IcrFit:
    """Least-squares (y_l, y_r, x_v) from logged track speeds and ground truth.

    From the contact equations, per sample:
        omega * y_l = v_x - vL
        omega * y_r = v_x - vR
        omega * x_v = -v_y
    Each is a one-parameter regression through the origin. x_v needs the
    ground-truth LATERAL velocity; without `v_y` it is not observable and is
    returned as NaN -- absent, not zero.
    """
    vl = np.asarray(v_left, float)
    vr = np.asarray(v_right, float)
    vx = np.asarray(v_x, float)
    w = np.asarray(omega, float)
    ok = np.isfinite(vl) & np.isfinite(vr) & np.isfinite(vx) & np.isfinite(w)
    ok &= np.abs(w) >= MIN_FIT_OMEGA
    n = int(ok.sum())
    if n < MIN_FIT_SAMPLES:
        raise ValueError(f'fit_icr: only {n} usable turning samples '
                         f'(|omega| >= {MIN_FIT_OMEGA} rad/s, finite); need '
                         f'{MIN_FIT_SAMPLES}. Log a run that actually turns.')
    ww = w[ok]
    den = float(ww @ ww)
    y_l = float(ww @ (vx[ok] - vl[ok])) / den
    y_r = float(ww @ (vx[ok] - vr[ok])) / den
    x_v = float('nan')
    if v_y is not None:
        vy = np.asarray(v_y, float)
        okv = ok & np.isfinite(vy)
        if okv.sum() >= MIN_FIT_SAMPLES:
            wv = w[okv]
            x_v = float(wv @ (-vy[okv])) / float(wv @ wv)
    res = np.concatenate([ww * y_l - (vx[ok] - vl[ok]), ww * y_r - (vx[ok] - vr[ok])])
    if not y_l - y_r > 0.0:
        raise ValueError(f'fit_icr: fitted y_l={y_l:.3f} <= y_r={y_r:.3f}; '
                         'track speeds are probably swapped or sign-flipped')
    return IcrFit(params=IcrParams(y_l, y_r, x_v), n_used=n,
                  rms_residual=float(np.sqrt(np.mean(res ** 2))))


class SlipGate:
    """Flags track slip when the gyro and the tracks disagree about yaw rate.

    The gyro measures how the body actually turns; the tracks measure how they
    were driven. On a sliding surface the second runs away from the first.
    Raised after `k` consecutive samples with |gyro - odom| > threshold, and
    cleared only after `k` consecutive clean ones, so a single noisy sample
    neither raises nor drops it. A NaN on either side is not evidence: the
    state is held and the sample is counted as absent.
    """

    def __init__(self, threshold_rad_s: float, k: int = 3):
        if not (threshold_rad_s > 0.0 and k >= 1):
            raise ValueError(f'SlipGate: threshold > 0 and k >= 1 required, '
                             f'got {threshold_rad_s}, {k}')
        self.threshold = float(threshold_rad_s)
        self.k = int(k)
        self.slipping = False
        self._bad = 0
        self._good = 0
        self.events = 0
        self.absent = 0
        self.last_diff = float('nan')

    def update(self, gyro_omega: float, odom_omega: float) -> bool:
        if not (math.isfinite(gyro_omega) and math.isfinite(odom_omega)):
            self.absent += 1
            self.last_diff = float('nan')
            return self.slipping
        diff = abs(gyro_omega - odom_omega)
        self.last_diff = diff
        if diff > self.threshold:
            self._bad += 1
            self._good = 0
            if not self.slipping and self._bad >= self.k:
                self.slipping = True
                self.events += 1
        else:
            self._good += 1
            self._bad = 0
            if self.slipping and self._good >= self.k:
                self.slipping = False
        return self.slipping
