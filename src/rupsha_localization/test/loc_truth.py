"""A known trajectory and seeded noisy sensors: the ground truth the filter is held to.

Figure-eight: constant forward speed, yaw rate omega0*sin(2*pi*t/period) with
omega0 = 2*pi^2/period so each half period turns exactly one full loop. Truth
is integrated on SE(2) with the exact exponential at `truth_hz` (finer than the
filter), so the filter's piecewise-constant twist is itself an approximation
of the truth, as it is on the vehicle.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from rupsha_localization import se2
from rupsha_localization.riekf import RIEKF, ProcessNoise


@dataclass
class Sim:
    duration_s: float = 240.0
    speed: float = 1.0
    period_s: float = 120.0
    filter_hz: float = 20.0
    truth_hz: float = 100.0
    gnss_hz: float = 1.0
    gnss_std: float = 1.5
    speed_std: float = 0.05          # per filter-rate sample
    gyro_std: float = 0.01           # rad/s per filter-rate sample
    gyro_bias: float = 0.005         # rad/s
    yaw0: float = 0.3


@dataclass
class Run:
    t: np.ndarray
    truth: List[np.ndarray]
    est: List[np.ndarray]
    P: List[np.ndarray]
    gnss_results: list = field(default_factory=list)
    filt: Optional[RIEKF] = None


def matched_noise(sim: Sim) -> ProcessNoise:
    """Densities equal to what the simulator injects: a consistent filter."""
    r = sim.filter_hz
    return ProcessNoise(speed=sim.speed_std / math.sqrt(r), lateral=0.002,
                        gyro=sim.gyro_std / math.sqrt(r), bias_walk=1e-5)


def omega_true(sim: Sim, t: float) -> float:
    w0 = 2.0 * math.pi ** 2 / sim.period_s
    return w0 * math.sin(2.0 * math.pi * t / sim.period_s)


def run(sim: Sim, seed: int, yaw_offset: float = 0.0, yaw_std0: Optional[float] = None,
        outlier_at_s: Optional[float] = None, outlier_m: float = 50.0,
        bias_std0: float = 0.01) -> Run:
    rng = np.random.default_rng(seed)
    dt = 1.0 / sim.filter_hz
    sub = int(round(sim.truth_hz / sim.filter_hz))
    n = int(round(sim.duration_s * sim.filter_hz))
    gnss_every = int(round(sim.filter_hz / sim.gnss_hz))
    T = se2.make(0.0, 0.0, sim.yaw0)

    fix0 = T[:2, 2] + rng.normal(0.0, sim.gnss_std, 2)
    if yaw_std0 is None:
        yaw_std0 = math.radians(5.0)
        yaw_init = sim.yaw0 + rng.normal(0.0, yaw_std0)
    else:
        yaw_init = sim.yaw0 + yaw_offset
    f = RIEKF(fix0[0], fix0[1], yaw_init, pos_std=sim.gnss_std, yaw_std=yaw_std0,
              estimate_bias=True, bias=0.0, bias_std=bias_std0, noise=matched_noise(sim))

    ts, truth, est, Ps, results = [], [], [], [], []
    t = 0.0
    for k in range(1, n + 1):
        # Truth over one filter step, finely; the sensors see its average twist.
        w_acc = 0.0
        for j in range(sub):
            tj = t + (j + 0.5) * dt / sub
            w = omega_true(sim, tj)
            w_acc += w
            T = se2.compose(T, se2.exp(np.array([sim.speed, 0.0, w]) * dt / sub))
        w_mean = w_acc / sub
        t += dt
        v_m = sim.speed + rng.normal(0.0, sim.speed_std)
        g_m = w_mean + sim.gyro_bias + rng.normal(0.0, sim.gyro_std)
        f.propagate(v_m, 0.0, g_m, dt)
        if k % gnss_every == 0:
            y = T[:2, 2] + rng.normal(0.0, sim.gnss_std, 2)
            if outlier_at_s is not None and abs(t - outlier_at_s) < 0.5 * dt:
                y = y + np.array([outlier_m, 0.0])
            res = f.update_position(y[0], y[1], np.eye(2) * sim.gnss_std ** 2)
            results.append((t, res))
            ts.append(t)
            truth.append(T.copy())
            est.append(f.T.copy())
            Ps.append(f.P[:3, :3].copy())
    return Run(np.array(ts), truth, est, Ps, results, f)


def pose_error(T_est: np.ndarray, T_true: np.ndarray) -> np.ndarray:
    """xi with T_true = T_est exp(xi): the filter's own error definition."""
    return se2.log(se2.inverse(T_est) @ T_true)


def chi2_band(dof: int, p_lo: float = 0.025, p_hi: float = 0.975):
    """Wilson-Hilferty chi-square quantiles; accurate to <1% for dof >= 10."""
    z = {0.025: -1.959963984540054, 0.975: 1.959963984540054,
         0.005: -2.5758293035489, 0.995: 2.5758293035489}
    def q(p):
        c = 2.0 / (9.0 * dof)
        return dof * (1.0 - c + z[p] * math.sqrt(c)) ** 3
    return q(p_lo), q(p_hi)
