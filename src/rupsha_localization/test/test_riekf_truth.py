"""The filter against a known trajectory: error bounds and consistency, not agreement.

Every run is seeded. A bound here is a claim about the estimator on this
simulated vehicle; a failure means the estimator changed, not the dice.
"""
import math

import numpy as np
import pytest
from loc_truth import Sim, chi2_band, pose_error, run

from rupsha_localization import se2
from rupsha_localization.riekf import CHI2_95, RIEKF

SETTLE_S = 30.0


def test_figure_eight_rms_position_error_under_one_metre():
    sim = Sim()
    r = run(sim, seed=11)
    keep = r.t >= SETTLE_S
    err = np.array([np.linalg.norm(Te[:2, 2] - Tt[:2, 2])
                    for Te, Tt, k in zip(r.est, r.truth, keep, strict=True) if k])
    rms = float(np.sqrt(np.mean(err ** 2)))
    # GNSS alone is 1.5 m per axis = 2.1 m radial; the filter must beat it well.
    assert rms < 1.0, f'RMS position error {rms:.3f} m'


def _anees(M, seed0=100):
    sim = Sim(duration_s=180.0)
    nees = []
    for seed in range(seed0, seed0 + M):
        r = run(sim, seed=seed)
        nees.append([float(pose_error(Te, Tt) @ np.linalg.solve(P, pose_error(Te, Tt)))
                     for Te, Tt, P in zip(r.est, r.truth, r.P, strict=True)])
    return np.mean(np.array(nees), axis=0)


def test_ungated_filter_is_consistent(monkeypatch):
    """The filter's own maths, with the gate's selection effect removed.

    Gated, the ANEES is ~12% high over 48 runs (refused fixes leave errors the
    covariance does not know about; see riekf.py). Ungated it is 3.04, so any
    excess here is the propagation or update being wrong, not the gate.
    """
    monkeypatch.setitem(CHI2_95, 1, math.inf)
    monkeypatch.setitem(CHI2_95, 2, math.inf)
    M = 12
    mean = float(np.mean(_anees(M, seed0=500)))
    lo, hi = chi2_band(3 * M)
    assert lo / M <= mean <= hi / M, f'ungated ANEES {mean:.2f}'


def test_figure_eight_average_nees_is_consistent():
    """Monte Carlo ANEES of the 3-dof pose error against the chi-square band.

    At each GNSS epoch the NEES is averaged over M independent runs, so
    M * ANEES ~ chi2(3M) for a consistent filter; the time-mean of that ANEES
    must lie in the two-sided 95% band. Too high is overconfidence (the gate
    starts refusing good fixes); too low is a filter that ignores its sensors.
    """
    M = 12
    anees = _anees(M)
    lo, hi = chi2_band(3 * M)
    mean = float(np.mean(anees))
    assert lo / M <= mean <= hi / M, f'ANEES {mean:.2f} outside [{lo / M:.2f}, {hi / M:.2f}]'
    # And no epoch is grossly overconfident.
    lo99, hi99 = chi2_band(3 * M, 0.005, 0.995)
    frac_out = float(np.mean(anees > hi99 / M))
    assert frac_out < 0.05, f'{frac_out:.1%} of epochs above the 99.5% point'


def test_gnss_outlier_is_rejected_and_counted():
    sim = Sim(duration_s=150.0)
    clean = run(sim, seed=5)
    bad = run(sim, seed=5, outlier_at_s=100.0, outlier_m=50.0)
    hit = [res for t, res in bad.gnss_results if abs(t - 100.0) < 1e-6]
    assert len(hit) == 1 and hit[0].accepted is False
    assert hit[0].nis > 100.0
    assert bad.filt.report()['position']['rejected'] == \
        clean.filt.report()['position']['rejected'] + 1
    # Losing one good fix (the one the outlier replaced) is all that happened.
    assert np.linalg.norm(bad.est[-1][:2, 2] - clean.est[-1][:2, 2]) < 0.05
    assert np.linalg.norm(bad.est[-1][:2, 2] - bad.truth[-1][:2, 2]) < 1.5


def test_a_rejected_update_leaves_the_state_untouched():
    f = RIEKF(x=1.0, y=2.0, yaw=0.3, pos_std=0.5)
    T0, P0, b0 = f.T.copy(), f.P.copy(), f.b
    r = f.update_position(51.0, 2.0, np.eye(2) * 1.5 ** 2)
    assert not r.accepted
    assert np.array_equal(f.T, T0) and np.array_equal(f.P, P0) and f.b == b0
    assert f.report()['position'] == {'accepted': 0, 'rejected': 1,
                                      'relocalisations': 0, 'streak': 1}


def test_clean_rejection_rate_is_about_the_gate_tail():
    """At 95% about 1 fix in 20 is refused even when nothing is wrong."""
    r = run(Sim(duration_s=240.0), seed=21)
    rep = r.filt.report()['position']
    frac = rep['rejected'] / (rep['accepted'] + rep['rejected'])
    assert frac < 0.12, rep


@pytest.mark.parametrize('offset_deg', [90.0, -90.0, 170.0])
def test_heading_converges_from_ninety_degrees_wrong(offset_deg):
    """No compass: GNSS + odometry alone, heading declared unknown (std pi).

    The invariant error recursion does not depend on the wrong estimate, so
    propagation never misleads it; the first-order UPDATE still can, on a
    1 m baseline with 1.5 m fixes. Measured over 40 seeds: about 1 run in 5
    locks itself out of the gate after a wrong first swing, and recovers only
    through relocalisation. So this runs several seeds and asserts every one
    converges, and that the yaw error is consistent with the yaw std.
    """
    sim = Sim(duration_s=120.0)
    for seed in range(30, 38):
        r = run(sim, seed=seed, yaw_offset=math.radians(offset_deg), yaw_std0=math.pi)
        yaw_err = math.degrees(abs(se2.wrap_pi(se2.yaw_of(r.est[-1]) - se2.yaw_of(r.truth[-1]))))
        pos_err = np.linalg.norm(r.est[-1][:2, 2] - r.truth[-1][:2, 2])
        sd = math.degrees(math.sqrt(r.P[-1][2, 2]))
        assert yaw_err < 6.0, f'seed {seed}: final yaw error {yaw_err:.2f} deg'
        assert yaw_err < 3.0 * sd, f'seed {seed}: yaw error {yaw_err:.2f} vs std {sd:.2f}'
        assert pos_err < 1.5, f'seed {seed}: final position error {pos_err:.2f} m'


def test_gate_lockout_relocalises_instead_of_diverging():
    f = RIEKF(x=0.0, y=0.0, yaw=0.0, pos_std=0.3, yaw_std=math.radians(2.0))
    for _ in range(4):
        assert not f.update_position(30.0, 0.0, np.eye(2)).relocalised
    r = f.update_position(30.0, 0.0, np.eye(2))
    assert not r.accepted and r.relocalised
    assert (f.x, f.y) == (30.0, 0.0)
    assert f.yaw_std() == pytest.approx(math.pi)
    rep = f.report()['position']
    assert rep == {'accepted': 0, 'rejected': 5, 'relocalisations': 1, 'streak': 0}
    assert f.update_position(30.5, 0.2, np.eye(2)).accepted


def test_error_recursion_is_exactly_log_linear():
    """Noise-free, xi_{k+1} = Ad(exp(-u dt)) xi_k holds EXACTLY, even for a
    90 degree, 10 m error -- the property the filter is chosen for."""
    dt = 0.05
    T_true = se2.make(3.0, -1.0, 0.4)
    T_est = T_true @ se2.exp([10.0, -4.0, math.pi / 2])
    for k in range(200):
        u = np.array([1.2, 0.0, 0.3 * math.sin(0.05 * k)]) * dt
        xi = pose_error(T_est, T_true)
        T_true = T_true @ se2.exp(u)
        T_est = T_est @ se2.exp(u)
        assert np.allclose(pose_error(T_est, T_true), se2.adjoint(se2.exp(-u)) @ xi, atol=1e-9)


def test_position_jacobian_is_state_independent():
    """Numerical H of z = R^T (p(T exp(xi)) - p) at wildly different states."""
    for T in (se2.make(0, 0, 0), se2.make(500.0, -300.0, 2.9), se2.make(-7, 1e3, -1.3)):
        H = np.zeros((2, 3))
        h = 1e-6
        for j in range(3):
            d = np.zeros(3)
            d[j] = h
            p = (T @ se2.exp(d))[:2, 2]
            H[:, j] = T[:2, :2].T @ (p - T[:2, 2]) / h
        assert np.allclose(H, [[1, 0, 0], [0, 1, 0]], atol=1e-5), H


def test_covariance_stays_symmetric_psd_and_counts_are_reported():
    r = run(Sim(duration_s=60.0), seed=2)
    P = r.filt.P
    assert np.allclose(P, P.T)
    assert np.linalg.eigvalsh(P).min() > 0.0
    rep = r.filt.report()
    assert set(rep) == {'position'}
    assert rep['position']['accepted'] > 0


def test_gate_uses_chi2_95():
    assert CHI2_95[1] == pytest.approx(3.8415, abs=1e-4)
    assert CHI2_95[2] == pytest.approx(5.9915, abs=1e-4)


def test_heading_update_and_its_gate():
    f = RIEKF(yaw=0.0, yaw_std=math.radians(5.0))
    assert f.update_heading(math.radians(4.0), math.radians(3.0)).accepted
    assert f.yaw > 0.0
    r = f.update_heading(math.radians(120.0), math.radians(3.0))
    assert not r.accepted and f.report()['heading']['rejected'] == 1


def test_heading_innovation_wraps():
    f = RIEKF(yaw=math.radians(179.0), yaw_std=math.radians(5.0))
    r = f.update_heading(math.radians(-179.0), math.radians(3.0))
    assert r.accepted and r.nis < 1.0


def test_zero_velocity_update_learns_the_gyro_bias():
    rng = np.random.default_rng(4)
    bias, dt, sd = 0.01, 0.05, 0.01
    f = RIEKF(bias_std=0.02)
    for _ in range(600):
        w = bias + rng.normal(0.0, sd)
        f.propagate(0.0, 0.0, w, dt)
        f.update_zero_velocity(w, sd)
    assert f.bias == pytest.approx(bias, abs=0.002)
    assert abs(f.yaw) < math.radians(0.5)


def test_absent_inputs_are_refused_not_zeroed():
    f = RIEKF()
    with pytest.raises(ValueError):
        f.propagate(float('nan'), 0.0, 0.0, 0.05)
    with pytest.raises(ValueError):
        f.propagate(1.0, 0.0, float('nan'), 0.05)
    with pytest.raises(ValueError):
        f.update_position(float('nan'), 0.0, np.eye(2))
    with pytest.raises(ValueError):
        f.update_heading(float('nan'), 0.1)
    with pytest.raises(ValueError):
        RIEKF(estimate_bias=False).update_zero_velocity(0.0, 0.01)
    assert f.propagations == 0 and f.counters == {}


def test_slip_inflates_position_uncertainty():
    a = RIEKF(pos_std=0.01, yaw_std=1e-4, estimate_bias=False)
    b = RIEKF(pos_std=0.01, yaw_std=1e-4, estimate_bias=False)
    for _ in range(40):
        a.propagate(1.0, 0.0, 0.0, 0.05)
        b.propagate(1.0, 0.0, 0.0, 0.05, slip=True)
    assert b.position_std() > 3.0 * a.position_std()
    assert b.slip_steps == 40 and a.slip_steps == 0
