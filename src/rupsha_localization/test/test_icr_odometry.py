"""ICR kinematics and the slip gate against rigid-body truth."""
import math

import numpy as np
import pytest

from rupsha_localization.icr_odometry import IcrParams, SlipGate, fit_icr, icr_twist


def _tracks(v_x, omega, p: IcrParams):
    """Rigid body, FLU: a contact line at lateral y moves forward at v_x - omega*y."""
    return v_x - omega * p.y_l, v_x - omega * p.y_r


def test_ideal_diff_drive_is_exact():
    p = IcrParams.from_track(0.6)
    assert icr_twist(1.0, 1.0, p) == pytest.approx((1.0, 0.0, 0.0))
    v_x, v_y, w = icr_twist(-0.3, 0.3, p)
    assert v_x == pytest.approx(0.0)
    # Right track faster turns LEFT: counter-clockwise, positive in REP-103.
    assert w == pytest.approx(0.6 / 0.6)
    v_x, _, w = icr_twist(0.5, 1.1, p)
    assert v_x == pytest.approx(0.8)
    assert w == pytest.approx(1.0)


def test_icr_twist_inverts_the_rigid_body_for_any_params():
    p = IcrParams(y_l=0.47, y_r=-0.39, x_v=0.06)
    rng = np.random.default_rng(0)
    for v_x, w in rng.uniform([-1.5, -1.0], [1.5, 1.0], (50, 2)):
        vl, vr = _tracks(v_x, w, p)
        out = icr_twist(vl, vr, p)
        assert out == pytest.approx((v_x, -p.x_v * w, w), abs=1e-12)


def test_expansion_factor_reduces_turn_rate():
    ideal = IcrParams.from_track(0.6, 1.0)
    skid = IcrParams.from_track(0.6, 1.5)
    assert icr_twist(-0.3, 0.3, skid)[2] == pytest.approx(icr_twist(-0.3, 0.3, ideal)[2] / 1.5)
    with pytest.raises(ValueError):
        IcrParams.from_track(0.6, 0.9)


def test_absent_track_speed_gives_absent_twist():
    assert all(math.isnan(v) for v in icr_twist(float('nan'), 1.0, IcrParams.from_track(0.6)))


def test_fit_recovers_known_icr_within_two_percent():
    true = IcrParams(y_l=0.55, y_r=-0.50, x_v=0.08)
    rng = np.random.default_rng(42)
    n = 2000
    v_x = rng.uniform(-1.0, 1.5, n)
    w = rng.uniform(-1.2, 1.2, n)
    vl, vr = _tracks(v_x, w, true)
    vl = vl + rng.normal(0.0, 0.01, n)
    vr = vr + rng.normal(0.0, 0.01, n)
    v_y = -true.x_v * w + rng.normal(0.0, 0.005, n)
    fit = fit_icr(vl, vr, v_x, w, v_y)
    for name in ('y_l', 'y_r', 'x_v'):
        got, want = getattr(fit.params, name), getattr(true, name)
        assert abs(got - want) <= 0.02 * abs(want), f'{name}: {got:.4f} vs {want:.4f}'
    assert fit.n_used > 1500


def test_fit_without_lateral_truth_reports_x_v_absent():
    true = IcrParams(y_l=0.5, y_r=-0.5)
    w = np.linspace(-1, 1, 200)
    v_x = np.full(200, 0.5)
    vl, vr = _tracks(v_x, w, true)
    fit = fit_icr(vl, vr, v_x, w)
    assert math.isnan(fit.params.x_v)
    assert fit.params.y_l == pytest.approx(0.5)


def test_fit_refuses_a_run_that_never_turns():
    v = np.ones(500)
    with pytest.raises(ValueError, match='turning samples'):
        fit_icr(v, v, v, np.zeros(500))


def test_slip_gate_quiet_on_clean_data():
    rng = np.random.default_rng(9)
    g = SlipGate(math.radians(15.0), k=3)
    for t in np.arange(0, 120, 0.05):
        w = 0.5 * math.sin(0.2 * t)
        g.update(w + rng.normal(0.0, 0.01), w + rng.normal(0.0, 0.02))
    assert g.events == 0 and not g.slipping


def test_slip_gate_triggers_on_injected_slip_and_clears():
    rng = np.random.default_rng(10)
    g = SlipGate(math.radians(15.0), k=3)
    flags = []
    for i in range(400):
        w = 0.2
        odom = w + (0.6 if 100 <= i < 160 else 0.0)   # tracks spin, body does not follow
        flags.append(g.update(w + rng.normal(0.0, 0.01), odom))
    flags = np.array(flags)
    assert g.events == 1
    assert flags[100 + 3:160].all()
    assert not flags[:100].any()
    assert not flags[160 + 3:].any()


def test_slip_gate_holds_on_absent_samples():
    g = SlipGate(0.2, k=2)
    g.update(0.0, 1.0)
    g.update(0.0, 1.0)
    assert g.slipping
    assert g.update(float('nan'), 0.0) is True
    assert g.absent == 1
