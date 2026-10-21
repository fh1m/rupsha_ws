"""The wiring decisions, without ROS: what reaches the filter and what gets published."""
import math

import numpy as np
import pytest

from rupsha_localization import health
from rupsha_localization.geodesy import LocalTangentPlane
from rupsha_localization.shadow import MAX_STEP_DT_S, Shadow, ShadowConfig

LAT, LON = 38.4064, -110.7919
DT = 0.05


def _shadow(**kw):
    logs = []
    s = Shadow(ShadowConfig(**kw), log=logs.append)
    return s, logs


def _state(s, t, fix=3, yaw_deg=90.0, speed=0.0, lat=LAT, lon=LON, ekf_ok=True, rate=float('nan')):
    s.on_state(t, lat, lon, yaw_deg, speed, fix, ekf_ok, rate)


def _origin(s, t=0.0, **kw):
    _state(s, t, **kw)
    s.on_imu(t, 0.0)
    s.on_navsat(t, LAT, LON, 1350.0, True)


def test_nothing_is_published_before_the_origin():
    s, _ = _shadow()
    s.on_imu(0.0, 0.0)
    _state(s, 0.0, fix=2)
    s.on_navsat(0.0, LAT, LON, 1350.0, True)      # a 2D fix is not an origin
    assert s.filter is None
    assert s.step(0.05) is None
    assert 'origin' in s.stale_reason
    assert s.health(0.05, None).status == health.ABSENT


def test_origin_is_the_first_3d_fix_and_is_logged():
    s, logs = _shadow()
    _origin(s)
    assert s.plane is not None and s.plane.lat0 == LAT
    assert any('ORIGIN' in m and f'{LAT:.8f}' in m for m in logs)
    # Board yaw 90 compass = east = ENU yaw 0.
    assert s.filter.yaw == pytest.approx(0.0, abs=1e-12)
    out = s.step(DT)
    assert out is not None and out.x == pytest.approx(0.0, abs=1e-9)


def test_origin_without_board_yaw_starts_with_unknown_heading():
    s, logs = _shadow()
    _origin(s, yaw_deg=float('nan'))
    assert s.filter.yaw_std() == pytest.approx(math.pi)
    assert any('unknown' in m for m in logs)


def test_absent_speed_is_not_propagated_as_zero():
    s, _ = _shadow()
    _origin(s)
    _state(s, 0.0, speed=float('nan'))
    before = s.filter.position_std()
    assert s.step(DT) is None
    assert s.stale_reason == 'no velocity source'
    assert s.filter.propagations == 0
    assert s.filter.position_std() > before        # time passed; we know less


def test_absent_gyro_stops_publishing():
    s, _ = _shadow()
    _origin(s)
    _state(s, 1.0, speed=1.0)
    assert s.step(1.0) is None and s.stale_reason == 'no gyro'


def test_a_gap_is_inflated_not_integrated():
    s, logs = _shadow()
    _origin(s)
    t = 2.0 * MAX_STEP_DT_S
    _state(s, t, speed=1.0)
    s.on_imu(t, 0.0)
    assert s.step(t) is None
    assert s.gaps == 1 and s.filter.propagations == 0
    assert any('gap' in m for m in logs)


def test_drives_east_on_groundspeed_and_gyro():
    s, _ = _shadow()
    _origin(s)
    t = 0.0
    for _ in range(100):
        t += DT
        _state(s, t, speed=1.0)
        s.on_imu(t, 0.0)
        out = s.step(t)
    assert out.x == pytest.approx(5.0, abs=1e-6)
    assert out.y == pytest.approx(0.0, abs=1e-6)
    e, n, _ = LocalTangentPlane(LAT, LON, 1350.0).to_enu(out.lat, out.lon, 1350.0)
    assert e == pytest.approx(5.0, abs=1e-3)


def test_standstill_learns_bias_through_zero_velocity_updates():
    rng = np.random.default_rng(1)
    s, _ = _shadow(heading_from_board=False)
    _origin(s, yaw_deg=float('nan'))
    t, bias = 0.0, 0.008
    for _ in range(1200):
        t += DT
        _state(s, t, speed=0.0, yaw_deg=float('nan'))
        s.on_imu(t, bias + rng.normal(0.0, s._gyro_sample_std()))
        s.step(t)
    assert s.zupts > 1000
    assert s.filter.bias == pytest.approx(bias, abs=0.0015)


def test_wheel_slip_is_flagged_and_blocks_extnav():
    s, _ = _shadow(extnav_enabled=True, extnav_max_std_m=50.0)
    _origin(s)
    t, outs = 0.0, []
    for _ in range(60):
        t += DT
        _state(s, t, speed=1.0)
        s.on_imu(t, 0.0)                          # the body does not turn
        s.on_wheels(t, 0.2, 1.2)                  # the tracks say it is turning hard
        outs.append(s.step(t))
    assert s.slip.slipping and s.slip.events == 1
    assert s.extnav(outs[-1]) is None
    assert 'slip' in s.extnav_last_refusal


def test_extnav_is_off_by_default_and_gated_when_on():
    off, _ = _shadow()
    _origin(off)
    _state(off, DT, speed=0.5)
    off.on_imu(DT, 0.0)
    assert off.extnav(off.step(DT)) is None
    assert off.extnav_refusals == 0               # disabled is not a refusal

    on, _ = _shadow(extnav_enabled=True, extnav_max_std_m=1.0)
    _origin(on)                                   # 1.5 m GNSS prior: over the gate
    _state(on, DT, speed=0.5)
    on.on_imu(DT, 0.0)
    out = on.step(DT)
    assert out is not None and on.extnav(out) is None
    assert on.extnav_refusals == 1 and 'gate' in on.extnav_last_refusal

    loose, _ = _shadow(extnav_enabled=True, extnav_max_std_m=3.0)
    _origin(loose)
    _state(loose, DT, speed=0.5)
    loose.on_imu(DT, 0.0)
    out = loose.step(DT)
    assert loose.extnav(out) is out


def test_streak_of_rejected_fixes_reseeds_position_loudly():
    s, logs = _shadow()
    _origin(s)
    t = 0.0
    far = LocalTangentPlane(LAT, LON, 1350.0).to_geodetic(80.0, 0.0, 0.0)
    for _fix in range(5):
        for _ in range(20):
            t += DT
            _state(s, t, speed=0.0)
            s.on_imu(t, 0.0)
            s.step(t)
        s.on_navsat(t, far[0], far[1], 1350.0, True)
    assert s.resets == 1
    assert s.filter.x == pytest.approx(80.0, abs=0.1)
    assert any('RELOCALISED' in m for m in logs)


def test_health_compares_against_the_board():
    s, _ = _shadow()
    _origin(s)
    _state(s, DT, speed=0.0)
    s.on_imu(DT, 0.0)
    out = s.step(DT)
    assert s.health(DT, out).status == health.OK
    far = LocalTangentPlane(LAT, LON).to_geodetic(0.0, 30.0, 0.0)
    _state(s, DT, lat=far[0], lon=far[1])
    assert s.health(DT, out).status == health.DIVERGED
    assert s.health(DT + 10.0, out).status == health.ABSENT   # board state went stale


def test_imu_sign_flip_is_detected():
    s, _ = _shadow()
    for i in range(30):
        t = i * DT
        s.on_imu(t, 0.5)                          # CCW in FLU
        _state(s, t, rate=+28.6)                  # board says CW: the IMU is FRD
    assert s.imu_sign_suspect()

    ok, _ = _shadow()
    for i in range(30):
        t = i * DT
        ok.on_imu(t, 0.5)
        _state(ok, t, rate=-28.6)
    assert not ok.imu_sign_suspect()
