"""The facade against a kinematic fake board, in simulated time.

Each test states the physical truth it checks (where the rover ended up, what the
board was told), not that a method returned what it returned.
"""

import inspect
import math
from dataclasses import dataclass

import pytest

from rupsha_control import geo
from rupsha_control.fc.fake import FakeRover
from rupsha_control.manifest import VERBS, Display, Outcome
from rupsha_control.rupsha import Rupsha


@dataclass
class Det:
    label: str
    tag_id: int
    bearing_deg: float
    range_m: float
    stamp: float
    confidence: float = 1.0


def make(**kw):
    fc = FakeRover(**kw)
    r = Rupsha(fc, clock=fc.clock, sleep=fc.sleep)
    return fc, r


def armed(**kw):
    fc, r = make(**kw)
    assert r.run('arm').success
    return fc, r


def test_every_manager_verb_has_a_method_with_the_manifest_fields():
    for verb, spec in VERBS.items():
        if spec['runs_on'] == 'executive':
            continue
        method = getattr(Rupsha, verb, None)
        assert method is not None, f'{verb} is in the manifest but unreachable on the facade'
        params = [p for p in inspect.signature(method).parameters if p != 'self']
        assert params == list(spec['fields']), f'{verb}: {params} != {spec["fields"]}'


def test_arm_refuses_below_the_firmware_floor():
    fc, r = make(firmware=(4, 5, 7))
    res = r.run('arm')
    assert res.outcome == Outcome.DENIED and 'floor' in res.message
    assert not fc.armed


def test_arm_refuses_when_firmware_unread():
    fc, r = make(firmware=())
    assert r.run('arm').outcome == Outcome.NOT_READY


def test_arm_surfaces_the_boards_reason():
    fc, r = make()
    fc.refuse_arm = 'PreArm: Radio failsafe on'
    res = r.run('arm')
    assert res.outcome == Outcome.DENIED and 'Radio failsafe' in res.message


def test_motion_refused_when_disarmed():
    fc, r = make()
    assert r.run('move_forward', duration=2.0).outcome == Outcome.NOT_READY
    assert not any(k == 'velocity_body' for k, _ in fc.sent)


def test_goto_gnss_arrives_inside_radius():
    fc, r = armed()
    lat, lon = geo.offset(*fc.origin, 15.0, 10.0)
    res = r.run('goto_gnss', lat=lat, lon=lon, radius=1.0)
    assert res.success, res.message
    assert geo.distance_m(*fc.latlon, lat, lon) <= 1.0
    assert fc.mode == 'GUIDED'


def test_goto_gnss_refuses_without_fix_and_zero_target():
    fc, r = armed()
    fc.gps_fix = 1
    assert r.run('goto_gnss', lat=1.0, lon=1.0).outcome == Outcome.NOT_READY
    fc.gps_fix = 3
    assert r.run('goto_gnss', lat=0.0, lon=0.0).outcome == Outcome.DENIED


def test_goto_times_out_and_holds_when_stuck():
    fc, r = armed()
    fc.stuck = True
    lat, lon = geo.offset(*fc.origin, 20.0, 0.0)
    res = r.run('goto_gnss', lat=lat, lon=lon, timeout=10.0)
    assert res.outcome == Outcome.TIMEOUT
    assert fc.mode == 'HOLD'
    assert res.error_value == pytest.approx(20.0, abs=0.5)


def test_move_forward_covers_speed_times_duration_and_ends_at_zero():
    fc, r = armed()
    res = r.run('move_forward', duration=4.0, speed=0.5)
    assert res.success
    assert fc.north == pytest.approx(2.0, abs=0.1)
    assert fc.sent[-1] == ('velocity_body', (0.0, 0.0))


def test_move_back_reverses_regardless_of_speed_sign():
    fc, r = armed()
    r.run('move_back', duration=2.0, speed=0.5)
    assert fc.north == pytest.approx(-1.0, abs=0.1)


def test_turn_to_takes_the_short_way_and_settles():
    fc, r = armed(yaw_deg=350.0)
    res = r.run('turn_to', heading=20.0)
    assert res.success
    assert abs(geo.heading_error(20.0, fc.yaw)) <= 3.0
    rates = [a[1] for k, a in fc.sent if k == 'velocity_body' and a != (0.0, 0.0)]
    assert rates and all(w > 0 for w in rates), 'should turn clockwise through north'


def test_turn_relative():
    fc, r = armed(yaw_deg=90.0)
    assert r.run('turn', angle=-45.0).success
    assert abs(geo.heading_error(45.0, fc.yaw)) <= 3.0


def test_drive_body_offset():
    fc, r = armed(yaw_deg=90.0)
    res = r.run('drive', distance=5.0)
    assert res.success, res.message
    assert fc.east == pytest.approx(5.0, abs=0.6)
    assert abs(fc.north) < 0.6


def test_verb_radius_widens_to_the_boards_wp_radius():
    fc, r = armed()
    r.board_wp_radius = 2.0
    real = fc._command

    def board_stops_at_wp_radius():
        if fc._pos_target is not None and math.hypot(fc._pos_target[0] - fc.north,
                                                     fc._pos_target[1] - fc.east) <= 2.0:
            return 0.0, 0.0
        return real()
    fc._command = board_stops_at_wp_radius
    res = r.run('drive', distance=6.0, radius=0.5, timeout=30.0)
    assert res.success, res.message
    assert 'WP_RADIUS' in res.message


def test_abort_stops_a_running_stream():
    fc, r = armed()
    calls = {'n': 0}
    real_sleep = fc.sleep

    def sleep(dt):
        calls['n'] += 1
        if calls['n'] == 10:
            r.request_abort()
        real_sleep(dt)
    r.sleep = sleep
    res = r.run('move_forward', duration=30.0)
    assert res.outcome == Outcome.ABORTED
    assert res.elapsed_s < 2.0
    assert fc.sent[-1] == ('velocity_body', (0.0, 0.0))


def test_link_loss_mid_stream_is_lost_not_success():
    fc, r = armed()
    real_sleep = fc.sleep

    def sleep(dt):
        if fc.clock.t > 1.0:
            fc.link_alive = False
        real_sleep(dt)
    r.sleep = sleep
    assert r.run('move_forward', duration=5.0).outcome == Outcome.LOST


def test_stop_holds_and_bypass_does_not_clear_a_pending_abort():
    fc, r = armed()
    r.request_abort()
    assert r.run('stop').success
    assert fc.mode == 'HOLD'
    assert r.aborted, 'a bypass verb must not swallow the abort it was sent with'


def test_board_setpoint_expiry_is_modelled():
    fc, r = armed()
    fc.set_mode('GUIDED')
    fc.velocity_body(1.0, 0.0)
    fc.sleep(5.0)
    assert fc.north == pytest.approx(3.0, abs=0.05), 'the board stops 3 s after the last setpoint'


def test_vision_approach_stops_at_standoff():
    fc, r = armed()
    post_n = 8.0

    def vision():
        rng = post_n - fc.north
        return [Det('tag', 3, 0.0, rng, fc.clock.t)]
    r.vision = vision
    res = r.run('vision_approach', tag_id=3, standoff=2.0)
    assert res.success, res.message
    assert post_n - fc.north == pytest.approx(2.0, abs=0.3)


def test_vision_approach_without_range_refuses():
    fc, r = armed()
    r.vision = lambda: [Det('tag', 3, 0.0, math.nan, fc.clock.t)]
    assert r.run('vision_approach', tag_id=3).outcome == Outcome.FAILED


def test_vision_approach_lost_target():
    fc, r = armed()
    r.vision = lambda: []
    assert r.run('vision_approach', tag_id=3).outcome == Outcome.LOST


def test_vision_approach_closes_in_on_a_tag_seen_once_past_its_detection_range():
    """Gazebo: one frame at ~7 m, then nothing until ~6 m. Stopping there never re-acquires."""
    fc, r = armed()
    post_n, decode_m = 8.0, 6.0
    first = {'t': None}

    def vision():
        rng = post_n - fc.north
        if first['t'] is None:
            first['t'] = fc.clock.t
        if rng <= decode_m or fc.clock.t == first['t']:
            return [Det('tag', 3, 0.0, rng, fc.clock.t)]
        return []
    r.vision = vision
    res = r.run('vision_approach', tag_id=3, standoff=2.0)
    assert res.success, res.message
    assert post_n - fc.north == pytest.approx(2.0, abs=0.3)


def test_coasting_on_a_lost_tag_never_crosses_the_standoff():
    fc, r = armed()
    post_n, standoff = 3.0, 2.0
    seen = {'n': 0}

    def vision():
        seen['n'] += 1
        return [Det('tag', 3, 0.0, post_n - fc.north, fc.clock.t)] if seen['n'] <= 3 else []
    r.vision = vision
    assert r.run('vision_approach', tag_id=3, standoff=standoff).outcome == Outcome.LOST
    fc.sleep(5.0)
    assert post_n - fc.north > standoff


def test_tag_zero_is_a_real_id():
    fc, r = armed()
    r.vision = lambda: [Det('tag', 0, 0.0, 1.0, fc.clock.t)]
    assert r.run('vision_approach', tag_id=0, standoff=2.0).success


def test_search_spiral_finds_a_post_it_drives_near():
    fc, r = armed()
    post = (6.0, 3.0)

    def vision():
        d = math.hypot(post[0] - fc.north, post[1] - fc.east)
        return [Det('tag', 7, 0.0, d, fc.clock.t)] if d < 3.0 else []
    r.vision = vision
    res = r.run('search_spiral', tag_id=7, radius=20.0)
    assert res.success, res.message


def test_search_spiral_gives_up_at_radius():
    fc, r = armed()
    res = r.run('search_spiral', radius=5.0, timeout=600.0)
    assert res.outcome == Outcome.FAILED
    assert math.hypot(fc.north, fc.east) < 12.0


def test_extnav_refused_until_viso_configured():
    fc, r = make()
    assert r.run('set_ekf_source', target_name='extnav').outcome == Outcome.DENIED
    fc.params['VISO_TYPE'] = 1.0
    assert r.run('set_ekf_source', target_name='extnav').success
    assert fc.ekf_source == 2


def test_led_states():
    fc, r = make()
    seen = []
    r.led_sink = seen.append
    assert r.run('led', target_name='arrived').success
    assert seen == [Display.ARRIVED]
    assert r.run('led', target_name='purple').outcome == Outcome.DENIED
