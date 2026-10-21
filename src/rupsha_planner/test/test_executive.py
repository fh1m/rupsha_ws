"""The executive against the facade on a FakeRover, in simulated time.

Truth = where the fake rover physically ended up and what the scorer would see,
never "the executive reported success".
"""

import math
from dataclasses import dataclass

from rupsha_control import geo
from rupsha_control.fc.fake import FakeRover
from rupsha_control.manifest import AutonomyState as S
from rupsha_control.manifest import Display
from rupsha_control.rupsha import Rupsha
from rupsha_planner import routing
from rupsha_planner.executive import ExecConfig, Executive
from rupsha_planner.ports import FacadePort
from rupsha_planner.stuck import StuckConfig, StuckDetector


@dataclass
class Det:
    label: str
    tag_id: int
    bearing_deg: float
    range_m: float
    stamp: float
    confidence: float = 1.0


def world(posts=(), objects=(), fov_deg=35.0, max_range=6.0, **kw):
    """A fake rover and a fake camera that sees posts/objects inside a cone."""
    fc = FakeRover(**kw)
    r = Rupsha(fc, clock=fc.clock, sleep=fc.sleep)
    leds = []
    r.led_sink = leds.append

    def vision():
        out = []
        for label, ident, n, e in [('tag', t, n, e) for t, n, e in posts] + \
                                  [(c, -1, n, e) for c, n, e in objects]:
            dn, de = n - fc.north, e - fc.east
            rng = math.hypot(dn, de)
            bearing = geo.heading_error(math.degrees(math.atan2(de, dn)), fc.yaw)
            if rng <= max_range and abs(bearing) <= fov_deg:
                out.append(Det(label, ident, bearing, rng, fc.clock.t))
        return out
    r.vision = vision
    assert r.run('arm').success
    port = FacadePort(r)
    ex = Executive(port, ExecConfig(arrive_dwell_s=1.0))
    return fc, r, ex, leds


def ll(fc, n, e):
    return geo.offset(*fc.origin, n, e)


def test_two_position_legs_end_inside_radius():
    fc, r, ex, leds = world()
    ex.add_position_leg(*ll(fc, 20, 0), radius=2.0)
    ex.add_position_leg(*ll(fc, 20, 15), radius=2.0)
    res = ex.run()
    assert res.ok, res.message
    assert res.legs_done == 2
    assert math.hypot(fc.north - 20, fc.east - 15) <= 2.0
    assert ex.state == S.IDLE
    assert leds.count(Display.ARRIVED) == 2 and leds[0] == Display.AUTONOMY


def test_marker_leg_searches_approaches_and_verifies():
    post = (4, 26.0, 9.0)           # GNSS point is 5 m off the real post
    fc, r, ex, _ = world(posts=[post])
    ex.add_marker_leg(*ll(fc, 22, 6), tag_id=4, radius=15.0)
    res = ex.run()
    assert res.ok, (res.message, ex.history)
    rng = math.hypot(post[1] - fc.north, post[2] - fc.east)
    assert rng <= ex.cfg.approach_standoff_m + 0.5
    states = [s for s, _ in ex.history]
    assert 'SEARCH_PATTERN' in states or 'APPROACHING_MARKER' in states
    assert 'VERIFYING_MARKER' in states


def test_wrong_tag_is_not_success():
    fc, r, ex, _ = world(posts=[(9, 12.0, 0.0)])
    ex.cfg.search_timeout_s = 60.0
    ex.add_marker_leg(*ll(fc, 10, 0), tag_id=4, radius=6.0)
    res = ex.run()
    assert not res.ok


def test_object_leg():
    fc, r, ex, _ = world(objects=[('mallet', 14.0, -6.0)])
    ex.add_object_leg(*ll(fc, 12, -3), target_name='mallet', radius=10.0)
    res = ex.run()
    assert res.ok, (res.message, ex.history)
    assert math.hypot(14 - fc.north, -6 - fc.east) <= 2.5


def test_declared_obstacle_is_routed_around():
    fc, r, ex, _ = world()
    fc.rocks = [(10.0, 0.0, 1.5)]
    rock_ll = ll(fc, 10, 0)
    ex.add_obstacle(*rock_ll, radius=1.5)
    ex.add_position_leg(*ll(fc, 20, 0), radius=2.0)
    track = []
    real = fc.sleep

    def sleep(dt):
        real(dt)
        track.append((fc.north, fc.east))
    fc.sleep = sleep
    r.sleep = sleep
    res = ex.run()
    assert res.ok, res.message
    assert min(math.hypot(n - 10, e) for n, e in track) >= 1.5
    assert not any(s == 'REVERSING' for s, _ in ex.history), 'a known obstacle should never cause a stuck'


def test_undeclared_rock_triggers_ladder_then_learned_detour():
    fc, r, ex, _ = world()
    fc.rocks = [(10.0, 0.0, 1.0)]
    ex.add_position_leg(*ll(fc, 20, 0), radius=2.0)
    res = ex.run()
    assert res.ok, (res.message, ex.history)
    states = [s for s, _ in ex.history]
    assert 'REVERSING' in states
    assert ex.learned, 'the stuck point should become an obstacle'
    assert math.hypot(fc.north - 20, fc.east) <= 2.0


def test_permanently_stuck_exhausts_the_ladder_and_says_so():
    fc, r, ex, _ = world()
    ex.add_position_leg(*ll(fc, 20, 0), radius=2.0)
    fc.stuck = True
    res = ex.run()
    assert not res.ok
    assert ex.state == S.STUCK
    assert 'ladder' in res.message
    reversings = [d for s, d in ex.history if s == 'REVERSING']
    assert len(reversings) == 3
    assert fc.mode == 'HOLD'


def test_post_in_view_at_leg_start_backs_off_first():
    fc, r, ex, _ = world(posts=[(2, 1.0, 0.0)])
    ex.add_marker_leg(*ll(fc, 1, 0), tag_id=2, radius=8.0)
    ex.run()
    assert ex.history[0][0] == 'REVERSING'


def test_operator_stop_ends_the_run_and_holds():
    fc, r, ex, _ = world()
    ex.add_position_leg(*ll(fc, 60, 0), radius=2.0)
    real = r.sleep

    def sleep(dt):
        real(dt)
        if fc.clock.t > 5.0:
            ex.request_stop()
    r.sleep = sleep
    res = ex.run()
    assert not res.ok and 'operator' in res.message
    assert fc.north < 20
    assert fc.mode == 'HOLD'


def test_legs_frozen_while_running_and_zero_rejected():
    fc, r, ex, _ = world()
    assert ex.add_position_leg(0.0, 0.0, 2.0)[0] is False
    ex.state = S.NAVIGATING
    assert ex.add_position_leg(*ll(fc, 5, 5), 2.0)[0] is False
    assert ex.clear_legs()[0] is False


def test_stuck_detector_grace_and_absent_telemetry():
    det = StuckDetector(StuckConfig(), now=0.0)
    assert not any(det.update(t, 0.0, 0.0) for t in (0.5, 1.0, 1.5))
    assert not any(det.update(t, math.nan, 0.0) for t in (3, 4, 5, 6))
    hits = [det.update(t, 0.0, 0.0) for t in (7, 8, 9)]
    assert hits == [False, False, True]


def test_router_clears_every_obstacle():
    obs = [((10.0, 0.0), 2.0), ((20.0, 1.0), 1.5)]
    pts = routing.plan_local((0.0, 0.0), (30.0, 0.0), obs)
    path = [(0.0, 0.0)] + pts
    for a, b in zip(path, path[1:], strict=False):
        for c, rad in obs:
            q, _ = routing._closest_on_segment(c, a, b)
            assert math.dist(q, c) >= rad, (a, b, c)
    assert pts[-1] == (30.0, 0.0)
