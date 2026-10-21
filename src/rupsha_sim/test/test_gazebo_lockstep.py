"""Gazebo Harmonic + ArduRover SITL (JSON, lockstep), driven through the real driver.

Truth is Gazebo's own model pose (rupsha_sim.gazebo.truth), never the EKF the
rover steered by. Each test pins a failure that was real on 2026-10-21:

  * forward drifted 50 deg off heading       -> a z-up IMU negated yaw rate
  * forward crawled at 0.55 m/s on 1.0       -> CRUISE_THROTTLE set without CRUISE_SPEED
  * turn_to overshot 140 deg then spun       -> same IMU sign, rate loop ran away

    source install/setup.bash && pytest -m gazebo src/rupsha_sim
"""

import math
import threading
import time

import pytest

from rupsha_control.manifest import Outcome
from rupsha_sim import gazebo
from rupsha_sim.sitl import find_ardupilot

pytestmark = [
    pytest.mark.gazebo,
    pytest.mark.skipif(find_ardupilot() is None, reason='ArduRover SITL not built'),
    pytest.mark.skipif(gazebo.available() is not None, reason=str(gazebo.available())),
]


def heading_error(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def test_forward_holds_heading_and_speed(gz_rover):
    fc, r = gz_rover
    e0, n0, h0 = gazebo.truth()
    res = r.run('move_forward', duration=4.0, speed=1.0)
    assert res.outcome == Outcome.SUCCEEDED, res.message
    speed = fc.telemetry().groundspeed
    e1, n1, h1 = gazebo.truth()
    travelled = math.hypot(e1 - e0, n1 - n0)
    track = math.degrees(math.atan2(e1 - e0, n1 - n0)) % 360.0
    assert abs(heading_error(h1, h0)) < 3.0, f'heading drifted {h0:.1f} -> {h1:.1f}'
    assert abs(heading_error(track, h0)) < 5.0, f'tracked {track:.1f}, facing {h0:.1f}'
    assert 2.5 < travelled < 4.5, f'{travelled:.2f} m in 4 s at 1.0 m/s'
    assert speed == pytest.approx(1.0, abs=0.15)


def test_turn_to_settles_without_overshoot(gz_rover):
    fc, r = gz_rover
    target = (gazebo.truth()[2] + 90.0) % 360.0
    worst = [0.0]
    stop = threading.Event()

    def watch():
        while not stop.is_set():
            # overshoot = going past the target in the direction of the turn
            worst[0] = max(worst[0], heading_error(fc.telemetry().yaw_deg, target))
            time.sleep(0.05)
    threading.Thread(target=watch, daemon=True).start()
    res = r.run('turn_to', heading=target, timeout=15.0)
    stop.set()
    assert res.outcome == Outcome.SUCCEEDED, res.message
    assert res.elapsed_s < 8.0
    assert worst[0] < 10.0, f'overshot by {worst[0]:.1f} deg'
    assert abs(heading_error(gazebo.truth()[2], target)) < 5.0


def test_board_yaw_agrees_with_gazebo(gz_rover):
    fc, _ = gz_rover
    time.sleep(1.0)
    assert abs(heading_error(fc.telemetry().yaw_deg, gazebo.truth()[2])) < 5.0
