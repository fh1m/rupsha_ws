"""The board contract's arithmetic, with no simulator.

The Gazebo test checks that a real plant follows these numbers. This one
checks the numbers themselves, so a wrong steer sign or a drifted tick
calibration fails in a second.
"""

import math

from rupsha_sim import chassis


def test_straight_steer_is_zero():
    assert chassis.ackermann_angles(0.0) == (0.0, 0.0)


def test_left_turn_sharpens_the_inner_wheel():
    left, right = chassis.ackermann_angles(0.30)
    assert left > right > 0.0
    assert left < chassis.STEER_LIMIT_RAD


def test_steer_is_clamped_to_the_xacro():
    left, right = chassis.ackermann_angles(2.0)
    assert left <= chassis.STEER_LIMIT_RAD + 1e-6
    assert right <= chassis.STEER_LIMIT_RAD + 1e-6
    # A finite turn. The centre is not allowed inside the track.
    assert math.isfinite(left) and math.isfinite(right)


def test_one_metre_of_axle_is_1873_ticks():
    # Both wheels roll one metre: angle = distance / radius.
    angle = 1.0 / chassis.WHEEL_RADIUS_M
    assert chassis.ticks_from_axle(angle, angle) == 1873


def test_wheel_rate_matches_the_commanded_speed():
    # 400 mm/s on a 0.02 m wheel.
    assert chassis.wheel_rad_s(400.0) == 20.0
    assert chassis.wheel_rates(400.0, 0.0) == (20.0, 20.0)


def test_left_turn_slows_the_inner_rear_wheel():
    left, right = chassis.wheel_rates(400.0, 0.30)
    assert 0.0 < left < right
