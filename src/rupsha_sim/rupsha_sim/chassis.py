"""The Ackermann body, in one table.

Lengths are Lazygo's, read from their xacro and README and written up in
.claude/context/platform/chassis.md. The steer limit is the xacro's ±45 deg.
Their bridge used 30 deg and the CAD note claims a turn inside 15 cm, which
on this wheelbase wants about 39 deg. Those disagree. The real radius is
unmeasured until the chassis is here, so the plant uses the xacro.
"""

from __future__ import annotations

import math

WHEELBASE_M = 0.12
TRACK_M = 0.094
WHEEL_RADIUS_M = 0.02
LENGTH_M = 0.20
WIDTH_M = 0.10
HEIGHT_M = 0.07
# xacro steer joints are ±pi/4. Not their bridge's 30 deg, and not the CAD note.
STEER_LIMIT_RAD = math.pi / 4
TPM = 1873.0
LINK_TIMEOUT_S = 0.5
STATE_HZ = 50
CONTROL_HZ = 200
# Nominal 3S pack. The sim has no cells to measure.
BATTERY_V = 11.1

BOARD_COMMAND = '/rupsha/board/command'
BOARD_STATE = '/rupsha/board/state'
DRIVE_COMMAND = '/drive_velocity/commands'
STEER_COMMAND = '/steer_position/commands'


def _bicycle(steer_rad: float) -> float:
    """Bicycle angle whose inner hinge stays on the xacro stop."""
    half = TRACK_M / 2.0
    cap = math.atan(WHEELBASE_M / (WHEELBASE_M / math.tan(STEER_LIMIT_RAD) + half))
    return max(-cap, min(cap, steer_rad))


def ackermann_angles(steer_rad: float) -> tuple[float, float]:
    """Front hinge angles (left, right) for a bicycle steer, positive left.

    The inner wheel is the sharper one. A bicycle angle of 45 deg would ask
    the inner hinge for about 59 deg, past the xacro stop, so the command is
    capped and the inner hinge lands on the limit.
    """
    steer = _bicycle(steer_rad)
    if abs(steer) < 1e-6:
        return 0.0, 0.0
    half = TRACK_M / 2.0
    radius = WHEELBASE_M / math.tan(steer)
    left = math.atan(WHEELBASE_M / (radius - half))
    right = math.atan(WHEELBASE_M / (radius + half))
    return left, right


def wheel_rad_s(speed_mms: float) -> float:
    """Rear-axle rate for a straight command. The encoder reads the average."""
    return (speed_mms / 1000.0) / WHEEL_RADIUS_M


def wheel_rates(speed_mms: float, steer_rad: float) -> tuple[float, float]:
    """Left and right rear rates. A locked axle will not follow the hinges."""
    v = speed_mms / 1000.0
    steer = _bicycle(steer_rad)
    if abs(steer) < 1e-6 or abs(v) < 1e-9:
        w = v / WHEEL_RADIUS_M
        return w, w
    radius = WHEELBASE_M / math.tan(steer)
    half = TRACK_M / 2.0
    left = v * (radius - half) / radius / WHEEL_RADIUS_M
    right = v * (radius + half) / radius / WHEEL_RADIUS_M
    return left, right


def ticks_from_axle(left_rad: float, right_rad: float) -> int:
    """1873 ticks per metre of the average of the two driven wheels."""
    arc_m = (left_rad + right_rad) / 2.0 * WHEEL_RADIUS_M
    return int(round(arc_m * TPM))
