"""Shadow-versus-board cross-check: does the Pi agree with the Pixhawk?

The board's EKF3 is the authority. This verdict does not overrule it; it
raises a flag that a human or the executive acts on. Three outcomes:

  OK        both sides present and within thresholds
  DIVERGED  both present and at least one divergence over its threshold
  ABSENT    either side has a NaN in lat, lon or yaw -- there is nothing to
            compare, and "no evidence of divergence" must not read as OK

Thresholds are the caller's. The node's defaults are tied to GNSS: two
independent estimates each fed a 1.5 m-sigma fix can differ by several metres
without either being wrong.

Pure function. No ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from rupsha_localization.geodesy import LocalTangentPlane

OK = 'OK'
DIVERGED = 'DIVERGED'
ABSENT = 'ABSENT'


@dataclass(frozen=True)
class Verdict:
    status: str
    position_m: float        # NaN when ABSENT
    yaw_deg: float           # signed shadow - board, (-180, 180]; NaN when ABSENT
    detail: str


def _yaw_diff_deg(a: float, b: float) -> float:
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


def compare(shadow_lat: float, shadow_lon: float, shadow_yaw_deg: float,
            board_lat: float, board_lon: float, board_yaw_deg: float,
            max_position_m: float, max_yaw_deg: float) -> Verdict:
    """Yaws are compass degrees (clockwise from north), the board's convention."""
    vals = {'shadow_lat': shadow_lat, 'shadow_lon': shadow_lon,
            'shadow_yaw': shadow_yaw_deg, 'board_lat': board_lat,
            'board_lon': board_lon, 'board_yaw': board_yaw_deg}
    missing = [k for k, v in vals.items() if v is None or not math.isfinite(float(v))]
    if missing:
        return Verdict(ABSENT, float('nan'), float('nan'),
                       'absent: ' + ', '.join(missing))
    plane = LocalTangentPlane(float(board_lat), float(board_lon))
    e, n, _ = plane.to_enu(float(shadow_lat), float(shadow_lon))
    dist = math.hypot(e, n)
    dyaw = _yaw_diff_deg(float(shadow_yaw_deg), float(board_yaw_deg))
    over = []
    if dist > max_position_m:
        over.append(f'position {dist:.2f} m > {max_position_m:.2f}')
    if abs(dyaw) > max_yaw_deg:
        over.append(f'yaw {dyaw:+.1f} deg > {max_yaw_deg:.1f}')
    if over:
        return Verdict(DIVERGED, dist, dyaw, 'diverged: ' + '; '.join(over))
    return Verdict(OK, dist, dyaw, f'position {dist:.2f} m, yaw {dyaw:+.1f} deg')
