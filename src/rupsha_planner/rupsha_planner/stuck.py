"""Stuck detection and the recovery ladder.

Detection is MissouriMRDT Autonomy_Software's: while a motion verb is commanding,
sample |speed| and |yaw rate| every `check_period_s`; if both stay under their
thresholds for `checks` consecutive samples, the rover is stuck. A grace period
after each verb starts keeps the spin-up from reading as stuck.

The ladder is theirs too, with one addition: each rung reverses, the 2nd and
3rd first rotate +65 / -65 degrees, and the 4th gives up. A rung that moves the
rover `escape_m` from where it got stuck ends the ladder early. The addition: the
stuck point becomes a learned obstacle, so the resumed leg routes around it
instead of driving back into the same rock.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class StuckConfig:
    speed_mps: float = 0.10
    yaw_rate_dps: float = 4.0
    check_period_s: float = 1.0
    checks: int = 3
    grace_s: float = 2.0
    reverse_s: float = 3.0
    reverse_speed: float = 0.5
    rotate_deg: float = 65.0
    escape_m: float = 1.0
    same_place_m: float = 3.0
    obstacle_radius_m: float = 1.0
    obstacle_ahead_m: float = 1.0


class StuckDetector:

    def __init__(self, cfg: StuckConfig, now: float):
        self.cfg = cfg
        self.reset(now)

    def reset(self, now: float) -> None:
        self._start = now
        self._last_check = now
        self._still = 0

    def update(self, now: float, speed: float, yaw_rate: float) -> bool:
        c = self.cfg
        if now - self._start < c.grace_s or now - self._last_check < c.check_period_s:
            return False
        self._last_check = now
        if not (math.isfinite(speed) and math.isfinite(yaw_rate)):
            return False                       # absent telemetry is not evidence of stuck
        if abs(speed) < c.speed_mps and abs(yaw_rate) < c.yaw_rate_dps:
            self._still += 1
        else:
            self._still = 0
        return self._still >= c.checks


def ladder(cfg: StuckConfig, rung: int):
    """Verb sequence for rung 0..2, or None when the ladder is exhausted."""
    back = ('move_back', {'duration': cfg.reverse_s, 'speed': cfg.reverse_speed})
    if rung == 0:
        return [back]
    if rung == 1:
        return [('turn', {'angle': cfg.rotate_deg}), back]
    if rung == 2:
        return [('turn', {'angle': -cfg.rotate_deg}), back]
    return None
