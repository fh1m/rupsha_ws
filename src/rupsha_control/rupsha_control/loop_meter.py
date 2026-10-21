"""Per-loop rate telemetry: exact, average and 1%-low Hz.

The 1%-low is MissouriMRDT Autonomy_Software's habit and the right one: an average
of 10 Hz hides the 300 ms stall that let the rover coast past a post. Every
periodic loop in the stack ticks one of these and the manager publishes them on
/diagnostics.
"""

import math
import time
from collections import deque


class LoopMeter:

    def __init__(self, name: str, window: int = 500, clock=time.monotonic):
        self.name = name
        self._clock = clock
        self._last = None
        self._dts = deque(maxlen=window)

    def tick(self) -> None:
        now = self._clock()
        if self._last is not None:
            self._dts.append(now - self._last)
        self._last = now

    def stats(self) -> dict:
        """{'exact', 'avg', 'low1'} in Hz; NaN until two ticks have happened."""
        if not self._dts:
            return {'exact': math.nan, 'avg': math.nan, 'low1': math.nan}
        dts = sorted(self._dts)
        k = max(1, math.ceil(len(dts) * 0.01))
        worst = sum(dts[-k:]) / k
        mean = sum(dts) / len(dts)
        last = self._dts[-1]
        return {'exact': 1.0 / last if last > 0 else math.inf,
                'avg': 1.0 / mean if mean > 0 else math.inf,
                'low1': 1.0 / worst if worst > 0 else math.inf}
