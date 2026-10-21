"""Per-frame latency, measured by the vision node itself.

Two numbers per frame, because they answer different questions:

``compute_ms``  detector time on this host (perf_counter around the work).
                "Is the Pi fast enough for this phase's detectors?"
``e2e_ms``      capture stamp -> publish, on the node's clock. What a consumer
                of /rupsha/detections actually lives with; it includes the
                transport and the mailbox wait. NaN when the stamp is not in
                the node's clock domain (zero, in the future, absurdly old) --
                a negative or huge age is a clock problem, not a latency.
"""
from __future__ import annotations

import math
from collections import deque

# Outside these bounds the capture stamp is not trusted as a time on our clock.
SKEW_TOL_S = 0.05
MAX_AGE_S = 5.0


def e2e_ms(stamp_s: float, now_s: float, *, skew_tol_s: float = SKEW_TOL_S,
           max_age_s: float = MAX_AGE_S) -> float:
    if not (math.isfinite(stamp_s) and stamp_s > 0.0):
        return float('nan')
    age = now_s - stamp_s
    if not (-skew_tol_s <= age <= max_age_s):
        return float('nan')
    return max(age, 0.0) * 1e3


class LatencyWindow:
    """Rolling window of the last `n` samples; NaN samples are counted, not averaged."""

    def __init__(self, n: int = 100) -> None:
        self._v: deque[float] = deque(maxlen=int(n))
        self.nan_count = 0

    def add(self, ms: float) -> None:
        if math.isfinite(ms):
            self._v.append(float(ms))
        else:
            self.nan_count += 1

    def summary(self) -> dict[str, float]:
        if not self._v:
            return {'n': 0, 'mean': float('nan'), 'p95': float('nan'),
                    'max': float('nan')}
        s = sorted(self._v)
        p95 = s[min(len(s) - 1, math.ceil(0.95 * len(s)) - 1)]
        return {'n': len(s), 'mean': sum(s) / len(s), 'p95': p95, 'max': s[-1]}
