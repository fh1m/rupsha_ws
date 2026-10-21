"""Newest-frame mailbox: one slot, the producer overwrites, the consumer takes.

A frame stream is a mailbox, not a queue. Only the newest frame has any value
to a steering loop; a consumer that works through a backlog is acting on the
past, and the lag grows exactly when the pipeline is slowest. So there is one
slot. `put()` replaces whatever is in it and counts the replaced frame as a
DROP -- a frame nobody looked at -- and `get()` empties it, so the same frame
is never handed out twice as if it were a new observation.

The drop counter is the honest measure of "is the detector keeping up": at
30 fps in and 10 fps out it reads two drops per processed frame, and that is
correct behaviour, not a fault.

AGE. `age_s()` is measured from the instant the caller says the frame was
captured, on THIS host's monotonic clock. If the caller cannot say, arrival
time is used and the age is a lower bound -- it flatters, never the reverse.
"""
from __future__ import annotations

import threading
import time
from typing import Any


class FrameMailbox:
    """Single-slot, thread-safe, newest-wins."""

    def __init__(self) -> None:
        self._cv = threading.Condition()
        self._frame: Any = None
        self._stamp: Any = None
        self._full = False
        self._seq = 0
        self._last_mono = float('nan')
        self._closed = False
        self.puts = 0
        self.drops = 0
        self.gets = 0

    def put(self, frame: Any, stamp: Any = None,
            capture_mono: float | None = None) -> int:
        """Store `frame`, replacing any unconsumed one. Returns its seq (1-based)."""
        mono = time.monotonic() if capture_mono is None else float(capture_mono)
        with self._cv:
            if self._full:
                self.drops += 1
            self._seq += 1
            self._frame, self._stamp, self._full = frame, stamp, True
            self._last_mono = mono
            self.puts += 1
            self._cv.notify()
            return self._seq

    def get(self, timeout: float | None = None
            ) -> tuple[Any, Any, int] | None:
        """`(frame, stamp, seq)` of the newest unconsumed frame, or None.

        Blocks up to `timeout` s (None = forever) for a frame to arrive. Returns
        None on timeout or after `close()`; never returns a frame twice.
        """
        with self._cv:
            if not self._cv.wait_for(lambda: self._full or self._closed,
                                     timeout=timeout):
                return None
            if not self._full:
                return None
            item = (self._frame, self._stamp, self._seq)
            # Drop the reference: a 640x480x3 frame held after it was consumed
            # is memory the producer's next frame has to coexist with.
            self._frame, self._stamp, self._full = None, None, False
            self.gets += 1
            return item

    def age_s(self, now_mono: float | None = None) -> float:
        """Seconds since the newest frame PUT was captured. NaN if none ever was."""
        now = time.monotonic() if now_mono is None else float(now_mono)
        with self._cv:
            return now - self._last_mono

    def pending(self) -> bool:
        with self._cv:
            return self._full

    @property
    def seq(self) -> int:
        with self._cv:
            return self._seq

    def close(self) -> None:
        """Wake every waiter; subsequent `get()` returns None once the slot is empty."""
        with self._cv:
            self._closed = True
            self._cv.notify_all()
