"""The companion heartbeat. The board's GCS failsafe (FS_GCS_ENABLE, scoped to
MAV_GCS_SYSID = the companion sysid) puts the rover in HOLD when this stops.

⛔ It runs on its own daemon thread and never takes a lock a motion loop holds.
mongla measured a heartbeat starved behind a busy executor; a failsafe that fires
because the manager was *busy* rather than *dead* teaches operators to disable it.
"""

import threading
import time


class Heartbeat:

    def __init__(self, send, period_s: float = 0.5, log=None):
        self._send = send
        self._period = period_s
        self._log = log
        self._stop = threading.Event()
        self._thread = None
        self.sent = 0
        self.errors = 0
        self.max_gap_s = 0.0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name='heartbeat', daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            try:
                self._send()
                self.sent += 1
            except Exception as exc:              # noqa: BLE001 -- keep beating through a hiccup
                self.errors += 1
                if self._log and self.errors in (1, 10, 100):
                    self._log.warning(f'[HB   ] send failed ({self.errors}x): {exc}')
            now = time.monotonic()
            self.max_gap_s = max(self.max_gap_s, now - last)
            last = now
            self._stop.wait(self._period)
