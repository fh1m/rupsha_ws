"""How the executive and the mission DSL reach the vehicle.

A port runs ONE verb to completion and returns its result. `watch`, when given,
is polled while the verb runs; a non-None return is a reason to cut the verb
short (stuck, operator stop), and the port aborts it cooperatively.

Two ports, one contract:
  * FacadePort  -- in-process: the rupsha_control facade on a FakeRover (tests,
                   dry runs) or on a live ArduRover (bench scripts without ROS).
  * RosPort     -- in client.py: /rupsha/move over ROS 2, used on the vehicle.
The executive cannot tell them apart, which is what lets its tests mean something.
"""

from __future__ import annotations

WATCH_PERIOD_S = 0.2


class FacadePort:

    def __init__(self, rupsha, clock=None, sleep=None):
        self.rupsha = rupsha
        self._clock = clock or rupsha.clock
        self._sleep = sleep or rupsha.sleep
        self.log = []                         # (verb, kwargs, outcome name)

    def now(self) -> float:
        return self._clock()

    def sleep(self, dt: float) -> None:
        self._sleep(dt)

    def state(self):
        return self.rupsha.fc.telemetry()

    def detections(self):
        return self.rupsha.vision()

    def run(self, verb: str, watch=None, **kw):
        if watch is None:
            res = self.rupsha.run(verb, **kw)
        else:
            last = [self._clock()]
            real_sleep = self.rupsha.sleep

            def watched_sleep(dt):
                real_sleep(dt)
                if self._clock() - last[0] >= WATCH_PERIOD_S:
                    last[0] = self._clock()
                    if watch() is not None:
                        self.rupsha.request_abort()
            self.rupsha.sleep = watched_sleep
            try:
                res = self.rupsha.run(verb, **kw)
            finally:
                self.rupsha.sleep = real_sleep
        self.log.append((verb, kw, res.outcome.name))
        return res
