"""Goal acceptance for /rupsha/move, and goal -> verb kwargs.

The policy is mongla's: one verb at a time; a verb marked `bypass_busy` in the
manifest (stop, abort, disarm, disable_autonomy) is accepted while busy and
signals a cooperative abort so the running loop lets go.

The gate is an object, not a bool, for mongla's measured reason: deciding in
goal_callback and claiming in execute_callback is two steps on a
MultiThreadedExecutor, and two goals got through 40 of 40 times. Decide and
claim under one lock; never hold it across I/O.
"""

from __future__ import annotations

import math
import threading
import time

from .manifest import FIELDS, VERBS

BYPASS_VERBS = frozenset(v for v, s in VERBS.items() if s['bypass_busy'])
COMMAND_GATE_STALE_S = 5.0


def goal_acceptance(verb: str, busy: bool) -> tuple:
    """(accept, signal_abort)."""
    if verb not in VERBS:
        return False, False
    if not busy:
        return True, False
    if verb in BYPASS_VERBS:
        return True, True
    return False, False


class CommandGate:
    """IDLE -> CLAIMED -> (DRAINING) -> IDLE, transitions only under the lock."""

    def __init__(self, stale_s: float = COMMAND_GATE_STALE_S):
        self._lock = threading.Lock()
        self._verb = None
        self._claimed_at = 0.0
        self._confirmed = False
        self._draining = False
        self._stale_s = stale_s

    @property
    def active(self) -> bool:
        with self._lock:
            return self._verb is not None or self._draining

    @property
    def holder(self):
        with self._lock:
            return self._verb

    def claim(self, verb: str, policy=goal_acceptance) -> tuple:
        with self._lock:
            busy = self._verb is not None or self._draining
            accept, signal_abort = policy(verb, busy)
            if accept:
                self._verb = verb
                self._claimed_at = time.monotonic()
                self._confirmed = False
                self._draining = False
            return accept, signal_abort

    def confirm(self, verb: str) -> bool:
        with self._lock:
            if self._verb != verb:
                return False
            self._confirmed = True
            return True

    def begin_drain(self) -> None:
        with self._lock:
            self._verb = None
            self._confirmed = False
            self._draining = True

    def release(self, verb: str) -> None:
        """Release only a claim that is still `verb`'s: a pre-empting stop owns it now."""
        with self._lock:
            if self._verb not in (None, verb):
                return
            self._verb = None
            self._confirmed = False
            self._draining = False

    def reap_stale(self):
        with self._lock:
            if self._verb is None or self._confirmed:
                return None
            if time.monotonic() - self._claimed_at < self._stale_s:
                return None
            leaked, self._verb = self._verb, None
            return leaked


def _unset(value, ros_type: str) -> bool:
    if ros_type == 'string':
        return value == ''
    if ros_type.startswith('float'):
        return value == 0.0 or math.isnan(value)
    return False


def fields_for(verb: str, goal) -> dict:
    """The kwargs `verb` reads from a Move.Goal (or any object with the attributes).

    Zero on a float field means "unset" and takes the manifest default, because
    rosidl cannot tell an omitted float from 0.0. Integers carry an explicit wire
    default (tag_id = -1) instead. A field with no default left unset is passed as
    is, and the verb decides whether zero is legal for it.
    """
    spec = VERBS[verb]
    out = {}
    for name in spec['fields']:
        ros_type = FIELDS[name][0]
        value = getattr(goal, name)
        if name in spec['defaults'] and _unset(value, ros_type):
            value = spec['defaults'][name]
        out[name] = value
    return out
