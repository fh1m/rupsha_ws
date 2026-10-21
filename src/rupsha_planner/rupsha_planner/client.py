"""RosPort: the port contract over ROS 2.

Board/host/meta verbs go to /rupsha/move (the manager), executive verbs to
/rupsha/autonomy (the executive node); the route is the manifest's `runs_on`, so
a verb cannot be sent to the wrong server by a typo in a mission.

The node spins on its own executor thread. `run()` blocks the caller, polls
`watch` every 0.2 s, and cancels the goal when it returns a reason. A Ctrl-C
during `run()` cancels the goal before re-raising: an interrupted mission must
not leave a verb streaming on the manager.
"""

from __future__ import annotations

import math
import threading
import time
from types import SimpleNamespace

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from rupsha_control.manifest import FIELDS, VERBS, Outcome, Topics
from rupsha_control.rupsha import VerbResult
from rupsha_interfaces.action import Move
from rupsha_interfaces.msg import Detections, RupshaState

WATCH_PERIOD_S = 0.2
STATE_STALE_S = 2.0


def _absent_state():
    return SimpleNamespace(link_alive=False, armed=None, mode='', lat=math.nan, lon=math.nan,
                           yaw_deg=math.nan, yaw_rate_dps=math.nan, groundspeed=math.nan,
                           gps_fix=-1, ekf_ok=None)


class RosPort:

    def __init__(self, node, connect_timeout: float = 15.0):
        self.node = node
        self._move = ActionClient(node, Move, Topics.MOVE_ACTION)
        self._auto = ActionClient(node, Move, Topics.AUTONOMY_ACTION)
        self._state = None
        self._state_at = -math.inf
        self._dets = []
        self._lock = threading.Lock()
        node.create_subscription(RupshaState, Topics.STATE, self._on_state, 10)
        node.create_subscription(Detections, Topics.DETECTIONS, self._on_dets, qos_profile_sensor_data)
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()
        if not self._move.wait_for_server(timeout_sec=connect_timeout):
            raise TimeoutError(f'{Topics.MOVE_ACTION} not available -- is the manager running?')

    # ── port contract ────────────────────────────────────────────────────────
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, dt: float) -> None:
        time.sleep(dt)

    def _on_state(self, msg):
        with self._lock:
            self._state, self._state_at = msg, time.monotonic()

    def _on_dets(self, msg):
        stamp = time.monotonic() - msg.latency_ms * 1e-3
        dets = [SimpleNamespace(label=d.label, tag_id=d.tag_id, bearing_deg=d.bearing_deg,
                                range_m=d.range_m, confidence=d.confidence, stamp=stamp)
                for d in msg.detections]
        with self._lock:
            self._dets = dets

    def state(self):
        with self._lock:
            m, at = self._state, self._state_at
        if m is None or time.monotonic() - at > STATE_STALE_S:
            return _absent_state()
        return SimpleNamespace(link_alive=m.link_alive, armed=m.armed, mode=m.mode, lat=m.lat,
                               lon=m.lon, yaw_deg=m.yaw_deg, yaw_rate_dps=m.yaw_rate_dps,
                               groundspeed=m.groundspeed, gps_fix=m.gps_fix, ekf_ok=m.ekf_ok)

    def detections(self):
        with self._lock:
            return list(self._dets)

    def run(self, verb: str, watch=None, **kw) -> VerbResult:
        if verb not in VERBS:
            return VerbResult(Outcome.DENIED, f'unknown verb {verb}', elapsed_s=0.0)
        client = self._auto if VERBS[verb]['runs_on'] == 'executive' else self._move
        goal = Move.Goal()
        goal.verb = verb
        for name, value in kw.items():
            if name not in FIELDS:
                return VerbResult(Outcome.DENIED, f'{verb}: unknown field {name}', elapsed_s=0.0)
            setattr(goal, name, type(getattr(goal, name))(value))
        t0 = time.monotonic()
        if not client.wait_for_server(timeout_sec=5.0):
            return VerbResult(Outcome.NOT_READY, f'{verb}: server not available', elapsed_s=0.0)
        fut = client.send_goal_async(goal)
        if not self._wait(fut, 10.0):
            return VerbResult(Outcome.NOT_READY, f'{verb}: no goal response', elapsed_s=0.0)
        handle = fut.result()
        if not handle.accepted:
            return VerbResult(Outcome.DENIED, f'{verb}: rejected (busy, or wrong server)',
                              elapsed_s=time.monotonic() - t0)
        res_fut = handle.get_result_async()
        try:
            last = time.monotonic()
            while not res_fut.done():
                time.sleep(0.02)
                if watch is not None and time.monotonic() - last >= WATCH_PERIOD_S:
                    last = time.monotonic()
                    if watch() is not None:
                        handle.cancel_goal_async()
                        self._wait(res_fut, 10.0)
                        break
        except KeyboardInterrupt:
            handle.cancel_goal_async()
            self._wait(res_fut, 5.0)
            raise
        if not res_fut.done():
            return VerbResult(Outcome.LOST, f'{verb}: no result after cancel',
                              elapsed_s=time.monotonic() - t0)
        r = res_fut.result().result
        return VerbResult(Outcome(r.outcome), r.message, r.final_value, r.error_value, r.elapsed_s)

    @staticmethod
    def _wait(fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.01)
        return fut.done()

    def close(self):
        # Exiting with the spin thread still inside rclpy's C++ wait aborts the
        # process ("terminate called without an active exception") and loses the
        # exit code, so stop it, join it, then tear the node and context down.
        self._executor.shutdown()
        self._spin.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown()


def make_port(name='rupsha_client'):
    if not rclpy.ok():
        rclpy.init()
    return RosPort(rclpy.create_node(name))
