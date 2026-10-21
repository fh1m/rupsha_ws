#!/usr/bin/env python3
"""rupsha executive: /rupsha/autonomy for the leg-queue verbs.

    ros2 run rupsha_planner executive

`start_autonomy` returns at once and the run continues on a worker thread;
progress is /rupsha/autonomy_status. `disable_autonomy` is always accepted, stops
the run and sends `stop` to the manager. The executive reaches the vehicle only
through /rupsha/move, so the manager's gate still decides what runs.
"""

from __future__ import annotations

import math
import threading

import rclpy
from rclpy.action import ActionServer, GoalResponse

from rupsha_control.dispatch import fields_for
from rupsha_control.manifest import VERBS, Outcome, Topics
from rupsha_interfaces.action import Move
from rupsha_interfaces.msg import AutonomyStatus

from .client import RosPort
from .executive import Executive

EXEC_VERBS = {v for v, s in VERBS.items() if s['runs_on'] == 'executive'}


class ExecutiveNode:

    def __init__(self):
        self.node = rclpy.create_node('rupsha_executive')
        self.pub = self.node.create_publisher(AutonomyStatus, Topics.AUTONOMY_STATUS, 10)
        self.server = ActionServer(self.node, Move, Topics.AUTONOMY_ACTION,
                                   execute_callback=self.execute, goal_callback=self.accept)
        self.node.create_timer(0.5, self.publish)
        self.port = RosPort(self.node)
        self.exe = Executive(self.port, on_status=lambda _: self.publish())
        self.worker = None
        self.last = None
        self.node.get_logger().info(f'[READY] serving {Topics.AUTONOMY_ACTION}')

    def accept(self, goal):
        return GoalResponse.ACCEPT if goal.verb in EXEC_VERBS else GoalResponse.REJECT

    def publish(self):
        e = self.exe
        m = AutonomyStatus()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.state, m.display = int(e.state), int(e.display)
        m.leg_index, m.legs_total = e.leg_index, len(e.legs)
        m.distance_to_goal = float(e.distance_to_goal)
        m.detail = e.detail
        self.pub.publish(m)

    def _run(self):
        self.last = self.exe.run()
        self.node.get_logger().info(f'[AUTO ] {self.last.message}')

    def execute(self, goal_handle):
        verb = goal_handle.request.verb
        kw = fields_for(verb, goal_handle.request)
        out = Move.Result()
        ok, msg = True, ''
        if verb == 'start_autonomy':
            if self.worker and self.worker.is_alive():
                ok, msg = False, 'already running'
            elif not self.exe.legs:
                ok, msg = False, 'no legs queued'
            else:
                self.worker = threading.Thread(target=self._run, name='executive', daemon=True)
                self.worker.start()
                msg = f'started {len(self.exe.legs)} legs'
        elif verb == 'disable_autonomy':
            self.exe.request_stop()
            if self.worker:
                self.worker.join(timeout=15.0)
            self.port.run('stop')
            msg = 'autonomy disabled'
        else:
            ok, msg = getattr(self.exe, verb)(**kw)
        out.outcome = int(Outcome.SUCCEEDED if ok else Outcome.DENIED)
        out.success, out.message = ok, msg
        out.final_value = out.error_value = math.nan
        out.elapsed_s = 0.0
        goal_handle.succeed() if ok else goal_handle.abort()
        self.publish()
        return out


def main(args=None):
    rclpy.init(args=args)
    ex = ExecutiveNode()
    try:
        ex.port._spin.join()
    except KeyboardInterrupt:
        ex.exe.request_stop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
