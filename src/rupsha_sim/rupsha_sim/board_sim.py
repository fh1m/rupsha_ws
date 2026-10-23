"""The ESP32, stood in for by a node, until the board exists.

Speaks the firmware contract and nothing else the planner needs. Speed and
steer become rear-axle rate and Ackermann hinge angles. Ticks are the
average of the two driven wheels at 1873 per metre. A heartbeat that stops
changing for half a second is a dead link: speed zero, steer centred.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, NavSatFix
from std_msgs.msg import Float64MultiArray

from rupsha_interfaces.msg import BoardCommand, BoardState

from . import chassis, contract

SENSOR_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)


class BoardSim(Node):

    def __init__(self):
        super().__init__('board_sim')
        self.speed_mms = 0.0
        self.steer_rad = 0.0
        self.heartbeat = None
        self.heartbeat_at = None
        self.joints: dict[str, float] = {}
        self.orientation = Quaternion(w=1.0)
        self.latitude = math.nan
        self.longitude = math.nan
        self.altitude = math.nan
        self.gps_fix = 0
        self.create_subscription(BoardCommand, chassis.BOARD_COMMAND, self._command, 10)
        self.create_subscription(JointState, '/joint_states', self._joints, 10)
        self.create_subscription(Imu, contract.SIM_IMU, self._imu, SENSOR_QOS)
        self.create_subscription(NavSatFix, contract.SIM_NAVSAT, self._navsat, SENSOR_QOS)
        self.drive = self.create_publisher(Float64MultiArray, chassis.DRIVE_COMMAND, 10)
        self.steer = self.create_publisher(Float64MultiArray, chassis.STEER_COMMAND, 10)
        self.state = self.create_publisher(BoardState, chassis.BOARD_STATE, 10)
        self.create_timer(1.0 / chassis.STATE_HZ, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _command(self, msg: BoardCommand) -> None:
        now = self._now()
        if self.heartbeat != msg.heartbeat:
            self.heartbeat = msg.heartbeat
            self.heartbeat_at = now
        self.speed_mms = float(msg.speed_mms)
        self.steer_rad = float(msg.steer_rad)

    def _joints(self, msg: JointState) -> None:
        self.joints = {n: p for n, p in zip(msg.name, msg.position, strict=True)}

    def _imu(self, msg: Imu) -> None:
        self.orientation = msg.orientation

    def _navsat(self, msg: NavSatFix) -> None:
        self.latitude = float(msg.latitude)
        self.longitude = float(msg.longitude)
        self.altitude = float(msg.altitude)
        # A finite sample from the sim navsat is a 3D fix. There is no sky to wait on.
        self.gps_fix = 3 if math.isfinite(self.latitude) and math.isfinite(self.longitude) else 0

    def _fresh(self, now: float) -> bool:
        if self.heartbeat is None or self.heartbeat_at is None:
            return False
        return (now - self.heartbeat_at) < chassis.LINK_TIMEOUT_S

    def _tick(self) -> None:
        now = self._now()
        fresh = self._fresh(now)
        speed = self.speed_mms if fresh else 0.0
        steer = self.steer_rad if fresh else 0.0
        left_w, right_w = chassis.wheel_rates(speed, steer)
        left, right = chassis.ackermann_angles(steer)
        self.drive.publish(Float64MultiArray(data=[left_w, right_w]))
        self.steer.publish(Float64MultiArray(data=[left, right]))
        ticks = chassis.ticks_from_axle(
            self.joints.get('rear_left_wheel_joint', 0.0),
            self.joints.get('rear_right_wheel_joint', 0.0))
        out = BoardState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.encoder_ticks = ticks
        out.orientation = self.orientation
        out.latitude_deg = self.latitude
        out.longitude_deg = self.longitude
        out.altitude_m = self.altitude if math.isfinite(self.altitude) else 0.0
        out.gps_fix = self.gps_fix
        out.battery_v = chassis.BATTERY_V
        out.link_fresh = fresh
        self.state.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoardSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
