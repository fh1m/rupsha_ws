#!/usr/bin/env python3
"""rupsha manager: the one node that owns the Pixhawk.

    ros2 run rupsha_control manager                                  # real board, USB
    ros2 run rupsha_control manager --ros-args -p url:=tcp:127.0.0.1:5760   # SITL

Serves /rupsha/move (board, host and meta verbs), publishes /rupsha/state,
/rupsha/imu, /rupsha/navsat and loop rates on /diagnostics, and -- only when
`extnav_uplink` is true -- forwards /rupsha/extnav/odometry to the board as
MAVLink ODOMETRY for EKF3 source set 2.

Startup is neutral: if the board comes up armed in GUIDED (a manager restarted
under a moving rover), the first thing sent is HOLD.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from rupsha_interfaces.action import Move
from rupsha_interfaces.msg import Detections, RupshaState

from .dispatch import BYPASS_VERBS, CommandGate, fields_for
from .fc.ardurover import ArduRover
from .heartbeat import Heartbeat
from .loop_meter import LoopMeter
from .manifest import VERBS, Companion, Outcome, Topics
from .param_audit import resolve_url
from .rupsha import Rupsha, VerbResult

DEFAULT_URL = '/dev/serial/by-id/usb-ArduPilot_Pixhawk1*'


class _Det:
    __slots__ = ('label', 'tag_id', 'bearing_deg', 'range_m', 'confidence', 'stamp')

    def __init__(self, d, stamp):
        self.label, self.tag_id = d.label, d.tag_id
        self.bearing_deg, self.range_m = d.bearing_deg, d.range_m
        self.confidence, self.stamp = d.confidence, stamp


class RoverManager(Node):

    def __init__(self):
        super().__init__('rupsha_manager')
        p = self.declare_parameter
        self.url = p('url', DEFAULT_URL).value
        self.baud = p('baud', 115200).value
        self.state_hz = p('state_hz', 10.0).value
        self.extnav_uplink = p('extnav_uplink', False).value
        log = self.get_logger()

        url = resolve_url(self.url)
        log.info(f'[LINK ] connecting {url} as sysid {Companion.SYSID}/{Companion.COMPID}')
        self.fc = ArduRover(url, baud=self.baud, log=log)
        self.fc.wait_board(timeout=30.0)
        self.fc.start()

        self._dets: list = []
        self._dets_lock = threading.Lock()
        self.rupsha = Rupsha(self.fc, vision=self._detections, log=log)

        self.heartbeat = Heartbeat(self.fc.send_heartbeat, 1.0 / Companion.HEARTBEAT_HZ, log)
        self.heartbeat.start()
        self._neutral_on_startup()
        self._read_board_limits()

        self._gate = CommandGate()
        self._cancelled = set()
        self.meters = {n: LoopMeter(n) for n in ('state', 'imu', 'extnav')}

        verbs_cb = ReentrantCallbackGroup()
        self.server = ActionServer(
            self, Move, Topics.MOVE_ACTION, execute_callback=self.execute_callback,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback,
            callback_group=verbs_cb)

        pubs_cb = MutuallyExclusiveCallbackGroup()
        self.pub_state = self.create_publisher(RupshaState, Topics.STATE, 10)
        self.pub_imu = self.create_publisher(Imu, Topics.BOARD_IMU, qos_profile_sensor_data)
        self.pub_fix = self.create_publisher(NavSatFix, Topics.BOARD_NAVSAT, 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, Topics.LOOP_RATES, 10)
        self.create_timer(1.0 / self.state_hz, self._state_tick, callback_group=pubs_cb)
        self.create_timer(1.0 / 20.0, self._imu_tick, callback_group=pubs_cb)
        self.create_timer(1.0, self._diag_tick, callback_group=pubs_cb)
        self.create_timer(1.0, self._reap_tick, callback_group=pubs_cb)
        self.create_subscription(Detections, Topics.DETECTIONS, self._on_detections,
                                 qos_profile_sensor_data)
        if self.extnav_uplink:
            self.create_subscription(Odometry, Topics.EXTNAV_ODOMETRY, self._on_extnav, 10)
            log.warning('[EXTN ] ExtNav uplink ON: /rupsha/extnav/odometry -> board ODOMETRY')
        log.info(f'[READY] firmware {self.fc.firmware or "unread"}; serving {Topics.MOVE_ACTION}')

    # ── startup ──────────────────────────────────────────────────────────────
    def _neutral_on_startup(self) -> None:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not self.fc.telemetry().mode:
            time.sleep(0.05)
        tel = self.fc.telemetry()
        if tel.armed and tel.mode == 'GUIDED':
            self.get_logger().warning('[START] board armed in GUIDED at startup -- HOLD')
            self.rupsha.stop()

    def _read_board_limits(self) -> None:
        try:
            self.rupsha.board_wp_radius = self.fc.get_param('WP_RADIUS')
            gcs = int(self.fc.get_param('MAV_GCS_SYSID'))
            fs = int(self.fc.get_param('FS_GCS_ENABLE'))
        except Exception as exc:                       # noqa: BLE001 -- report, keep serving
            self.get_logger().error(f'[PARAM] board limits unread: {exc}')
            return
        if gcs != Companion.SYSID or not fs:
            self.get_logger().error(
                f'[SAFE ] MAV_GCS_SYSID={gcs} FS_GCS_ENABLE={fs}: the board will NOT hold if '
                f'this manager dies. Load rupsha_baseline.parm (param_audit --apply).')

    # ── detections ───────────────────────────────────────────────────────────
    def _on_detections(self, msg: Detections) -> None:
        # Stamped on arrival in the facade's clock (monotonic); capture latency is in
        # the message and the vision node already measures it.
        now = time.monotonic() - msg.latency_ms * 1e-3
        dets = [_Det(d, now) for d in msg.detections]
        with self._dets_lock:
            self._dets = dets

    def _detections(self):
        with self._dets_lock:
            return list(self._dets)

    # ── action server ────────────────────────────────────────────────────────
    def goal_callback(self, goal_request):
        verb = goal_request.verb
        if verb in VERBS and VERBS[verb]['runs_on'] == 'executive':
            self.get_logger().warning(f'[ACT  ] {verb} belongs on {Topics.AUTONOMY_ACTION}')
            return GoalResponse.REJECT
        accept, signal_abort = self._gate.claim(verb)
        if not accept:
            self.get_logger().warning(f'[ACT  ] rejected {verb} (busy with {self._gate.holder} '
                                      f'or unknown verb)')
            return GoalResponse.REJECT
        if signal_abort:
            self.get_logger().info(f'[ACT  ] {verb} pre-empts -- signalling abort')
            self.rupsha.request_abort()
        elif verb not in BYPASS_VERBS:
            self.rupsha.clear_abort()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self._cancelled.add(bytes(goal_handle.goal_id.uuid))
        self.rupsha.request_abort()
        self._gate.begin_drain()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        verb = req.verb
        self._gate.confirm(verb)
        self.get_logger().info(f'[ACT  ] {verb} -> EXECUTING')

        def progress(phase, fraction, line):
            fb = Move.Feedback()
            fb.phase, fb.progress, fb.status_line = phase, float(fraction), line
            goal_handle.publish_feedback(fb)

        self.rupsha.progress = progress
        try:
            result = getattr(self.rupsha, verb)(**fields_for(verb, req))
        except Exception as exc:                       # noqa: BLE001 -- a raise is a bug; still stop
            self.get_logger().error(f'[ACT  ] {verb} raised: {exc!r}')
            self.rupsha._zero()
            result = VerbResult(Outcome.FAILED, f'{verb}: exception {exc!r}')
        finally:
            self.rupsha.progress = None
            self._gate.release(verb)

        out = Move.Result()
        out.outcome = int(result.outcome)
        out.success = result.success
        out.message = result.message
        out.final_value = float(result.final_value)
        out.error_value = float(result.error_value)
        out.elapsed_s = float(result.elapsed_s)
        uid = bytes(goal_handle.goal_id.uuid)
        if result.success:
            goal_handle.succeed()
        elif uid in self._cancelled and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self._cancelled.discard(uid)
        self.get_logger().info(f'[ACT  ] {verb} -> {result.outcome.name}: {result.message}')
        return out

    def _reap_tick(self) -> None:
        leaked = self._gate.reap_stale()
        if leaked:
            self.get_logger().error(f'[GATE ] released a claim that never executed: {leaked}')

    # ── publishers ───────────────────────────────────────────────────────────
    def _state_tick(self) -> None:
        self.meters['state'].tick()
        t = self.fc.telemetry()
        m = RupshaState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.armed = bool(t.armed)
        m.mode = t.mode
        m.link_alive = t.link_alive
        m.lat, m.lon = t.lat, t.lon
        m.yaw_deg, m.yaw_rate_dps = t.yaw_deg, t.yaw_rate_dps
        m.groundspeed, m.battery_v = t.groundspeed, t.battery_v
        m.gps_fix, m.gps_sats = t.gps_fix, t.gps_sats
        m.ekf_ok, m.prearm_ok = bool(t.ekf_ok), bool(t.prearm_ok)
        m.last_text = t.last_text
        self.pub_state.publish(m)

        fix = NavSatFix()
        fix.header.stamp = m.header.stamp
        fix.header.frame_id = 'gps'
        fix.latitude, fix.longitude, fix.altitude = t.lat, t.lon, t.alt_m
        fix.status.status = (NavSatStatus.STATUS_FIX if t.gps_fix >= 3 else NavSatStatus.STATUS_NO_FIX)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_fix.publish(fix)

    def _imu_tick(self) -> None:
        att = self.fc.raw('ATTITUDE', 0.5)
        raw = self.fc.raw('RAW_IMU', 0.5)
        if att is None:
            return
        self.meters['imu'].tick()
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # ATTITUDE is NED/FRD; publish ENU/FLU (REP-103).
        roll, pitch, yaw = att.roll, -att.pitch, math.pi / 2 - att.yaw
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.angular_velocity.x = att.rollspeed
        msg.angular_velocity.y = -att.pitchspeed
        msg.angular_velocity.z = -att.yawspeed
        if raw is not None:
            g = 9.80665e-3          # RAW_IMU accel is mG
            msg.linear_acceleration.x = raw.xacc * g
            msg.linear_acceleration.y = -raw.yacc * g
            msg.linear_acceleration.z = -raw.zacc * g
        else:
            msg.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(msg)

    def _diag_tick(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for name, meter in self.meters.items():
            s = meter.stats()
            st = DiagnosticStatus(name=f'rupsha_manager/{name}', hardware_id='rupsha')
            st.values = [KeyValue(key=k, value=f'{v:.2f}') for k, v in s.items()]
            arr.status.append(st)
        hb = DiagnosticStatus(name='rupsha_manager/heartbeat', hardware_id='rupsha')
        hb.level = DiagnosticStatus.OK if self.heartbeat.alive else DiagnosticStatus.ERROR
        hb.values = [KeyValue(key='sent', value=str(self.heartbeat.sent)),
                     KeyValue(key='max_gap_s', value=f'{self.heartbeat.max_gap_s:.3f}'),
                     KeyValue(key='reader_errors', value=str(self.fc.reader_errors))]
        arr.status.append(hb)
        self.pub_diag.publish(arr)

    # ── ExtNav uplink ────────────────────────────────────────────────────────
    def _on_extnav(self, msg: Odometry) -> None:
        """ENU/FLU odometry -> MAVLink ODOMETRY in LOCAL_FRD (NED-aligned) / BODY_FRD."""
        self.meters['extnav'].tick()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw_enu = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        yaw_ned = math.pi / 2 - yaw_enu
        qn = [math.cos(yaw_ned / 2), 0.0, 0.0, math.sin(yaw_ned / 2)]
        v = msg.twist.twist.linear
        cov = msg.pose.covariance
        self.fc.send_odometry(
            time_usec=int(time.time() * 1e6), x=p.y, y=p.x, z=-p.z, q=qn,
            vx=v.x, vy=-v.y, vz=-v.z, pos_var=max(cov[0], cov[7]), yaw_var=cov[35])

    def destroy_node(self):
        self.heartbeat.stop()
        self.fc.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
