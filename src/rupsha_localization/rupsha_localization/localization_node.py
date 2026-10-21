"""`localization`: the shadow estimator's ROS wiring. Deliberately thin.

Every decision worth arguing about -- origin, velocity source, gaps, ZUPT,
gating, the ExtNav gate -- lives in `shadow.py` and is tested without ROS.
This file converts messages and calls it.

AUTHORITY. The Pixhawk's EKF3 navigates. This node publishes a second opinion
on SHADOW_ODOMETRY, a health verdict on /diagnostics, and -- only when
`extnav_enabled` is true and the position std is under `extnav_max_std_m` --
the same estimate on EXTNAV_ODOMETRY. It never talks to the board; the uplink
to MAVLink ODOMETRY is rupsha_control's, the one node that owns the board.

NOTHING BEFORE THE ORIGIN. Until the first GNSS fix with board fix_type >= 3
arrives there is no frame to publish in, and this node publishes no odometry
at all rather than a pose of zeros that looks like one.

ROS is imported inside `_node_class()` and `main()`, never at module scope,
so the module (and its pure helpers) import on a box with no ROS.
"""
from __future__ import annotations

import math
from typing import List, Optional

from rupsha_localization import health
from rupsha_localization.riekf import ProcessNoise
from rupsha_localization.shadow import Shadow, ShadowConfig, ShadowOutput

DIAGNOSTICS_TOPIC = '/diagnostics'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
DIAG_NAME = 'rupsha_localization: shadow'

# z, roll and pitch are not estimated by a planar filter. Their variance is
# published as this, never 0: zero would claim they are known exactly.
UNESTIMATED_VAR = 1e6

# DiagnosticStatus levels, mirrored so the mapping is testable without ROS.
DIAG_OK, DIAG_WARN, DIAG_ERROR = 0, 1, 2


def topics():
    """The manifest's topic names. Imported late; a broken manifest must fail loudly here."""
    from rupsha_control.manifest import Topics
    return Topics


def pose_covariance_6x6(cov3) -> List[float]:
    """3x3 (x, y, yaw) -> row-major 6x6 (x, y, z, roll, pitch, yaw)."""
    idx = (0, 1, 5)
    out = [0.0] * 36
    for i in (2, 3, 4):
        out[i * 6 + i] = UNESTIMATED_VAR
    for a, ia in enumerate(idx):
        for b, ib in enumerate(idx):
            out[ia * 6 + ib] = float(cov3[a][b])
    return out


def twist_covariance_6x6(var_vx: float, var_vy: float, var_wz: float) -> List[float]:
    out = [0.0] * 36
    for i in (2, 3, 4):
        out[i * 6 + i] = UNESTIMATED_VAR
    out[0], out[7], out[35] = var_vx, var_vy, var_wz
    return out


def diag_level(status: str) -> int:
    return {health.OK: DIAG_OK, health.ABSENT: DIAG_WARN,
            health.DIVERGED: DIAG_ERROR}[status]


def yaw_to_quaternion(yaw: float):
    """(x, y, z, w) for a pure rotation about z."""
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def config_from_params(get) -> ShadowConfig:
    """Build the config from a `name -> value` getter (the node's parameters)."""
    noise = ProcessNoise(speed=get('noise_speed'), lateral=get('noise_lateral'),
                         gyro=get('noise_gyro'), bias_walk=get('noise_bias_walk'),
                         slip_inflation=get('slip_inflation'))
    return ShadowConfig(
        rate_hz=get('rate_hz'), track_m=get('track_m'), icr_chi=get('icr_chi'),
        slip_threshold_dps=get('slip_threshold_dps'), slip_k=get('slip_k'),
        extnav_enabled=get('extnav_enabled'), extnav_max_std_m=get('extnav_max_std_m'),
        gnss_std_m=get('gnss_std_m'), heading_from_board=get('heading_from_board'),
        heading_std_deg=get('heading_std_deg'), heading_period_s=get('heading_period_s'),
        health_max_position_m=get('health_max_position_m'),
        health_max_yaw_deg=get('health_max_yaw_deg'), noise=noise)


# Every parameter, its default, in one table: the yaml, the launch file and
# the node read the same names.
PARAMETERS = {
    'rate_hz': 20.0,
    'track_m': 0.60,
    'icr_chi': 1.0,
    'slip_threshold_dps': 15.0,
    'slip_k': 3,
    'slip_inflation': 10.0,
    'extnav_enabled': False,
    'extnav_max_std_m': 2.0,
    'gnss_std_m': 1.5,
    'heading_from_board': True,
    'heading_std_deg': 5.0,
    'heading_period_s': 1.0,
    'health_max_position_m': 5.0,
    'health_max_yaw_deg': 15.0,
    'health_period_s': 1.0,
    'noise_speed': 0.02,
    'noise_lateral': 0.02,
    'noise_gyro': 0.003,
    'noise_bias_walk': 1e-4,
}


def _node_class():
    import numpy as np
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
    from std_msgs.msg import Float32MultiArray

    from rupsha_interfaces.msg import RupshaState

    T = topics()

    class LocalizationNode(Node):
        def __init__(self):
            super().__init__('localization')
            for name, default in PARAMETERS.items():
                self.declare_parameter(name, default)
            cfg = config_from_params(lambda n: self.get_parameter(n).value)
            self.shadow = Shadow(cfg, log=self.get_logger().info)
            self.pub_odom = self.create_publisher(Odometry, T.SHADOW_ODOMETRY, 10)
            self.pub_extnav = self.create_publisher(Odometry, T.EXTNAV_ODOMETRY, 10)
            self.pub_diag = self.create_publisher(DiagnosticArray, DIAGNOSTICS_TOPIC, 10)
            self.create_subscription(Imu, T.BOARD_IMU, self._on_imu, 50)
            self.create_subscription(NavSatFix, T.BOARD_NAVSAT, self._on_navsat, 10)
            self.create_subscription(RupshaState, T.STATE, self._on_state, 10)
            self.create_subscription(Float32MultiArray, T.WHEEL_SPEEDS, self._on_wheels, 50)
            self._last_out: Optional[ShadowOutput] = None
            self.create_timer(1.0 / cfg.rate_hz, self._tick)
            self.create_timer(self.get_parameter('health_period_s').value, self._diagnose)
            self.get_logger().info(
                f'localization: shadow estimator up; board EKF3 is the authority. '
                f'extnav_enabled={cfg.extnav_enabled} (gate {cfg.extnav_max_std_m} m); '
                f'ICR chi={cfg.icr_chi} on track {cfg.track_m} m is UNMEASURED until fitted. '
                'No odometry is published before the GNSS origin.')

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def _on_imu(self, msg):
            self.shadow.on_imu(self._now(), msg.angular_velocity.z)

        def _on_wheels(self, msg):
            if len(msg.data) != 2:
                self.get_logger().warning(
                    f'{topics().WHEEL_SPEEDS}: expected [vL, vR], got {len(msg.data)} values; ignored',
                    throttle_duration_sec=5.0)
                return
            self.shadow.on_wheels(self._now(), float(msg.data[0]), float(msg.data[1]))

        def _on_state(self, msg):
            self.shadow.on_state(self._now(), msg.lat, msg.lon, msg.yaw_deg,
                                 msg.groundspeed, msg.gps_fix, msg.ekf_ok,
                                 getattr(msg, 'yaw_rate_dps', float('nan')))

        def _on_navsat(self, msg):
            cov = None
            if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
                c = msg.position_covariance
                cov = np.array([[c[0], c[1]], [c[3], c[4]]])
            self.shadow.on_navsat(self._now(), msg.latitude, msg.longitude, msg.altitude,
                                  msg.status.status >= NavSatStatus.STATUS_FIX, cov)

        def _tick(self):
            out = self.shadow.step(self._now())
            self._last_out = out
            if out is None:
                return
            odom = self._odometry(out)
            self.pub_odom.publish(odom)
            if self.shadow.extnav(out) is not None:
                self.pub_extnav.publish(odom)

        def _odometry(self, out: ShadowOutput):
            m = Odometry()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = ODOM_FRAME
            m.child_frame_id = BASE_FRAME
            m.pose.pose.position.x = out.x
            m.pose.pose.position.y = out.y
            q = yaw_to_quaternion(out.yaw)
            o = m.pose.pose.orientation
            o.x, o.y, o.z, o.w = q
            m.pose.covariance = pose_covariance_6x6(out.cov)
            m.twist.twist.linear.x = out.v_x
            m.twist.twist.linear.y = out.v_y
            m.twist.twist.angular.z = out.omega
            n = self.shadow.cfg.noise
            r = self.shadow.cfg.rate_hz
            k = n.slip_inflation if out.slipping else 1.0
            f = self.shadow.filter
            m.twist.covariance = twist_covariance_6x6(
                (k * n.speed) ** 2 * r, (k * n.lateral) ** 2 * r,
                n.gyro ** 2 * r + (f.P[3, 3] if f.estimate_bias else 0.0))
            return m

        def _diagnose(self):
            t = self._now()
            v = self.shadow.health(t, self._last_out)
            st = DiagnosticStatus()
            st.name = DIAG_NAME
            st.hardware_id = 'pi'
            st.level = bytes([diag_level(v.status)])
            st.message = f'{v.status}: {v.detail}'
            s = self.shadow
            kv = {'verdict': v.status, 'position_divergence_m': f'{v.position_m:.3f}',
                  'yaw_divergence_deg': f'{v.yaw_deg:.2f}',
                  'stale_reason': s.stale_reason or '-',
                  'origin': (f'{s.plane.lat0:.8f},{s.plane.lon0:.8f}' if s.plane else 'none'),
                  'gaps': str(s.gaps), 'position_resets': str(s.resets),
                  'zupts': str(s.zupts), 'slip_events': str(s.slip.events),
                  'slipping': str(s.slip.slipping),
                  'extnav_enabled': str(s.cfg.extnav_enabled),
                  'extnav_refusals': str(s.extnav_refusals),
                  'extnav_last_refusal': s.extnav_last_refusal or '-',
                  'imu_sign_suspect': str(s.imu_sign_suspect())}
            if s.filter is not None:
                f = s.filter
                kv['position_std_m'] = f'{f.position_std():.3f}'
                kv['yaw_std_deg'] = f'{math.degrees(f.yaw_std()):.2f}'
                kv['gyro_bias_dps'] = f'{math.degrees(f.bias):.4f}'
                for kind, c in f.report().items():
                    kv[f'gate_{kind}'] = (f"accepted={c['accepted']} rejected={c['rejected']} "
                                          f"streak={c['streak']}")
            if s.imu_sign_suspect():
                st.level = bytes([DIAG_ERROR])
                st.message += ' | IMU yaw-rate sign disagrees with the board (FRD published as FLU?)'
            st.values = [KeyValue(key=k, value=val) for k, val in kv.items()]
            arr = DiagnosticArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.status = [st]
            self.pub_diag.publish(arr)

    return LocalizationNode


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = _node_class()()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
