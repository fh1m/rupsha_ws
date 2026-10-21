"""The node, live: a rendered tag in, a Detections with the image's stamp out.

Skipped unless rclpy, cv_bridge, rupsha_interfaces AND the generated manifest
all import -- i.e. on a sourced, built workspace. The cores it wires are
covered without ROS by the other files.
"""
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cv_bridge')
pytest.importorskip('rupsha_interfaces.msg')
try:
    from rupsha_control.manifest import AutonomyState, Topics
except Exception as e:          # noqa: BLE001
    pytest.skip(f'rupsha_control.manifest does not import: {e}', allow_module_level=True)

from rupsha_vision.camera_model import PinholeCamera  # noqa: E402
from rupsha_vision.synthetic import render_tag, rotation  # noqa: E402


def test_tag_frame_round_trip():
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import CameraInfo, Image

    from rupsha_interfaces.msg import AutonomyStatus, Detections
    from rupsha_vision.vision_node import _make_node_class

    rclpy.init()
    try:
        node = _make_node_class()()
        peer = rclpy.create_node('vision_test_peer')
        got = []
        peer.create_subscription(Detections, Topics.DETECTIONS, got.append, 10)
        img_pub = peer.create_publisher(Image, Topics.CAMERA_FRONT, 1)
        info_pub = peer.create_publisher(CameraInfo, Topics.CAMERA_FRONT_INFO, 1)
        st_pub = peer.create_publisher(AutonomyStatus, Topics.AUTONOMY_STATUS, 1)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(peer)

        cam = PinholeCamera.from_fov(640, 480, 70.0)
        info = CameraInfo(width=640, height=480, distortion_model='plumb_bob')
        info.k = cam.K.reshape(-1).tolist()
        info.d = [0.0] * 5
        gray, truth = render_tag(cam, 4, 0.20, rotation(15.0), [0.3, 0.0, 3.0])
        import cv2
        msg = CvBridge().cv2_to_imgmsg(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), 'bgr8')
        msg.header.frame_id = 'camera_front_optical'

        st = AutonomyStatus()
        st.state = int(AutonomyState.APPROACHING_MARKER)
        # Frames sent before camera_info lands are, correctly, published with
        # NaN range; the round trip under test is the calibrated one.
        deadline = time.monotonic() + 10.0
        while (node._camera_info is None or node._state_name is None) \
                and time.monotonic() < deadline:
            info_pub.publish(info)
            st_pub.publish(st)
            ex.spin_once(timeout_sec=0.1)
        assert node._camera_info is not None and node._camera_info.calibrated
        deadline = time.monotonic() + 10.0
        while not got and time.monotonic() < deadline:
            msg.header.stamp = node.get_clock().now().to_msg()
            img_pub.publish(msg)
            ex.spin_once(timeout_sec=0.1)
        assert got, 'no Detections within 10 s'
        out = got[-1]
        assert out.image_width == 640 and out.image_height == 480
        assert out.header.frame_id == 'camera_front_optical'
        (d,) = out.detections
        assert d.tag_id == 4 and d.source == 'aruco'
        assert abs(d.range_m - truth.range_m) < 0.03 * truth.range_m
        assert math.isfinite(out.latency_ms) and out.latency_ms >= 0.0
        node.shutdown()
    finally:
        rclpy.try_shutdown()
