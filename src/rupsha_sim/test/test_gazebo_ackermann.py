"""The Ackermann plant against the board contract, scored on Gazebo truth.

    source install/setup.bash && pytest -m gazebo src/rupsha_sim/test/test_gazebo_ackermann.py

Needs gz sim and gz_ros2_control. Does not need ArduRover.
"""

import math
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from rupsha_control.manifest import Topics
from rupsha_interfaces.msg import BoardCommand, BoardState
from rupsha_sim import chassis, contract, gazebo

HOME_LAT = -35.363262
HOME_LON = 149.165237

pytestmark = pytest.mark.gazebo


def _herr(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def _control_plugin():
    import shutil
    if shutil.which('gz') is None:
        return 'gz not on PATH (source /opt/ros/jazzy/setup.bash)'
    roots = ['/opt/ros/jazzy/lib']
    roots += os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '').split(os.pathsep)
    for root in roots:
        if not root:
            continue
        for name in ('libgz_ros2_control-system.so', 'gz_ros2_control-system'):
            if os.path.exists(os.path.join(root, name)):
                return None
    return 'gz_ros2_control-system not installed (ros-jazzy-gz-ros2-control)'


class _Probe(Node):

    def __init__(self):
        super().__init__('ackermann_probe')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.state = None
        self.image = None
        self.pub = self.create_publisher(BoardCommand, chassis.BOARD_COMMAND, 10)
        self.create_subscription(BoardState, chassis.BOARD_STATE, self._state, 10)
        self.create_subscription(Image, Topics.CAMERA_FRONT, self._image, qos)
        self.hb = 1

    def _state(self, msg):
        self.state = msg

    def _image(self, msg):
        self.image = msg

    def command(self, speed_mms, steer_rad):
        msg = BoardCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed_mms = float(speed_mms)
        msg.steer_rad = float(steer_rad)
        msg.heartbeat = self.hb
        self.hb += 1
        self.pub.publish(msg)


def _spin(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


@pytest.fixture(scope='module')
def plant(tmp_path_factory):
    reason = _control_plugin()
    if reason:
        pytest.skip(reason)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    logs = tmp_path_factory.mktemp('ackermann')
    log = open(logs / 'launch.log', 'w')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'rupsha_sim', 'sim.launch.py', 'headless:=true'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = _Probe()
    try:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.state is not None and gazebo.truth(timeout=0.2) is not None:
                break
        else:
            pytest.fail(f'plant never published state and truth (see {logs / "launch.log"})')
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()


def test_drive_steer_ticks_camera_and_gps(plant):
    node = plant
    # Let the navsat and the camera produce a sample while we are still.
    for _ in range(80):
        node.command(0.0, 0.0)
        _spin(node, 0.1)
        if node.state.gps_fix == 3 and node.image is not None:
            break
    assert node.state.gps_fix == 3
    assert node.state.latitude_deg == pytest.approx(HOME_LAT, abs=0.01)
    assert node.state.longitude_deg == pytest.approx(HOME_LON, abs=0.01)
    assert math.isfinite(node.state.orientation.w)
    assert node.image.width == contract.CAMERA_WIDTH
    assert node.image.height == contract.CAMERA_HEIGHT
    assert len(set(node.image.data[::50])) > 1

    e0, n0, h0 = gazebo.truth()
    ticks0 = node.state.encoder_ticks
    for _ in range(40):
        node.command(400.0, 0.0)
        _spin(node, 0.05)
    e1, n1, h1 = gazebo.truth()
    travelled = math.hypot(e1 - e0, n1 - n0)
    assert travelled > 0.4, f'only {travelled:.2f} m in 2 s at 0.4 m/s'
    assert e1 > e0 + 0.3, f'drove north/south instead of east: {(e0, n0)} -> {(e1, n1)}'
    dticks = node.state.encoder_ticks - ticks0
    expected = travelled * chassis.TPM
    assert dticks == pytest.approx(expected, rel=0.35), f'{dticks} ticks, truth {expected:.0f}'
    assert abs(_herr(h1, h0)) < 12.0

    for _ in range(50):
        node.command(400.0, 0.35)
        _spin(node, 0.05)
    _, _, h2 = gazebo.truth()
    assert abs(_herr(h2, h1)) > 8.0, f'heading {h1:.1f} -> {h2:.1f}'

    # Heartbeat stops. The board must drop the link.
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline and node.state.link_fresh:
        _spin(node, 0.1)
    assert node.state.link_fresh is False
