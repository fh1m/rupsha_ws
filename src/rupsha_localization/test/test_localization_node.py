"""The node's pure helpers run everywhere; the node itself only where ROS is built."""
import math
import pathlib

import pytest
import yaml

from rupsha_localization import health
from rupsha_localization import localization_node as ln

PKG = pathlib.Path(__file__).resolve().parents[1]


def test_module_imports_without_ros():
    """Checked in a fresh interpreter: this one may have rclpy loaded by another test."""
    import subprocess
    import sys
    code = ('import sys; import rupsha_localization.localization_node, '
            'rupsha_localization.shadow; '
            "bad = [m for m in sys.modules if m.split('.')[0] in "
            "('rclpy', 'rupsha_interfaces', 'rupsha_control')]; "
            'assert not bad, bad')
    r = subprocess.run([sys.executable, '-c', code], cwd=str(PKG),
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_pose_covariance_layout():
    cov3 = [[1.0, 0.1, 0.2], [0.1, 2.0, 0.3], [0.2, 0.3, 3.0]]
    c = ln.pose_covariance_6x6(cov3)
    assert c[0] == 1.0 and c[7] == 2.0 and c[35] == 3.0
    assert c[1] == c[6] == 0.1
    assert c[5] == c[30] == 0.2
    assert c[11] == c[31] == 0.3
    for i in (2, 3, 4):
        assert c[i * 7] == ln.UNESTIMATED_VAR     # unestimated is large, never 0


def test_twist_covariance_layout():
    c = ln.twist_covariance_6x6(0.1, 0.2, 0.3)
    assert (c[0], c[7], c[35]) == (0.1, 0.2, 0.3)
    assert c[14] == ln.UNESTIMATED_VAR


def test_verdict_to_diagnostic_level():
    assert ln.diag_level(health.OK) == ln.DIAG_OK
    assert ln.diag_level(health.ABSENT) == ln.DIAG_WARN
    assert ln.diag_level(health.DIVERGED) == ln.DIAG_ERROR


def test_quaternion_is_yaw_only():
    x, y, z, w = ln.yaw_to_quaternion(math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sqrt(0.5)) and w == pytest.approx(math.sqrt(0.5))


def test_yaml_and_node_declare_the_same_parameters():
    cfg = yaml.safe_load((PKG / 'config' / 'localization.yaml').read_text())
    params = cfg['localization']['ros__parameters']
    assert set(params) == set(ln.PARAMETERS)
    for k, v in params.items():
        assert type(v) is type(ln.PARAMETERS[k]), k
    assert params['extnav_enabled'] is False


def test_config_from_params_round_trips_defaults():
    cfg = ln.config_from_params(ln.PARAMETERS.__getitem__)
    assert cfg.extnav_enabled is False
    assert cfg.rate_hz == ln.PARAMETERS['rate_hz']
    assert cfg.noise.gyro == ln.PARAMETERS['noise_gyro']


def test_manifest_topics_resolve():
    """The node takes its topic names from rupsha_control.manifest and nowhere else."""
    T = ln.topics()
    assert T.SHADOW_ODOMETRY == '/rupsha/localization/odometry'
    assert T.EXTNAV_ODOMETRY == '/rupsha/extnav/odometry'
    assert T.BOARD_IMU == '/rupsha/imu'
    assert T.BOARD_NAVSAT == '/rupsha/navsat'
    assert T.STATE == '/rupsha/state'


def test_node_constructs_and_publishes_nothing_before_origin():
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('rupsha_interfaces.msg')
    rclpy.init()
    try:
        node = ln._node_class()()
        node._tick()
        assert node._last_out is None
        assert node.get_parameter('extnav_enabled').value is False
        node.destroy_node()
    finally:
        rclpy.shutdown()
