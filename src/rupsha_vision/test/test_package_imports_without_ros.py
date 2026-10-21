"""Every core imports with no ROS on the path, and does not pull ROS in.

Run in a subprocess: this suite's own process may already have rclpy loaded by
another package's tests, which would make an in-process check vacuous.
"""
import pathlib
import subprocess
import sys

PKG = pathlib.Path(__file__).resolve().parents[1]
CORES = ['mailbox', 'camera_model', 'aruco', 'detector', 'scheduler', 'latency',
         'synthetic', 'vision_node']


def test_cores_import_without_ros():
    code = (
        'import sys\n'
        f'sys.path.insert(0, {str(PKG)!r})\n'
        + ''.join(f'import rupsha_vision.{m}\n' for m in CORES)
        + 'bad = [m for m in sys.modules if m.split(".")[0] in '
          '("rclpy", "rupsha_interfaces", "cv_bridge", "rupsha_control", "onnxruntime")]\n'
          'assert not bad, bad\n')
    env = {'PATH': '/usr/bin:/bin', 'PYTHONNOUSERSITE': '1'}
    r = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                       env=env, timeout=60, check=False)
    assert r.returncode == 0, r.stderr
