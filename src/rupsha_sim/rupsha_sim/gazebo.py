"""A headless Gazebo server for tests and probes, and the sim's ground truth.

`GzServer` starts `gz sim -s -r` on one of our generated worlds with the model,
world and ArduPilotPlugin paths in its environment; `truth()` reads the model's
pose straight from Gazebo, so a scenario can check where the rover *is*, not
where its EKF believes it is.
"""

from __future__ import annotations

import math
import os
import pathlib
import shutil
import signal
import subprocess
import time

from . import contract

PKG = pathlib.Path(__file__).resolve().parent.parent
PLUGIN = 'libArduPilotPlugin.so'


def _paths(var: str) -> list[pathlib.Path]:
    return [pathlib.Path(p) for p in os.environ.get(var, '').split(os.pathsep) if p]


def find_plugin_dir() -> pathlib.Path | None:
    ws_install = PKG.parent.parent / 'install' / 'ardupilot_gazebo' / 'lib' / 'ardupilot_gazebo'
    for d in [*_paths('GZ_SIM_SYSTEM_PLUGIN_PATH'), ws_install]:
        if (d / PLUGIN).exists():
            return d
    return None


def available() -> str | None:
    """None if a lockstep run can start here, else the reason it cannot."""
    if shutil.which('gz') is None:
        return 'gz not on PATH (source /opt/ros/jazzy/setup.bash)'
    if find_plugin_dir() is None:
        return f'{PLUGIN} not built (colcon build --packages-select ardupilot_gazebo)'
    return None


def world_path(name: str) -> pathlib.Path:
    return PKG / 'worlds' / f'{name}.sdf'


class GzServer:

    def __init__(self, world: str = 'flat_ci', log_path: str | os.PathLike | None = None):
        self.world = world
        self.log_path = log_path
        self.proc: subprocess.Popen | None = None

    def env(self) -> dict:
        env = dict(os.environ)
        res = [str(PKG / 'models'), str(PKG / 'worlds'), *map(str, _paths('GZ_SIM_RESOURCE_PATH'))]
        env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(res)
        plugins = [str(find_plugin_dir()), *map(str, _paths('GZ_SIM_SYSTEM_PLUGIN_PATH'))]
        env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(plugins)
        return env

    def start(self, timeout: float = 20.0) -> 'GzServer':
        out = open(self.log_path, 'w') if self.log_path else subprocess.DEVNULL
        self.proc = subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '--headless-rendering', str(world_path(self.world))],
            env=self.env(), stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(f'gz sim exited with {self.proc.returncode} (see {self.log_path})')
            if truth(timeout=2.0) is not None:
                return self
            time.sleep(0.5)
        self.stop()
        raise RuntimeError(f'gz sim never published {contract.GZ_ODOMETRY}')

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.stop()


def truth(timeout: float = 5.0) -> tuple[float, float, float] | None:
    """(east_m, north_m, heading_deg) of the model from Gazebo, or None.

    Heading is compass-style (0 = north, clockwise), comparable with the board's yaw.
    """
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.transport13 import Node

    got = []
    node = Node()
    node.subscribe(Odometry, f'/{contract.GZ_ODOMETRY}', got.append)
    deadline = time.monotonic() + timeout
    while not got and time.monotonic() < deadline:
        time.sleep(0.02)
    if not got:
        return None
    p, q = got[-1].pose.position, got[-1].pose.orientation
    yaw_enu = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, (90.0 - math.degrees(yaw_enu)) % 360.0
