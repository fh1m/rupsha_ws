"""Find and run ArduRover SITL.

    from rupsha_sim.sitl import Sitl
    with Sitl(model='rover-skid') as s:      # built-in physics, no Gazebo
        url = s.url                          # tcp:127.0.0.1:5760
    with Sitl(model='JSON') as s:            # physics from Gazebo's ArduPilotPlugin
        ...

The binary is launched directly (no sim_vehicle.py, no MAVProxy): the manager
is the only MAVLink peer, exactly as on the rover. ArduPilot is located by
$ARDUPILOT_HOME, else the usual checkout places; the binary must already be
built (`./waf configure --board sitl && ./waf rover`).
"""

from __future__ import annotations

import os
import pathlib
import shutil
import socket
import subprocess
import tempfile
import time

CANDIDATES = ('~/tmp/ardupilot', '~/ardupilot', '/opt/ardupilot', '~/src/ardupilot')
PKG = pathlib.Path(__file__).resolve().parent.parent
# Canberra model-aircraft field, ArduPilot's default SITL home; worlds set their own.
DEFAULT_HOME = (-35.363262, 149.165237, 584.0, 0.0)


def find_ardupilot() -> pathlib.Path | None:
    env = os.environ.get('ARDUPILOT_HOME')
    # also an `ardupilot/` beside any parent of this checkout (~/tmp/{rupsha_ws,ardupilot}),
    # which holds whatever HOME is -- the distrobox home is not the host one
    siblings = [str(d / 'ardupilot') for d in PKG.parents]
    for c in ([env] if env else []) + list(CANDIDATES) + siblings:
        p = pathlib.Path(c).expanduser()
        if (p / 'build/sitl/bin/ardurover').is_file():
            return p
    return None


def _share(pkg: str, rel: str) -> pathlib.Path:
    src = PKG.parent / pkg / rel
    if src.exists():
        return src
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory(pkg)) / rel


def default_params(model: str) -> list[pathlib.Path]:
    ap = find_ardupilot()
    base = [ap / 'Tools/autotest/default_params/rover.parm',
            ap / 'Tools/autotest/default_params/rover-skid.parm',
            _share('rupsha_control', 'config/ardupilot/rupsha_baseline.parm')]
    if model == 'JSON':
        base.append(_share('rupsha_sim', 'config/ardupilot/rupsha_gazebo.parm'))
    return base


def port_open(host: str, port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.2)
        return s.connect_ex((host, port)) == 0


class Sitl:

    def __init__(self, model: str = 'rover-skid', home=DEFAULT_HOME, instance: int = 0,
                 speedup: float = 1.0, extra_params=(), wipe: bool = True, log_dir=None,
                 serial0: str | None = None):
        """serial0: override SITL's SERIAL0 (default a TCP server on `port`), e.g.
        'tcpclient:192.168.0.102:5760' to dial out to a companion that listens --
        the direction a host firewall lets through for Pi-in-the-loop."""
        self.ap = find_ardupilot()
        if self.ap is None:
            raise FileNotFoundError('ArduRover SITL not built; set ARDUPILOT_HOME '
                                    '(./waf configure --board sitl && ./waf rover)')
        self.model, self.home, self.instance = model, home, instance
        self.speedup, self.wipe = speedup, wipe
        self.params = default_params(model) + [pathlib.Path(p) for p in extra_params]
        self.port = 5760 + 10 * instance
        self.url = f'tcp:127.0.0.1:{self.port}'
        self.serial0 = serial0
        self._own_dir = log_dir is None
        self.dir = pathlib.Path(log_dir or tempfile.mkdtemp(prefix='rupsha_sitl_'))
        self.proc = None

    def command(self) -> list[str]:
        cmd = [str(self.ap / 'build/sitl/bin/ardurover'), '--model', self.model,
               '--speedup', str(self.speedup), '-I', str(self.instance),
               '--home', ','.join(repr(float(v)) for v in self.home),
               '--defaults', ','.join(str(p) for p in self.params)]
        if self.wipe:
            cmd.append('-w')
        if self.serial0:
            cmd += ['--serial0', self.serial0]
        return cmd

    def start(self, timeout: float = 20.0) -> 'Sitl':
        if port_open('127.0.0.1', self.port):
            raise RuntimeError(f'port {self.port} busy: another SITL instance is running')
        self.dir.mkdir(parents=True, exist_ok=True)
        self.log = open(self.dir / 'sitl.out', 'w')
        self.proc = subprocess.Popen(self.command(), cwd=self.dir, stdout=self.log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(f'SITL exited: {(self.dir / "sitl.out").read_text()[-800:]}')
            # with SERIAL0 dialling out, SERIAL1 (port + 2) is the readiness signal
            if port_open('127.0.0.1', self.port + 2 if self.serial0 else self.port):
                return self
            time.sleep(0.1)
        self.stop()
        raise TimeoutError(f'SITL did not open {self.port} within {timeout:.0f} s')

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(5.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if getattr(self, 'log', None):
            self.log.close()
        if self._own_dir:
            shutil.rmtree(self.dir, ignore_errors=True)

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.stop()
