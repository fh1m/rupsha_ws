import math
import threading
import time

import pytest

from rupsha_control.manifest import Outcome
from rupsha_sim import gazebo
from rupsha_sim.sitl import Sitl


@pytest.fixture(scope='module')
def gz_rover(tmp_path_factory):
    """Gazebo flat_ci + ArduRover SITL in JSON lockstep, armed: (fc, rupsha)."""
    from rupsha_control.fc.ardurover import ArduRover
    from rupsha_control.rupsha import Rupsha
    logs = tmp_path_factory.mktemp('gz')
    with gazebo.GzServer('flat_ci', log_path=logs / 'gz.log'), \
            Sitl(model='JSON', log_dir=str(logs)) as sitl:
        fc = ArduRover(sitl.url)
        fc.wait_board(30)
        fc.start()
        stop = threading.Event()

        def beat():
            while not stop.is_set():
                fc.send_heartbeat()
                time.sleep(0.5)
        threading.Thread(target=beat, daemon=True).start()
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            t = fc.telemetry()
            if t.ekf_ok and t.gps_fix >= 3 and math.isfinite(t.north_m):
                break
            time.sleep(0.5)
        else:
            pytest.fail(f'lockstep never became ready: {fc.telemetry()} {fc.statustexts()[-5:]}')
        r = Rupsha(fc)
        r.board_wp_radius = fc.get_param('WP_RADIUS')
        res = r.run('arm')
        assert res.outcome == Outcome.SUCCEEDED, f'{res.message} {fc.statustexts()[-5:]}'
        yield fc, r
        stop.set()
        r.run('disarm')
        fc.close()
