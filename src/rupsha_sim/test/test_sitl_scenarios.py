"""ArduRover 4.7.2 SITL with rupsha_baseline.parm, driven through the real driver.

Built-in skid-steer physics (no Gazebo), so these run anywhere the SITL binary
is built. Truth is SITL's SIMSTATE (the simulator's own position), not the EKF
estimate the rover navigated on -- arriving by the EKF and being there are
checked separately.

    pytest -m sitl src/rupsha_sim
"""

import math
import threading
import time

import pytest

from rupsha_sim.sitl import Sitl, find_ardupilot

pytestmark = [pytest.mark.sitl,
              pytest.mark.skipif(find_ardupilot() is None, reason='ArduRover SITL not built')]


class Beat:
    def __init__(self, fc):
        self.fc, self.on = fc, True
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop.is_set():
            if self.on:
                self.fc.send_heartbeat()
            time.sleep(0.5)

    def close(self):
        self._stop.set()


@pytest.fixture(scope='module')
def rover():
    from rupsha_control.fc.ardurover import ArduRover
    from rupsha_control.rupsha import Rupsha
    with Sitl(model='rover-skid') as sitl:
        fc = ArduRover(sitl.url)
        fc.wait_board(20)
        fc.start()
        beat = Beat(fc)
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            t = fc.telemetry()
            if t.ekf_ok and t.gps_fix >= 3 and t.firmware and math.isfinite(t.north_m):
                break
            time.sleep(0.5)
        else:
            pytest.fail(f'SITL never became ready: {fc.telemetry()} {fc.statustexts()[-5:]}')
        r = Rupsha(fc)
        r.board_wp_radius = fc.get_param('WP_RADIUS')
        yield fc, r, beat, sitl
        beat.close()
        fc.close()


def truth(fc):
    s = fc.raw('SIMSTATE', 1.0)
    assert s is not None, 'SITL SIMSTATE missing'
    return s.lat * 1e-7, s.lng * 1e-7


def test_baseline_is_loaded(rover):
    fc, *_ = rover
    assert fc.get_param('MAV_GCS_SYSID') == 245
    assert fc.get_param('GUID_OPTIONS') == 64
    assert fc.firmware >= (4, 7, 2)


def test_goto_gnss_arrives_by_truth(rover):
    from rupsha_control import geo
    fc, r, _, _ = rover
    fc._set_interval('SIMSTATE', 5)
    assert r.run('arm').success
    t = fc.telemetry()
    lat, lon = geo.offset(t.lat, t.lon, 20.0, 8.0)
    res = r.run('goto_gnss', lat=lat, lon=lon, radius=1.5, timeout=90)
    assert res.success, res.message
    time.sleep(0.5)
    err = geo.distance_m(*truth(fc), lat, lon)
    assert err <= 1.5 + 0.5, f'EKF said arrived, truth says {err:.2f} m out'
    r.run('stop')


def test_velocity_stream_drives_and_expires(rover):
    fc, r, _, _ = rover
    assert r.run('arm').success
    n0 = fc.telemetry().north_m
    e0 = fc.telemetry().east_m
    res = r.run('move_forward', duration=4.0, speed=1.0)
    assert res.success
    t = fc.telemetry()
    moved = math.hypot(t.north_m - n0, t.east_m - e0)
    assert moved > 2.0, f'1 m/s for 4 s moved only {moved:.2f} m: type_mask or mode wrong?'
    time.sleep(1.5)
    assert fc.telemetry().groundspeed < 0.3, 'the rover should stop once the stream ends'


def test_turn_to_settles(rover):
    from rupsha_control import geo
    fc, r, _, _ = rover
    assert r.run('arm').success
    target = (fc.telemetry().yaw_deg + 120) % 360
    res = r.run('turn_to', heading=target, timeout=30)
    assert res.success, res.message
    assert abs(geo.heading_error(target, fc.telemetry().yaw_deg)) < 5


def test_companion_heartbeat_loss_holds_the_rover(rover):
    """The reflex that makes a dead Pi safe: FS_GCS on sysid 245 -> HOLD."""
    fc, r, beat, _ = rover
    assert r.run('arm').success
    assert r.run('set_mode', target_name='GUIDED').success
    beat.on = False
    t0 = time.monotonic()
    held = None
    while time.monotonic() - t0 < 8.0:
        fc.velocity_body(1.0, 0.0)          # still streaming: busy is not dead, silence is
        if fc.telemetry().mode == 'HOLD':
            held = time.monotonic() - t0
            break
        time.sleep(0.1)
    beat.on = True
    assert held is not None, f'no HOLD after 8 s of silence: {fc.statustexts()[-5:]}'
    # the board acts at FS_GCS_TIMEOUT + FS_TIMEOUT; we see it in the next 1 Hz HEARTBEAT
    assert held <= 2.0 + 1.5 + 1.0, f'HOLD seen after {held:.1f} s; the board acts at 3.5 s'
    assert any('GCS' in t or 'Failsafe' in t for t in fc.statustexts())
    time.sleep(1.0)
    r.run('stop')
