"""Board-in-the-loop bench: the real Pixhawk on USB, no motors, no GPS.

Everything that can be true on a desk is checked against the board itself:
firmware, baseline, link rates, mode changes, neutral outputs, and that when
the board refuses to arm its own reason reaches the caller. Skipped when no
Pixhawk1 is plugged in.

    pytest -m board src/rupsha_control
"""

import glob
import time

import pytest

from rupsha_control.manifest import Companion, Firmware, Outcome
from rupsha_control.param_audit import BASELINE, diff, load

PORTS = sorted(glob.glob('/dev/serial/by-id/usb-ArduPilot_Pixhawk1*'))

pytestmark = [pytest.mark.board,
              pytest.mark.skipif(not PORTS, reason='no Pixhawk1 on USB')]


@pytest.fixture(scope='module')
def board():
    from rupsha_control.fc.ardurover import ArduRover
    from rupsha_control.heartbeat import Heartbeat
    from rupsha_control.rupsha import Rupsha
    fc = ArduRover(PORTS[0], baud=115200)
    fc.wait_board(timeout=20)
    fc.start()
    beat = Heartbeat(fc.send_heartbeat, 1.0 / Companion.HEARTBEAT_HZ, None)
    beat.start()
    deadline = time.monotonic() + 10
    while not fc.telemetry().firmware and time.monotonic() < deadline:
        time.sleep(0.2)
    yield fc, Rupsha(fc)
    beat.stop()
    fc.close()


def test_firmware_is_the_pinned_ardurover(board):
    fc, _ = board
    assert tuple(fc.telemetry().firmware) >= Firmware.MIN_VERSION


def test_baseline_is_loaded(board):
    fc, _ = board
    params = fc.fetch_all_params()
    assert not diff(load(BASELINE), params)



def test_telemetry_arrives_at_the_baseline_rates(board):
    """SR0/MAV1 rates in the baseline are a contract with the manager's loops."""
    fc, _ = board
    seen = {'ATTITUDE': [], 'SYS_STATUS': []}
    listening = [True]
    fc.add_listener(lambda m: seen[m.get_type()].append(time.monotonic())
                    if listening[0] and m.get_type() in seen else None)
    time.sleep(3.0)
    listening[0] = False
    hz = {k: (len(v) - 1) / (v[-1] - v[0]) if len(v) > 1 else 0.0 for k, v in seen.items()}
    assert hz['ATTITUDE'] >= 8.0, hz
    assert hz['SYS_STATUS'] >= 1.5, hz


def test_disarmed_outputs_are_neutral(board):
    fc, _ = board
    m = fc.raw('SERVO_OUTPUT_RAW', 3.0)
    assert m is not None, 'no SERVO_OUTPUT_RAW'
    for ch in (1, 3):                                   # ThrottleLeft / ThrottleRight
        pwm = getattr(m, f'servo{ch}_raw')
        trim = fc.get_param(f'SERVO{ch}_TRIM')
        assert pwm in (0, int(trim)), f'SERVO{ch} {pwm} while disarmed (trim {trim:g})'


def test_mode_round_trip_while_disarmed(board):
    fc, r = board
    start = fc.telemetry().mode
    for mode in ('HOLD', 'MANUAL'):
        res = r.run('set_mode', target_name=mode)
        assert res.outcome == Outcome.SUCCEEDED, res.message
        assert fc.telemetry().mode == mode
    r.run('set_mode', target_name=start or 'MANUAL')


def test_an_arm_refusal_carries_the_boards_reason(board):
    """On the bench (no GPS, compass maybe uncalibrated) the board may refuse;
    the caller must see why, not 'failed'. If it arms, disarm at once."""
    fc, r = board
    res = r.run('arm')
    if res.outcome == Outcome.SUCCEEDED:
        assert r.run('disarm').outcome == Outcome.SUCCEEDED
        return
    assert res.outcome in (Outcome.DENIED, Outcome.NOT_READY)
    assert any(word in res.message for word in ('Arm', 'PreArm', 'arm')), res.message
    assert len(res.message) > len('arm refused: '), res.message
    assert not fc.telemetry().armed
