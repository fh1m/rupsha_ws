"""Wire facts about ArduRover 4.7.2 the driver must not drift from.

Each constant here was read from the Rover-4.7.2 source, not from a forum post;
the comment says where.
"""

import math

from rupsha_control.fc import ardurover as ar
from rupsha_control.loop_meter import LoopMeter
from rupsha_control.manifest import RoverMode

# MAVLINK_SET_POS_TYPE_MASK_* groups as Rover tests them with `mask & GROUP`.
GROUPS = {'pos': 0b111, 'vel': 0b111000, 'acc': 0b111000000, 'yaw': 1 << 10, 'yaw_rate': 1 << 11}


def _groups(mask):
    """Which groups the board will treat as IGNORED (any bit set ignores the group)."""
    return {g for g, bits in GROUPS.items() if mask & bits}


def test_every_mask_sets_whole_groups_only():
    for mask in (ar.MASK_POSITION, ar.MASK_VEL_YAWRATE, ar.MASK_YAWRATE_ONLY, ar.MASK_HEADING_ONLY):
        for g, bits in GROUPS.items():
            assert mask & bits in (0, bits), f'mask {mask}: partial {g} group'


def test_masks_mean_what_they_say():
    assert _groups(ar.MASK_POSITION) == {'vel', 'acc', 'yaw', 'yaw_rate'}
    assert _groups(ar.MASK_VEL_YAWRATE) == {'pos', 'acc', 'yaw'}
    assert _groups(ar.MASK_YAWRATE_ONLY) == {'pos', 'vel', 'acc', 'yaw'}
    assert _groups(ar.MASK_HEADING_ONLY) == {'pos', 'vel', 'acc', 'yaw_rate'}
    assert ar.MASK_VEL_YAWRATE == 1479


def test_the_folk_mask_1511_would_drop_velocity():
    assert 'vel' in _groups(1511)


def test_rover_mode_numbers():
    # Rover/mode.h, enum class Number, Rover-4.7.2
    assert (RoverMode.MANUAL, RoverMode.HOLD, RoverMode.AUTO, RoverMode.GUIDED) == (0, 4, 10, 15)


def test_version_decode():
    assert ar.parse_version((4 << 24) | (7 << 16) | (2 << 8) | 255) == (4, 7, 2)


def test_loop_meter_low1_sees_the_stall():
    t = [0.0]
    m = LoopMeter('x', clock=lambda: t[0])
    for i in range(200):
        t[0] += 0.3 if i == 100 else 0.1
        m.tick()
    s = m.stats()
    assert s['avg'] > 9.5
    assert math.isclose(s['low1'], 1 / ((0.3 + 0.1) / 2), rel_tol=1e-6)


def _driver_with(ages):
    """An ArduRover with no port, holding messages received `age` seconds ago."""
    import threading
    import time
    from collections import deque
    from types import SimpleNamespace
    d = ar.ArduRover.__new__(ar.ArduRover)
    d._lock, d._texts, d._firmware = threading.Lock(), deque(), (4, 7, 2)
    now = time.monotonic()
    msgs = {'HEARTBEAT': SimpleNamespace(base_mode=0, custom_mode=int(RoverMode.HOLD)),
            'ATTITUDE': SimpleNamespace(yaw=0.0, yawspeed=0.0)}
    d._msgs = {t: (now - ages[t], msgs[t]) for t in ages}
    return d


def test_a_slow_sim_with_fresh_telemetry_is_alive():
    """Gazebo at 0.4x real time spaces the 1 Hz HEARTBEAT 2.5 s apart; ATTITUDE still flows."""
    tel = _driver_with({'HEARTBEAT': 2.5, 'ATTITUDE': 0.1}).telemetry()
    assert tel.link_alive and tel.mode == 'HOLD'


def test_a_silent_board_is_dead():
    tel = _driver_with({'HEARTBEAT': ar.LINK_STALE_S + 0.5, 'ATTITUDE': ar.LINK_STALE_S + 0.5}).telemetry()
    assert not tel.link_alive and tel.armed is not True
