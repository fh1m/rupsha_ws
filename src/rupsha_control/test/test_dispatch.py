import threading
import time

from rupsha_control.dispatch import BYPASS_VERBS, CommandGate, fields_for, goal_acceptance
from rupsha_control.manifest import VERBS


class Goal:
    """Zero-initialised like a rosidl Move.Goal."""
    def __init__(self, **kw):
        from rupsha_control.manifest import FIELDS
        for name, (t, _) in FIELDS.items():
            setattr(self, name, '' if t == 'string' else (-1 if name == 'tag_id' else 0))
        for k, v in kw.items():
            setattr(self, k, v)


def test_policy():
    assert goal_acceptance('move_forward', busy=False) == (True, False)
    assert goal_acceptance('move_forward', busy=True) == (False, False)
    for v in ('stop', 'abort', 'disarm', 'disable_autonomy'):
        assert v in BYPASS_VERBS
        assert goal_acceptance(v, busy=True) == (True, True)
    assert goal_acceptance('no_such_verb', busy=False) == (False, False)


def test_gate_admits_exactly_one_of_many_racing_claims():
    gate = CommandGate()
    wins = []
    barrier = threading.Barrier(16)

    def claim():
        barrier.wait()
        if gate.claim('move_forward')[0]:
            wins.append(1)
    threads = [threading.Thread(target=claim) for _ in range(16)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len(wins) == 1


def test_preempted_verb_does_not_release_the_stop_that_preempted_it():
    gate = CommandGate()
    gate.claim('move_forward')
    assert gate.claim('stop') == (True, True)
    gate.release('move_forward')
    assert gate.holder == 'stop'
    assert gate.claim('arm')[0] is False


def test_drain_blocks_ordinary_verbs_but_not_stop():
    gate = CommandGate()
    gate.claim('move_forward')
    gate.begin_drain()
    assert gate.claim('arm')[0] is False
    assert gate.claim('stop')[0] is True


def test_unconfirmed_claim_is_reaped():
    gate = CommandGate(stale_s=0.01)
    gate.claim('arm')
    time.sleep(0.02)
    assert gate.reap_stale() == 'arm'
    assert not gate.active


def test_fields_for_applies_defaults_only_to_unset():
    kw = fields_for('move_forward', Goal(duration=3.0))
    assert kw == {'duration': 3.0, 'speed': VERBS['move_forward']['defaults']['speed']}
    kw = fields_for('move_forward', Goal(duration=3.0, speed=0.8))
    assert kw['speed'] == 0.8
    kw = fields_for('vision_approach', Goal(tag_id=0))
    assert kw['tag_id'] == 0, 'tag 0 is a real id and must survive defaulting'
