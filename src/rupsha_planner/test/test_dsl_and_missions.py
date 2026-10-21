import logging

import pytest

from rupsha_control.manifest import VERBS
from rupsha_planner import mission as mission_runner
from rupsha_planner.dsl import MissionRefused, RupshaMission


def dsl(supported=None):
    port, fc = mission_runner.dry_port()
    return RupshaMission(port, supported=supported), fc


def test_every_manifest_verb_is_a_dsl_method_with_its_fields():
    for verb in VERBS:
        assert callable(getattr(RupshaMission, verb))


def test_unknown_field_is_a_typo_not_a_silent_default():
    r, _ = dsl()
    with pytest.raises(TypeError, match='no field'):
        r.move_forward(duration=1, sped=0.5)


def test_defaults_fill_and_positional_follow_manifest_order():
    r, fc = dsl()
    r.arm()
    res = r.move_forward(2.0)
    assert res.success and fc.north == pytest.approx(2 * VERBS['move_forward']['defaults']['speed'], abs=0.1)


def test_require_refuses_before_motion():
    r, fc = dsl(supported={'arm', 'move_forward'})
    with pytest.raises(MissionRefused):
        r.require('arm', 'start_autonomy')
    assert not fc.armed


def test_run_plan_skips_what_does_not_fit_and_abandons_overruns():
    r, fc = dsl()
    r.arm()
    r.use_budget(30)
    ran = []

    def short(m):
        ran.append('short')
        m.move_forward(duration=5)

    def overrun(m):
        ran.append('overrun')
        for _ in range(10):
            m.move_forward(duration=3)

    card = r.run_plan([
        r.step('short', points=10, worst_case_s=10, run=short),
        r.step('overrun', points=10, worst_case_s=8, run=overrun),
        r.step('huge', points=99, worst_case_s=500, run=short),
    ])
    names = {n: v for n, v, _ in card}
    assert names['short'] == 'full'
    assert names['overrun'].startswith('abandoned')
    assert names['huge'] == 'skipped'


MISSIONS = sorted(p.stem for p in mission_runner.MISSIONS.glob('*.py')
                  if not p.stem.startswith('_') and p.stem != 'urc_autonomy')


@pytest.mark.parametrize('name', MISSIONS)
def test_every_mission_dry_runs_to_completion(name):
    assert mission_runner.main([name, '--dry-run']) == 0


def test_square_drive_closes_the_square():
    port, fc = mission_runner.dry_port()
    r = RupshaMission(port)
    mission_runner.load('square_drive').run(r, logging.getLogger('test'))
    assert abs(fc.north) < 2.5 and abs(fc.east) < 2.5
    assert not fc.armed


def test_mission_exit_code_reports_a_failed_verb(tmp_path):
    ok = tmp_path / 'ok.py'
    ok.write_text('def run(rupsha, log):\n    rupsha.arm()\n    rupsha.move_forward(1.0)\n    rupsha.disarm()\n')
    unarmed = tmp_path / 'unarmed.py'
    unarmed.write_text('def run(rupsha, log):\n    rupsha.move_forward(1.0)\n')
    assert mission_runner.main([str(ok), '--dry-run']) == 0
    assert mission_runner.main([str(unarmed), '--dry-run']) == 1
