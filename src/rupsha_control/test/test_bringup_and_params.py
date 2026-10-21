import math
import pathlib

import pytest

from rupsha_control.bringup_check import ABSENT, FAIL, PASS, WARN, evaluate, verdict
from rupsha_control.fc.base import Telemetry
from rupsha_control.manifest import Companion
from rupsha_control.param_audit import config_dir, diff, load, parse_parm

ROOT = pathlib.Path(__file__).resolve().parents[3]
FACTORY = ROOT / '.claude/context/platform/records/ardurover_4.7.2_factory.parm'


def good_tel(**kw):
    t = Telemetry(link_alive=True, armed=False, mode='HOLD', firmware=(4, 7, 2), gps_fix=3,
                  gps_sats=10, ekf_ok=True, prearm_ok=True)
    for k, v in kw.items():
        setattr(t, k, v)
    return t


def rows_by_name(rows):
    return {n: v for v, n, _ in rows}


def test_baseline_names_all_exist_on_ardurover_4_7_2():
    factory = parse_parm(FACTORY.read_text())
    for f in config_dir().glob('*.parm'):
        for name in parse_parm(f.read_text()):
            assert name in factory, f'{f.name}: {name} is not an ArduRover 4.7.2 parameter'


def test_baseline_pins_the_manifest_companion_sysid():
    base = load('rupsha_baseline.parm')
    assert base['MAV_GCS_SYSID'] == Companion.SYSID
    assert base['FS_GCS_ENABLE'] >= 1


def test_baseline_does_not_touch_ekf_source_set_two():
    assert not any(k.startswith('EK3_SRC2') or k == 'VISO_TYPE' for k in load('rupsha_baseline.parm'))


def test_parse_rejects_duplicates_and_accepts_both_separators():
    assert parse_parm('A,1\nB 2.5  # c\n') == {'A': 1.0, 'B': 2.5}
    with pytest.raises(ValueError):
        parse_parm('A,1\nA,2\n')


def test_diff_float32_tolerance_and_missing():
    assert diff({'A': 0.1}, {'A': 0.10000000149}) == []
    d = diff({'A': 1.0, 'B': 2.0}, {'A': 1.0})
    assert d[0][0] == 'B' and math.isnan(d[0][2])


def test_bringup_all_good():
    expected = load('rupsha_baseline.parm')
    rows = evaluate(good_tel(), dict(expected), expected, [], require_gps=True)
    assert verdict(rows) == 0, rows


def test_bringup_absent_is_not_pass():
    expected = load('rupsha_baseline.parm')
    rows = evaluate(good_tel(firmware=(), prearm_ok=None), {}, expected, [], require_gps=True)
    r = rows_by_name(rows)
    assert r['firmware'] == ABSENT and r['params'] == ABSENT and r['pre-arm'] == ABSENT
    assert verdict(rows) == 1


def test_bringup_factory_board_fails_the_failsafe_check():
    factory = parse_parm(FACTORY.read_text())
    rows = evaluate(good_tel(), factory, load('rupsha_baseline.parm'), [], require_gps=True)
    r = rows_by_name(rows)
    assert r['companion failsafe'] == FAIL and r['baseline'] == FAIL


def test_bringup_flags_the_extnav_prearm_trap():
    expected = load('rupsha_baseline.parm')
    params = dict(expected, EK3_SRC2_POSXY=6.0, VISO_TYPE=0.0)
    assert rows_by_name(evaluate(good_tel(), params, expected, [], True))['ekf sources'] == FAIL


def test_bench_mode_downgrades_sky_checks():
    expected = load('rupsha_baseline.parm')
    rows = evaluate(good_tel(gps_fix=1, ekf_ok=False), dict(expected), expected, [], require_gps=False)
    r = rows_by_name(rows)
    assert r['gnss'] == WARN and r['ekf'] == WARN and verdict(rows) == 0


def test_prearm_reason_surfaces():
    expected = load('rupsha_baseline.parm')
    rows = evaluate(good_tel(prearm_ok=False), dict(expected), expected,
                    ['PreArm: Compass not calibrated'], True)
    detail = [d for v, n, d in rows if n == 'pre-arm'][0]
    assert 'Compass' in detail and PASS not in [v for v, n, _ in rows if n == 'pre-arm']
