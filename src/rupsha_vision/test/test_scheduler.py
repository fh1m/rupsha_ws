"""The phase table: which detectors each executive state runs."""
import pathlib

import pytest

from rupsha_vision import scheduler as S

A, O = S.ARUCO, S.OBJECTS  # noqa: E741

EXPECTED = {
    'IDLE': set(),
    'NAVIGATING': {A},
    'SEARCH_PATTERN': {A, O},
    'APPROACHING_MARKER': {A},
    'APPROACHING_OBJECT': {O},
    'VERIFYING_POSITION': set(),
    'VERIFYING_MARKER': {A},
    'VERIFYING_OBJECT': {O},
    'REVERSING': set(),
    'STUCK': set(),
}


@pytest.mark.parametrize('state,want', sorted(EXPECTED.items()))
def test_state_table(state, want):
    run, missing = S.schedule(state)
    assert run == want and missing == frozenset()


def test_phase_names_from_the_spec():
    assert S.detectors_for_phase('navigating') == {A}
    assert S.detectors_for_phase('search') == {A, O}
    assert S.detectors_for_phase('approach_marker') == {A}
    assert S.detectors_for_phase('approach_object') == {O}
    assert S.detectors_for_phase('idle') == frozenset()


def test_unavailable_detector_is_reported_not_silently_dropped():
    run, missing = S.schedule('SEARCH_PATTERN', available={A})
    assert run == {A} and missing == {O}
    run, missing = S.schedule('APPROACHING_OBJECT', available={A})
    assert run == frozenset() and missing == {O}


def test_unknown_state_and_phase_raise():
    with pytest.raises(KeyError, match='DOCKING'):
        S.schedule('DOCKING')
    with pytest.raises(KeyError, match='hover'):
        S.detectors_for_phase('hover')


def _manifest_state_names():
    """AutonomyState from the generated module, or -- if that module does not
    import -- from the manifest YAML it is generated from. Either way the
    names come from the source of truth, not from this package."""
    try:
        from rupsha_control.manifest import AutonomyState
    except (ImportError, SyntaxError):
        AutonomyState = None
    if AutonomyState is not None:
        return [s.name for s in AutonomyState]
    yaml = pytest.importorskip('yaml')
    for parent in pathlib.Path(__file__).resolve().parents:
        f = parent / 'manifest' / 'rupsha.yaml'
        if f.is_file():
            data = yaml.safe_load(f.read_text())
            break
    else:
        pytest.skip('manifest/rupsha.yaml not found')

    def find(node):
        if isinstance(node, dict):
            if 'AutonomyState' in node:
                return node['AutonomyState']
            for v in node.values():
                r = find(v)
                if r is not None:
                    return r
        return None
    values = find(data)['values']
    return [k for k, _ in sorted(values.items(), key=lambda kv: kv[1])]


def test_every_manifest_state_has_a_phase():
    names = _manifest_state_names()
    assert 'SEARCH_PATTERN' in names
    S.check_tables(names)
    assert set(names) == set(S.STATE_PHASE)


def test_check_tables_catches_a_new_state():
    with pytest.raises(KeyError, match='DOCKING'):
        S.check_tables(list(S.STATE_PHASE) + ['DOCKING'])
