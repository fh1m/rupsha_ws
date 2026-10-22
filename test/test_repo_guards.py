"""Whole-repo invariants. Each one is an architectural rule that code review
alone let slip at least once in mongla; here they fail CI instead.
"""

import inspect
import pathlib
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

from rupsha_control.manifest import VERBS
from rupsha_control.rupsha import Rupsha
from rupsha_planner.executive import Executive

WS = pathlib.Path(__file__).resolve().parent.parent
SRC = WS / 'src'
PACKAGES = sorted(p.parent for p in SRC.glob('rupsha_*/package.xml'))
OWNER_NAME, OWNER_EMAIL = 'Muhammad Fahim Faisal', 'fh1m.dev@gmail.com'
NODE_HANDLED = {'start_autonomy', 'disable_autonomy'}       # executive_node itself


def py_sources(include_tests=False):
    for pkg in PACKAGES:
        for f in pkg.rglob('*.py'):
            if include_tests or 'test' not in f.relative_to(pkg).parts[:1]:
                yield f


def test_manifest_outputs_are_fresh():
    r = subprocess.run([sys.executable, str(WS / 'tools' / 'gen_manifest.py'), '--check'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_tools_never_speak_as_the_companion():
    """A tool that speaks as the failsafe sysid and exits latches the board in HOLD."""
    from rupsha_control.manifest import Companion
    from rupsha_control.param_audit import BASELINE, load
    assert Companion.TOOLS_SYSID != Companion.SYSID
    assert load(BASELINE)['MAV_GCS_SYSID'] == Companion.SYSID != Companion.TOOLS_SYSID
    # the manager is the companion; SITL tests may be, since SITL restarts per test
    companions = {'rover_manager_node.py', 'conftest.py', 'test_sitl_scenarios.py'}
    files = list(py_sources(include_tests=True)) + list((WS / 'tools').glob('*.py'))
    for f in files:
        for call in re.findall(r'ArduRover\((.*)$', f.read_text(), re.M):
            if f.name not in companions and f.name != 'ardurover.py':
                assert 'sysid=' in call, f'{f.relative_to(WS)}: ArduRover({call}) speaks as {Companion.SYSID}'


def test_only_the_driver_imports_pymavlink():
    """One node touches the board, through one module."""
    pat = re.compile(r'^\s*(import pymavlink|from pymavlink)', re.M)
    hits = sorted(str(f.relative_to(SRC)) for f in py_sources(include_tests=True)
                  if pat.search(f.read_text()))
    assert hits == ['rupsha_control/rupsha_control/fc/ardurover.py']


def _handler(verb):
    runs_on = VERBS[verb]['runs_on']
    if runs_on == 'executive':
        return None if verb in NODE_HANDLED else getattr(Executive, verb, None)
    return getattr(Rupsha, verb, None)


@pytest.mark.parametrize('verb', sorted(VERBS))
def test_every_verb_is_reachable_with_the_manifest_fields(verb):
    if verb in NODE_HANDLED:
        src = (SRC / 'rupsha_planner' / 'rupsha_planner' / 'executive_node.py').read_text()
        assert f"verb == '{verb}'" in src
        return
    fn = _handler(verb)
    assert callable(fn), f'{verb} ({VERBS[verb]["runs_on"]}) has no handler'
    params = [p for p in inspect.signature(fn).parameters if p != 'self']
    assert params == list(VERBS[verb]['fields']), f'{verb}: handler {params} vs manifest'


def test_no_topic_literals_outside_the_manifest():
    """Topics come from rupsha_control.manifest.Topics; a literal is a fork."""
    pat = re.compile(r"""['"]/(rupsha|sim)/[a-z_/]+['"]""")
    allowed = {'rupsha_control/rupsha_control/manifest.py', 'rupsha_sim/rupsha_sim/contract.py'}
    hits = [f'{f.relative_to(SRC)}: {m.group(0)}' for f in py_sources()
            if str(f.relative_to(SRC)) not in allowed for m in pat.finditer(f.read_text())]
    assert not hits, hits


@pytest.mark.parametrize('pkg', PACKAGES, ids=lambda p: p.name)
def test_package_metadata_names_the_author(pkg):
    root = ET.parse(pkg / 'package.xml').getroot()
    m = root.find('maintainer')
    assert (m.text, m.get('email')) == (OWNER_NAME, OWNER_EMAIL)
    assert root.findtext('license') == 'MIT'
    setup = pkg / 'setup.py'
    if setup.exists():
        text = setup.read_text()
        assert f"maintainer='{OWNER_NAME}'" in text and f"maintainer_email='{OWNER_EMAIL}'" in text


@pytest.mark.parametrize('pkg', [p for p in PACKAGES if p.name != 'rupsha_interfaces'],
                         ids=lambda p: p.name)
def test_every_code_package_has_tests(pkg):
    assert list((pkg / 'test').glob('test_*.py')), f'{pkg.name} has no tests'
