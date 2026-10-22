"""compass_cal against a fake board: what it sends, when it saves, when it reboots."""
from types import SimpleNamespace

import pytest

from rupsha_control import compass_cal
from rupsha_control.fc import ardurover
from rupsha_control.manifest import Companion


class FakeBoard:
    def __init__(self, url, baud=115200, sysid=Companion.SYSID, report=None):
        self.sysid, self.commands, self.rebooted, self._report = sysid, [], False, report

    def wait_board(self, timeout):
        pass

    def start(self):
        pass

    def close(self):
        pass

    def command(self, cmd, *params):
        self.commands.append((cmd, params))

    def raw(self, t, max_age=2.0):
        if t == 'MAG_CAL_PROGRESS':
            return SimpleNamespace(cal_status=2, completion_pct=40)
        if t == 'MAG_CAL_REPORT' and len(self.commands) >= 3:
            return self._report
        return None

    def reboot(self):
        self.rebooted = True


def _run(monkeypatch, report, *argv):
    boards = []

    def make(*a, **kw):
        boards.append(FakeBoard(*a, report=report, **kw))
        return boards[0]
    monkeypatch.setattr(ardurover, 'ArduRover', make)
    rc = compass_cal.main(['--url', '/dev/null', *argv])
    return rc, boards[0]


def _report(status, saved):
    return SimpleNamespace(compass_id=0, cal_status=status, autosaved=saved, fitness=12.0,
                           ofs_x=100.0, ofs_y=-20.0, ofs_z=50.0)


def test_a_saved_fit_passes_and_reboots_only_when_asked(monkeypatch):
    rc, fc = _run(monkeypatch, _report(compass_cal.SUCCESS, 1))
    assert rc == 0 and not fc.rebooted
    assert fc.sysid == Companion.TOOLS_SYSID
    assert (compass_cal.MAV_CMD_DO_START_MAG_CAL, (0, 0, 1, 0, 0)) in fc.commands
    rc, fc = _run(monkeypatch, _report(compass_cal.SUCCESS, 1), '--reboot')
    assert rc == 0 and fc.rebooted


@pytest.mark.parametrize('status,saved', [(5, 0), (7, 0), (compass_cal.SUCCESS, 0)])
def test_a_failed_or_unsaved_fit_fails(monkeypatch, status, saved):
    rc, fc = _run(monkeypatch, _report(status, saved), '--reboot')
    assert rc == 1 and not fc.rebooted


def test_no_result_cancels_the_calibration(monkeypatch):
    rc, fc = _run(monkeypatch, None, '--timeout', '0.3')
    assert rc == 1
    assert fc.commands[-1][0] == compass_cal.MAV_CMD_DO_CANCEL_MAG_CAL
