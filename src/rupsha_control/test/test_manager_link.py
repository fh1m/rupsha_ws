"""The manager waits for a board that is not up yet, instead of dying."""
import logging

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('rupsha_interfaces.msg')

from rupsha_control import rover_manager_node as mgr  # noqa: E402
from rupsha_control.fc.base import LinkDown  # noqa: E402


class Board:
    closed = 0

    def __init__(self, url):
        self.url = url

    def wait_board(self, timeout):
        pass

    def close(self):
        Board.closed += 1


def _refusing(n):
    calls = []

    def make(url):
        calls.append(url)
        if len(calls) <= n:
            raise ConnectionRefusedError(111, 'Connection refused')
        return Board(url)
    return make, calls


def test_retries_until_the_board_answers():
    make, calls = _refusing(3)
    fc = mgr._connect_with_retry(make, lambda: 'tcp:127.0.0.1:5760', logging.getLogger(), 0.0,
                                 retry_s=0.0)
    assert isinstance(fc, Board) and len(calls) == 4


def test_a_bounded_wait_gives_up_with_link_down():
    make, _ = _refusing(10_000)
    with pytest.raises(LinkDown):
        mgr._connect_with_retry(make, lambda: 'tcp:127.0.0.1:5760', logging.getLogger(), 0.05,
                                retry_s=0.01)


def test_a_board_that_never_heartbeats_is_closed_before_the_retry():
    class Silent(Board):
        def wait_board(self, timeout):
            raise LinkDown('no autopilot HEARTBEAT')
    Board.closed = 0
    with pytest.raises(LinkDown):
        mgr._connect_with_retry(Silent, lambda: 'x', logging.getLogger(), 0.05, retry_s=0.01)
    assert Board.closed >= 1


def test_a_missing_usb_device_is_waited_for():
    urls = iter([SystemExit('no device matches /dev/serial/by-id/x'), '/dev/ttyACM0'])

    def url_of():
        u = next(urls)
        if isinstance(u, BaseException):
            raise u
        return u
    fc = mgr._connect_with_retry(Board, url_of, logging.getLogger(), 0.0, retry_s=0.0)
    assert fc.url == '/dev/ttyACM0'
