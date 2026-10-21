"""The mailbox's invariants: newest wins, every overwrite is counted, nothing is
served twice, and a consumer slower than the producer never sees the past.
"""
import math
import threading
import time

from rupsha_vision.mailbox import FrameMailbox


def test_newest_wins_and_overwrites_are_drops():
    mb = FrameMailbox()
    for i in range(5):
        mb.put(f'f{i}', stamp=i)
    frame, stamp, seq = mb.get(timeout=0)
    assert (frame, stamp, seq) == ('f4', 4, 5)
    assert mb.drops == 4 and mb.puts == 5 and mb.gets == 1


def test_a_frame_is_never_served_twice():
    mb = FrameMailbox()
    mb.put('a')
    assert mb.get(timeout=0)[0] == 'a'
    assert mb.get(timeout=0.01) is None
    assert not mb.pending()


def test_consumed_frames_are_not_drops():
    mb = FrameMailbox()
    for i in range(3):
        mb.put(i)
        assert mb.get(timeout=0)[0] == i
    assert mb.drops == 0


def test_get_times_out_with_none():
    mb = FrameMailbox()
    t0 = time.monotonic()
    assert mb.get(timeout=0.05) is None
    assert time.monotonic() - t0 >= 0.04


def test_get_wakes_on_put():
    mb = FrameMailbox()
    got = []
    th = threading.Thread(target=lambda: got.append(mb.get(timeout=2.0)))
    th.start()
    time.sleep(0.02)
    mb.put('x', stamp=1.5)
    th.join(1.0)
    assert got and got[0][:2] == ('x', 1.5)


def test_close_wakes_a_blocked_consumer():
    mb = FrameMailbox()
    got = []
    th = threading.Thread(target=lambda: got.append(mb.get(timeout=None)))
    th.start()
    time.sleep(0.02)
    mb.close()
    th.join(1.0)
    assert not th.is_alive() and got == [None]


def test_slow_consumer_sees_strictly_newer_frames_and_the_books_balance():
    """Producer at ~1 kHz, consumer sleeping 5 ms per frame: the consumer must
    skip ahead (drops > 0), every frame it gets must be newer than the last,
    and puts == gets + drops + (one still in the slot or not)."""
    mb = FrameMailbox()
    stop = threading.Event()
    seen = []

    def consume():
        while not stop.is_set() or mb.pending():
            item = mb.get(timeout=0.05)
            if item is not None:
                seen.append(item[2])
                time.sleep(0.005)

    th = threading.Thread(target=consume)
    th.start()
    for i in range(400):
        mb.put(i, stamp=i)
        time.sleep(0.001)
    stop.set()
    th.join(2.0)
    assert seen == sorted(set(seen))
    assert seen[-1] == 400, 'the last frame put must be the last frame consumed'
    assert mb.drops > 0
    assert mb.puts == mb.gets + mb.drops


def test_age_is_from_the_capture_instant():
    mb = FrameMailbox()
    assert math.isnan(mb.age_s())
    mb.put('a', capture_mono=100.0)
    assert mb.age_s(now_mono=100.25) == 0.25
    mb.get(timeout=0)
    assert mb.age_s(now_mono=101.0) == 1.0, 'consuming does not reset staleness'
