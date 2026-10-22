"""Onboard compass calibration, run by a person holding the board.

    ros2 run rupsha_control compass_cal              # rotate the board until it says done
    ros2 run rupsha_control compass_cal --reboot     # and reboot so the offsets apply

Start it, then turn the board slowly through every orientation: each face down, then
roll and yaw it around. ArduPilot fits the offsets on the board and saves them when the
fit passes. Exit codes: 0 saved, 1 failed or cancelled, 2 no board.
"""
from __future__ import annotations

import argparse
import sys
import time

from .manifest import Companion
from .param_audit import resolve_url

MAV_CMD_DO_START_MAG_CAL = 42424
MAV_CMD_DO_CANCEL_MAG_CAL = 42426
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MSG_MAG_CAL_PROGRESS, MSG_MAG_CAL_REPORT = 191, 192
SUCCESS = 4
STATUS = {0: 'not started', 1: 'waiting to start', 2: 'step one', 3: 'step two', 4: 'success',
          5: 'failed', 6: 'bad orientation', 7: 'bad radius'}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--url', default='/dev/serial/by-id/usb-ArduPilot_Pixhawk1*')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=180.0, help='seconds before giving up')
    ap.add_argument('--reboot', action='store_true', help='reboot the board after a saved fit')
    args = ap.parse_args(argv)

    from .fc.ardurover import ArduRover
    from .fc.base import CommandRejected, LinkDown
    try:
        fc = ArduRover(resolve_url(args.url), baud=args.baud, sysid=Companion.TOOLS_SYSID)
        fc.wait_board(timeout=15.0)
    except (LinkDown, SystemExit) as e:
        print(e)
        return 2
    fc.start()
    try:
        for msg_id in (MSG_MAG_CAL_PROGRESS, MSG_MAG_CAL_REPORT):
            fc.command(MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 200_000)
        # all compasses, no retry, autosave, no delay, no autoreboot
        fc.command(MAV_CMD_DO_START_MAG_CAL, 0, 0, 1, 0, 0)
        print('calibrating: rotate the board slowly through every orientation')
        started = time.monotonic()
        last = ''
        while time.monotonic() - started < args.timeout:
            rep = fc.raw('MAG_CAL_REPORT', max_age=time.monotonic() - started)
            if rep is not None and rep.cal_status >= SUCCESS:
                ok = rep.cal_status == SUCCESS and rep.autosaved
                print(f'compass {rep.compass_id}: {STATUS.get(rep.cal_status, rep.cal_status)}, '
                      f'fitness {rep.fitness:.1f} mG, offsets ({rep.ofs_x:.0f}, {rep.ofs_y:.0f}, '
                      f'{rep.ofs_z:.0f}), saved {bool(rep.autosaved)}')
                if ok and args.reboot:
                    fc.reboot()
                    print('rebooting so the offsets apply')
                elif ok:
                    print('reboot the board so the offsets apply')
                return 0 if ok else 1
            prog = fc.raw('MAG_CAL_PROGRESS', max_age=2.0)
            if prog is not None:
                line = f'  {STATUS.get(prog.cal_status, prog.cal_status)}: {prog.completion_pct}%'
                if line != last:
                    print(line)
                    last = line
            time.sleep(0.2)
        print(f'no result within {args.timeout:.0f} s; cancelling')
        fc.command(MAV_CMD_DO_CANCEL_MAG_CAL, 0)
        return 1
    except KeyboardInterrupt:
        fc.command(MAV_CMD_DO_CANCEL_MAG_CAL, 0)
        print('cancelled')
        return 1
    except CommandRejected as e:
        print(f'refused: {e}')
        return 1
    finally:
        fc.close()


if __name__ == '__main__':
    sys.exit(main())
