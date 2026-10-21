"""Diff a board's parameters against Rupsha's .parm files, and optionally apply.

    ros2 run rupsha_control param_audit --url /dev/serial/by-id/usb-ArduPilot_Pixhawk1*
    ros2 run rupsha_control param_audit --url tcp:127.0.0.1:5760 --apply

An empty diff is the pass condition. Exit codes: 0 match, 1 diff, 2 cannot read.
Parameters that need a reboot to take effect are listed after an --apply.
"""

from __future__ import annotations

import argparse
import glob
import math
import pathlib
import sys

CONFIG = pathlib.Path(__file__).resolve().parent.parent / 'config' / 'ardupilot'
BASELINE = 'rupsha_baseline.parm'

# Read at boot only; an --apply that touches one of these is not done until a reboot.
REBOOT_REQUIRED = {'VISO_TYPE', 'BRD_RTC_TYPES', 'SERVO1_FUNCTION', 'SERVO3_FUNCTION',
                   'MOT_PWM_TYPE', 'GPS_TYPE', 'EK3_ENABLE', 'AHRS_EKF_TYPE'}


def config_dir() -> pathlib.Path:
    if CONFIG.is_dir():
        return CONFIG
    try:
        from ament_index_python.packages import get_package_share_directory
        return pathlib.Path(get_package_share_directory('rupsha_control')) / 'config' / 'ardupilot'
    except Exception:                                  # noqa: BLE001
        return CONFIG


def parse_parm(text: str) -> dict:
    """MAVProxy/MissionPlanner .parm: `NAME,value` or `NAME value`, # comments."""
    out = {}
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.replace(',', ' ').split()
        if len(parts) != 2:
            raise ValueError(f'line {n}: cannot parse {raw!r}')
        name, value = parts
        if name in out:
            raise ValueError(f'line {n}: {name} set twice')
        out[name] = float(value)
    return out


def load(*names: str) -> dict:
    merged = {}
    for name in names:
        path = pathlib.Path(name)
        if not path.is_file():
            path = config_dir() / name
        merged.update(parse_parm(path.read_text()))
    return merged


def diff(expected: dict, actual: dict) -> list:
    """[(name, expected, actual_or_nan)] for every mismatch. float32 tolerance."""
    out = []
    for name, want in sorted(expected.items()):
        have = actual.get(name, math.nan)
        if math.isnan(have) or not math.isclose(have, want, rel_tol=1e-6, abs_tol=1e-5):
            out.append((name, want, have))
    return out


def resolve_url(url: str) -> str:
    if any(ch in url for ch in '*?['):
        hits = sorted(glob.glob(url))
        if not hits:
            raise SystemExit(f'no device matches {url}')
        return hits[0]
    return url


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--url', default='/dev/serial/by-id/usb-ArduPilot_Pixhawk1*')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--parm', action='append', help='parm file(s); default the baseline')
    ap.add_argument('--apply', action='store_true', help='set every differing parameter')
    ap.add_argument('--dump', help='also write the full board parameter set here')
    args = ap.parse_args(argv)

    from .fc.ardurover import ArduRover
    from .fc.base import CommandRejected
    expected = load(*(args.parm or [BASELINE]))
    fc = ArduRover(resolve_url(args.url), baud=args.baud)
    try:
        fc.wait_board(timeout=15.0)
        fc.start()
        actual = fc.fetch_all_params()
        if not actual:
            print('no parameters received')
            return 2
        if args.dump:
            pathlib.Path(args.dump).write_text(
                ''.join(f'{k},{v:.9g}\n' for k, v in sorted(actual.items())))
        delta = diff(expected, actual)
        if args.apply and delta:
            for name, want, _ in delta:
                try:
                    fc.set_param(name, want)
                    print(f'set {name} = {want:g}')
                except CommandRejected as e:
                    print(f'refused: {e}')
            needs_reboot = sorted({n for n, _, _ in delta} & REBOOT_REQUIRED)
            if needs_reboot:
                print('REBOOT REQUIRED for:', ', '.join(needs_reboot))
            actual.update({n: fc.get_param(n) for n, _, _ in delta})
            delta = diff(expected, actual)
        print(f'{len(expected)} expected, {len(actual)} on board, {len(delta)} differ')
        for name, want, have in delta:
            print(f'  {name:<20} want {want:<10g} board {have:g}')
        return 1 if delta else 0
    finally:
        fc.close()


if __name__ == '__main__':
    sys.exit(main())
