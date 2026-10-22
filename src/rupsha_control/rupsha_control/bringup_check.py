"""Is this board ready to be Rupsha's? One line per check, PASS / FAIL / ABSENT.

    ros2 run rupsha_control bringup_check                       # USB Pixhawk
    ros2 run rupsha_control bringup_check --url tcp:127.0.0.1:5760

ABSENT is not PASS: a check whose input never arrived says so instead of passing
on a default (mongla's gate passed on absent hardware once; never again). Exit
code is 0 only when nothing FAILED and nothing required is ABSENT.

Run it BEFORE the manager -- it opens the port itself.
"""

from __future__ import annotations

import argparse
import sys
import time

from .manifest import Companion, Firmware
from .param_audit import BASELINE, diff, load, resolve_url

PASS, FAIL, ABSENT, WARN = 'PASS', 'FAIL', 'ABSENT', 'WARN'


def evaluate(tel, params: dict, expected: dict, texts: list, require_gps: bool) -> list:
    """Pure: [(verdict, name, detail)]. Unit-tested without a board."""
    rows = []

    def add(v, name, detail):
        rows.append((v, name, detail))

    add(PASS if tel.link_alive else FAIL, 'link', 'HEARTBEAT fresh' if tel.link_alive else 'no HEARTBEAT')
    fw = tuple(tel.firmware)
    floor = '.'.join(map(str, Firmware.MIN_VERSION))
    if not fw:
        add(ABSENT, 'firmware', 'AUTOPILOT_VERSION never arrived')
    else:
        add(PASS if fw >= Firmware.MIN_VERSION else FAIL, 'firmware',
            f'{Firmware.VEHICLE} {".".join(map(str, fw))} (floor {floor})')
    if not params:
        add(ABSENT, 'params', 'no PARAM_VALUE received')
    else:
        delta = diff(expected, params)
        add(PASS if not delta else FAIL, 'baseline',
            'matches rupsha_baseline.parm' if not delta else
            f'{len(delta)} differ: ' + ', '.join(f'{n}={h:g}(want {w:g})' for n, w, h in delta[:6]))
        gcs = params.get('MAV_GCS_SYSID')
        ok = gcs == Companion.SYSID and params.get('FS_GCS_ENABLE', 0) >= 1
        add(PASS if ok else FAIL, 'companion failsafe',
            f'MAV_GCS_SYSID={gcs} FS_GCS_ENABLE={params.get("FS_GCS_ENABLE")} '
            f'(want {Companion.SYSID}, >=1)')
        viso = params.get('VISO_TYPE', 0)
        src2 = params.get('EK3_SRC2_POSXY', 0)
        if src2 == 6 and viso <= 0:
            add(FAIL, 'ekf sources', 'EK3_SRC2_POSXY=ExtNav but VISO_TYPE=0: pre-arm will fail')
        else:
            add(PASS, 'ekf sources', f'SRC2_POSXY={src2:g} VISO_TYPE={viso:g}')
    if tel.prearm_ok is None:
        add(ABSENT, 'pre-arm', 'SYS_STATUS has no pre-arm bit')
    else:
        prearm = [t for t in texts if t.startswith('PreArm')]
        add(PASS if tel.prearm_ok else FAIL, 'pre-arm',
            'board reports ready' if tel.prearm_ok else (prearm[-1] if prearm else 'not ready'))
    gps_v = PASS if tel.gps_fix >= 3 else (FAIL if require_gps else WARN)
    add(gps_v if tel.gps_fix >= 0 else (FAIL if require_gps else ABSENT), 'gnss',
        f'fix_type {tel.gps_fix}, {tel.gps_sats} sats')
    if tel.ekf_ok is None:
        add(ABSENT if require_gps else WARN, 'ekf', 'EKF_STATUS_REPORT never arrived')
    else:
        add(PASS if tel.ekf_ok else (FAIL if require_gps else WARN), 'ekf',
            'attitude + velocity + absolute position' if tel.ekf_ok else 'no absolute position')
    return rows


def verdict(rows) -> int:
    return 1 if any(v in (FAIL, ABSENT) for v, _, _ in rows) else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--url', default='/dev/serial/by-id/usb-ArduPilot_Pixhawk1*')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--parm', action='append')
    ap.add_argument('--settle', type=float, default=8.0, help='seconds to gather telemetry')
    ap.add_argument('--bench', action='store_true',
                    help='indoors / no sky: GNSS and EKF position are WARN, not FAIL')
    args = ap.parse_args(argv)

    from .fc.ardurover import ArduRover
    fc = ArduRover(resolve_url(args.url), baud=args.baud, sysid=Companion.TOOLS_SYSID)
    try:
        fc.wait_board(timeout=15.0)
        fc.start()
        end = time.monotonic() + args.settle
        while time.monotonic() < end:
            fc.send_heartbeat()
            time.sleep(0.5)
        params = fc.fetch_all_params()
        rows = evaluate(fc.telemetry(), params, load(*(args.parm or [BASELINE])),
                        fc.statustexts(), require_gps=not args.bench)
    finally:
        fc.close()
    width = max(len(n) for _, n, _ in rows)
    for v, name, detail in rows:
        print(f'[{v:<6}] {name:<{width}}  {detail}')
    code = verdict(rows)
    print('READY' if code == 0 else 'NOT READY')
    return code


if __name__ == '__main__':
    sys.exit(main())
