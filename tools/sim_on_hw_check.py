#!/usr/bin/env python3
"""Sim-on-Hardware: the real Pixhawk running ArduRover with SITL physics built in.

Flash first (ArduPilot tree at Rover-4.7.2):

    Tools/scripts/sitl-on-hardware/sitl-on-hw.py --board Pixhawk1 --vehicle rover --frame rover-skid
    Tools/scripts/uploader.py --port '/dev/serial/by-id/usb-ArduPilot_Pixhawk1*' build/Pixhawk1/bin/ardurover.apj

then, with NOTHING attached to the board's outputs:

    tools/sim_on_hw_check.py [--url ...] [--out result.json]

The same verbs as the SITL scenarios, through the real driver and facade, scored
against SIMSTATE. The board's CPU load comes from SYS_STATUS. Restore the Rover
firmware and run `param_audit` afterwards.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import threading
import time

WS = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(WS / 'src' / 'rupsha_control')]

from rupsha_control import geo  # noqa: E402
from rupsha_control.fc.ardurover import ArduRover  # noqa: E402
from rupsha_control.manifest import Companion  # noqa: E402
from rupsha_control.rupsha import Rupsha  # noqa: E402


def heartbeat(fc, stop):
    while not stop.is_set():
        fc.send_heartbeat()
        time.sleep(0.5)


def truth(fc):
    s = fc.raw('SIMSTATE', 1.0)
    return None if s is None else (s.lat * 1e-7, s.lng * 1e-7)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--url', default='/dev/serial/by-id/usb-ArduPilot_Pixhawk1*')
    ap.add_argument('--out', type=pathlib.Path)
    args = ap.parse_args(argv)
    url = next(iter(sorted(pathlib.Path('/').glob(args.url.lstrip('/')))), args.url)

    fc = ArduRover(str(url), sysid=Companion.TOOLS_SYSID)
    fc.wait_board(20)
    fc.start()
    stop = threading.Event()
    threading.Thread(target=heartbeat, args=(fc, stop), daemon=True).start()
    fc._set_interval('SIMSTATE', 5)
    report = {'firmware': '.'.join(map(str, fc.firmware))}
    try:
        deadline = time.monotonic() + 120
        while time.monotonic() < deadline:
            t = fc.telemetry()
            if t.ekf_ok and t.gps_fix >= 3 and math.isfinite(t.north_m) and truth(fc):
                break
            time.sleep(0.5)
        else:
            report['error'] = f'never ready: {fc.statustexts()[-5:]}'
            return 1
        report['simstate'] = truth(fc) is not None
        report['ready_s'] = round(120 - (deadline - time.monotonic()), 1)

        r = Rupsha(fc)
        r.board_wp_radius = fc.get_param('WP_RADIUS')
        loads = []
        sampler_stop = threading.Event()

        def sample():
            while not sampler_stop.is_set():
                s = fc.raw('SYS_STATUS', 1.0)
                if s is not None:
                    loads.append(s.load / 10.0)
                time.sleep(0.5)
        threading.Thread(target=sample, daemon=True).start()

        res = r.run('arm')
        report['arm'] = res.message if not res.success else 'ok'
        if not res.success:
            return 1

        n0, e0 = fc.telemetry().north_m, fc.telemetry().east_m
        res = r.run('move_forward', duration=4.0, speed=1.0)
        t = fc.telemetry()
        report['move_forward'] = {'ok': res.success,
                                  'moved_m': round(math.hypot(t.north_m - n0, t.east_m - e0), 2)}

        target = (fc.telemetry().yaw_deg + 90) % 360
        res = r.run('turn_to', heading=target, timeout=30)
        report['turn_to'] = {'ok': res.success, 'elapsed_s': round(res.elapsed_s, 1),
                             'error_deg': round(abs(geo.heading_error(target, fc.telemetry().yaw_deg)), 1)}

        t = fc.telemetry()
        lat, lon = geo.offset(t.lat, t.lon, 10.0, 5.0)
        res = r.run('goto_gnss', lat=lat, lon=lon, radius=1.5, timeout=90)
        time.sleep(0.5)
        here = truth(fc)
        report['goto_gnss'] = {'ok': res.success, 'elapsed_s': round(res.elapsed_s, 1),
                               'truth_error_m': round(geo.distance_m(*here, lat, lon), 2) if here else None}

        r.run('stop')
        r.run('disarm')
        sampler_stop.set()
        if loads:
            report['cpu_load_pct'] = {'mean': round(sum(loads) / len(loads), 1), 'max': round(max(loads), 1)}
        return 0 if all(report[k]['ok'] for k in ('move_forward', 'turn_to', 'goto_gnss')) else 1
    finally:
        stop.set()
        fc.close()
        print(json.dumps(report, indent=1))
        if args.out:
            args.out.write_text(json.dumps(report, indent=1) + '\n')


if __name__ == '__main__':
    sys.exit(main())
