"""Pi-in-the-loop, laptop half: SITL that dials the Pi, and the truth it drove.

The rover's real companion (Pi 4, rupsha_pi:jazzy container) runs the real
manager listening on tcpin:0.0.0.0:5760; SITL here dials out to it (a laptop
firewall usually drops inbound) and a recorder on SITL's spare SERIAL1 logs
SIMSTATE -- where the simulator says the rover is -- and the armed span.
tools/pil.sh drives both halves.

    python3 -m rupsha_sim.pil serve --pi 192.168.0.102 --out truth.jsonl
    python3 -m rupsha_sim.pil score truth.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import signal
import sys
import time

from .sitl import Sitl


def truth_row(msg):
    """A SIMSTATE or board HEARTBEAT as a truth-log row; None for anything else."""
    if msg.get_type() == 'SIMSTATE':
        return {'t': time.time(), 'lat': msg.lat * 1e-7, 'lng': msg.lng * 1e-7,
                'yaw': math.degrees(msg.yaw) % 360}
    if msg.get_type() == 'HEARTBEAT':
        return {'t': time.time(), 'armed': bool(msg.base_mode & 128)}
    return None


def serve(pi: str, out: pathlib.Path, log_dir: pathlib.Path, stop_file: pathlib.Path) -> int:
    from rupsha_control.fc.ardurover import ArduRover
    stop_file.unlink(missing_ok=True)
    running = [True]
    signal.signal(signal.SIGTERM, lambda *_: running.__setitem__(0, False))
    with Sitl(model='rover-skid', log_dir=log_dir, serial0=f'tcpclient:{pi}:5760') as sitl, \
            open(out, 'w') as f:
        print(f'SITL dialling {pi}:5760', flush=True)
        # sysid 250, not the companion's 245: this tap must not feed the GCS failsafe
        fc = ArduRover(f'tcp:127.0.0.1:{sitl.port + 2}', sysid=250)

        def record(msg):
            row = truth_row(msg)
            if row is not None:
                f.write(json.dumps(row) + '\n')
                f.flush()
        fc.wait_board(timeout=120)
        fc.add_listener(record)
        fc.start()
        fc._set_interval('SIMSTATE', 5)
        print('truth recorder up', flush=True)
        try:
            while running[0] and not stop_file.exists():
                time.sleep(0.5)
        finally:
            fc.close()
    return 0


def score(path: pathlib.Path) -> dict:
    """Truth over the armed span: closure (end vs start) and farthest excursion."""
    rows = [json.loads(line) for line in open(path)]
    armed = [r['t'] for r in rows if r.get('armed')]
    if not armed:
        return {'armed_s': 0.0}
    t0, t1 = armed[0], armed[-1]
    pts = [r for r in rows if 'lat' in r and t0 - 0.5 <= r['t'] <= t1 + 0.5]
    s = pts[0]
    k = math.cos(math.radians(s['lat']))

    def dist(p):
        return math.hypot((p['lat'] - s['lat']) * 111320.0, (p['lng'] - s['lng']) * 111320.0 * k)
    return {'armed_s': round(t1 - t0, 1), 'samples': len(pts),
            'closure_m': round(dist(pts[-1]), 2), 'farthest_m': round(max(map(dist, pts)), 2)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('serve')
    s.add_argument('--pi', default='192.168.0.102')
    s.add_argument('--out', type=pathlib.Path, default=pathlib.Path('pil_truth.jsonl'))
    s.add_argument('--log-dir', type=pathlib.Path, default=pathlib.Path('/tmp/rupsha_pil'))
    s.add_argument('--stop-file', type=pathlib.Path, default=pathlib.Path('/tmp/rupsha_pil.stop'))
    c = sub.add_parser('score')
    c.add_argument('truth', type=pathlib.Path)
    args = ap.parse_args(argv)
    if args.cmd == 'serve':
        return serve(args.pi, args.out, args.log_dir, args.stop_file)
    print(json.dumps(score(args.truth)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
