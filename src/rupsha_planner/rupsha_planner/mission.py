#!/usr/bin/env python3
"""Run a mission file: `def run(rupsha, log)`.

    ros2 run rupsha_planner mission square_drive            # missions/square_drive.py
    ros2 run rupsha_planner mission ./my_mission.py
    ros2 run rupsha_planner mission square_drive --dry-run  # FakeRover, simulated time

--dry-run executes the same file against the facade on a kinematic fake, in
simulated time, with no ROS. It is how a mission is proved to parse, branch and
terminate before it is allowed near a vehicle.
"""

from __future__ import annotations

import argparse
import importlib.util
import logging
import pathlib
import sys

MISSIONS = pathlib.Path(__file__).resolve().parent / 'missions'


def load(name: str):
    path = pathlib.Path(name)
    if not path.is_file():
        path = MISSIONS / f'{name}.py'
    if not path.is_file():
        avail = sorted(p.stem for p in MISSIONS.glob('*.py') if not p.stem.startswith('_'))
        raise SystemExit(f'no mission {name!r}; available: {", ".join(avail)}')
    spec = importlib.util.spec_from_file_location(f'rupsha_mission_{path.stem}', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not callable(getattr(mod, 'run', None)):
        raise SystemExit(f'{path} has no run(rupsha, log)')
    return mod


def dry_port():
    from rupsha_control.fc.fake import FakeRover
    from rupsha_control.rupsha import Rupsha

    from .ports import FacadePort
    fc = FakeRover()
    return FacadePort(Rupsha(fc, clock=fc.clock, sleep=fc.sleep)), fc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mission')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args(argv)
    mod = load(args.mission)
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    log = logging.getLogger(f'mission.{args.mission}')

    from rupsha_control.manifest import VERBS

    from .dsl import RupshaMission
    if args.dry_run:
        port, fc = dry_port()
        manager_verbs = {v for v, s in VERBS.items() if s['runs_on'] != 'executive'}
        rupsha = RupshaMission(port, log, supported=manager_verbs)
    else:
        from .client import make_port
        port = make_port('rupsha_mission')
        rupsha = RupshaMission(port, log)
    try:
        mod.run(rupsha, log)
    except KeyboardInterrupt:
        log.warning('interrupted -- stopping')
        port.run('stop')
        return 130
    finally:
        if args.dry_run:
            log.info(f'[dry-run] ended at north {fc.north:.1f} m east {fc.east:.1f} m, '
                     f'{fc.clock.t:.0f} s simulated')
        else:
            port.close()
    failed = [v for v, _, r in rupsha.results if not r.success]
    log.info(f'{len(rupsha.results)} verbs, {len(failed)} not successful {failed or ""}')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
