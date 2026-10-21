#!/usr/bin/env python3
"""rupsha CLI -- every verb in manifest/rupsha.yaml, flags from its fields.

    ros2 run rupsha_planner rupsha arm
    ros2 run rupsha_planner rupsha goto_gnss --lat -35.3631 --lon 149.1653
    ros2 run rupsha_planner rupsha move_forward --duration 3 --speed 0.5
    ros2 run rupsha_planner rupsha add_marker_leg --lat ... --lon ... --tag_id 4
    ros2 run rupsha_planner rupsha start_autonomy
    ros2 run rupsha_planner rupsha stop

Adding a verb to the manifest adds it here on the next build.
"""

from __future__ import annotations

import argparse
import sys

from rupsha_control.manifest import FIELDS, VERBS

_PY = {'float32': float, 'float64': float, 'int8': int, 'int32': int, 'uint8': int,
       'string': str, 'bool': bool}


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog='rupsha', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='verb', required=True)
    for verb, spec in VERBS.items():
        p = sub.add_parser(verb, help=spec['help'], description=spec['help'])
        for f in spec['fields']:
            ros_type, unit = FIELDS[f]
            default = spec['defaults'].get(f)
            hint = f'[{unit}] ' if unit else ''
            p.add_argument(f'--{f}', type=_PY[ros_type], default=default,
                           required=default is None and f not in ('target_name',),
                           help=f'{hint}default {default}' if default is not None else hint)
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    verb = args.verb
    kw = {f: getattr(args, f) for f in VERBS[verb]['fields'] if getattr(args, f) is not None}
    from .client import make_port
    port = make_port('rupsha_cli')
    try:
        res = port.run(verb, **kw)
    except KeyboardInterrupt:
        print(f'{verb}: interrupted -- goal cancelled')
        return 130
    finally:
        port.close()
    print(f'{verb}: {res.outcome.name} -- {res.message}')
    return 0 if res.success else 1


if __name__ == '__main__':
    sys.exit(main())
