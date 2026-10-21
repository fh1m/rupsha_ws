"""Is every sensor in the table actually arriving, fast enough? PASS / FAIL / ABSENT.

    ros2 run rupsha_sensors sensors_check              # listen 5 s
    ros2 run rupsha_sensors sensors_check --seconds 10

ABSENT (nothing arrived) is reported apart from FAIL (arrived, too slow): the
first is a cable or a driver, the second a bus or a CPU. Exit 0 only when every
enabled row PASSes.
"""

from __future__ import annotations

import argparse
import sys
import time

from .table import load

PASS, FAIL, ABSENT = 'PASS', 'FAIL', 'ABSENT'


def evaluate(sensors, stamps: dict, window_s: float) -> list:
    """Pure: [(verdict, name, detail)] from {topic: [arrival times]}."""
    rows = []
    for s in sensors:
        t = sorted(stamps.get(s.topic, []))
        if len(t) < 2:
            rows.append((ABSENT, s.name, f'{s.topic}: {len(t)} msg in {window_s:.0f} s'))
            continue
        hz = (len(t) - 1) / (t[-1] - t[0]) if t[-1] > t[0] else 0.0
        ok = hz >= s.min_hz
        rows.append((PASS if ok else FAIL, s.name, f'{s.topic}: {hz:.1f} Hz (min {s.min_hz:g})'))
    return rows


def verdict(rows) -> int:
    return 0 if rows and all(v == PASS for v, _, _ in rows) else 1


def _type_of(node, topic: str, wait_s: float):
    from rosidl_runtime_py.utilities import get_message
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        for name, types in node.get_topic_names_and_types():
            if name == topic and types:
                return get_message(types[0])
        time.sleep(0.1)
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--table', default=None)
    args = ap.parse_args(argv)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data

    sensors = load(args.table)
    rclpy.init()
    node = rclpy.create_node('sensors_check')
    stamps = {s.topic: [] for s in sensors}
    for s in sensors:
        msg_type = _type_of(node, s.topic, 2.0)
        if msg_type is not None:
            node.create_subscription(msg_type, s.topic,
                                     lambda _m, t=s.topic: stamps[t].append(time.monotonic()),
                                     qos_profile_sensor_data)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    rows = evaluate(sensors, stamps, args.seconds)
    for v, name, detail in rows:
        print(f'[{v:6}] {name:14} {detail}')
    node.destroy_node()
    rclpy.shutdown()
    return verdict(rows)


if __name__ == '__main__':
    sys.exit(main())
