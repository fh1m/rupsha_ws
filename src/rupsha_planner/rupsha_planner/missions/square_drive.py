"""square_drive -- the shakedown.

Four 5 m legs with 90-degree right pivots between them. Use it to prove, on any
new setup (SITL, Gazebo, the real rover on blocks, then on the ground):
  - move_forward drives forward, not back (SERVO1/3 = 73/74 the right way round);
  - turn +90 turns clockwise seen from above;
  - the rover ends within a couple of metres of where it started.

    ros2 run rupsha_planner mission square_drive [--dry-run]
"""

import math

SIDE_S = 5.0
SPEED = 1.0
TURN_DEG = 90.0


def run(rupsha, log):
    rupsha.require('arm', 'move_forward', 'turn', 'stop', 'disarm')
    if not rupsha.arm().success:
        return
    start = rupsha.here()
    for leg in range(4):
        if not rupsha.move_forward(duration=SIDE_S, speed=SPEED).success:
            break
        if leg < 3:
            rupsha.turn(angle=TURN_DEG)
    rupsha.stop()
    end = rupsha.here()
    if all(map(math.isfinite, start + end)):
        from rupsha_control.geo import distance_m
        log.info(f'closure error {distance_m(*start, *end):.2f} m')
    rupsha.disarm()
