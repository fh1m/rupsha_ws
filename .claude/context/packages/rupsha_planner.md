# rupsha_planner — missions, the DSL, and the autonomy executive

**State: BUILT** (executive on a simulated-time fake board) · **BENCH** for `square_drive`
(Pi-in-the-loop).

## Two ways to drive the rover

**The mission DSL** — plain Python over the verbs. A mission is a file with `run(rupsha, log)`:

```python
def run(rupsha, log):
    rupsha.require('arm', 'goto_gnss', 'disarm')   # refuse before moving if a verb is missing
    rupsha.arm()
    rupsha.goto_gnss(lat=23.81, lon=90.41)
    rupsha.disarm()
```

`ros2 run rupsha_planner mission <name|file> [--dry-run]` runs it; `--dry-run` uses `FakeRover`
in simulated time, no ROS. The exit code is 1 if any verb failed.

**The executive** — MRDT-style legs, served on `/rupsha/autonomy`:
`add_position_leg`, `add_marker_leg`, `add_object_leg`, `add_obstacle`, `clear_legs`,
`clear_obstacles`, `start_autonomy`, `disable_autonomy`. The executive is itself a client of
`/rupsha/move`, so the manager still owns the board.

## The executive

States (`AutonomyState`, from the manifest): IDLE, NAVIGATING, SEARCH_PATTERN,
APPROACHING_MARKER, APPROACHING_OBJECT, VERIFYING_POSITION, VERIFYING_MARKER,
VERIFYING_OBJECT, REVERSING, STUCK. Published on `/rupsha/autonomy_status` with the URC display
state (red autonomy, blue teleop, flashing green arrived).

A marker or object leg runs `goto_gnss` → `search_spiral` → `vision_approach` → verify. A
position leg verifies by averaging GNSS for `verify_position_s`; a target leg needs a hit
fraction of `verify_hit_fraction` over `verify_target_s`. If the post is already in view when a
leg starts, the rover backs off `backoff_m` first.

**Stuck ladder** (`stuck.py`, MRDT's detection): stuck when |speed| < 0.10 m/s and
|yaw rate| < 4°/s for 3 checks 1 s apart, after a 2 s grace. Recovery: reverse 3 s; rotate +65°
and reverse; rotate −65° and reverse; STUCK. Moving 1 m ends the ladder early. **Rupsha's
addition:** the stuck point becomes a learned obstacle, and `routing.py` detours the resumed leg
around it.

All thresholds are **defaults, not measurements** — see
[measured-bars §6](../measured-bars.md#6-not-measured-yet--shipped-as-assumptions-labelled-built).

## Modules

`dsl.py` · `mission.py` · `cli.py` (`rupsha <verb> --field value`) · `client.py` (the ROS port) ·
`ports.py` (one contract, two ports: in-process facade or ROS) · `executive.py` ·
`executive_node.py` · `stuck.py` · `routing.py` · `missions/` (`square_drive`, `urc_autonomy`)

## Launch

`ros2 launch rupsha_planner rupsha.launch.py` — the whole rover: control bringup, sensors,
vision, localization, executive, each switchable by argument.

## Tests

`test_executive.py` (legs, ladder, detours, verification — scenarios on simulated time with the
stall moment chosen) · `test_dsl_and_missions.py`.
