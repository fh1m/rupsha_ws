---
layout: default
title: Capability map
description: "Every capability of the Rupsha stack, the evidence for it, and whether it is field, bench, built, or blocked."
---

# Capability map

Every capability, the evidence for it, and one of four states: **FIELD** (the real rover,
outdoors) · **BENCH** (real hardware on a desk, or the real code closing a loop against
SITL/Gazebo — the column says which) · **BUILT** (code and tests, never closed a loop) ·
**BLOCKED** (and what unblocks it). Nothing is FIELD. Numbers come from
[`measured-bars.md`](https://github.com/fh1m/rupsha_ws/blob/main/.claude/context/measured-bars.md).

## 1. The board

| Capability | State | Evidence |
|---|---|---|
| ArduRover 4.7.2 on the Pixhawk 2.4.8 (`Pixhawk1`, STM32F427 rev 3) | BENCH (board) | `test_board_bench.py::test_firmware_is_the_pinned_ardurover` |
| Baseline parameters loaded and audited | BENCH (board) | `param_audit`; `test_baseline_is_loaded` |
| Telemetry at the baseline rates (position 10 Hz, IMU 20 Hz) | BENCH (board) | `test_telemetry_arrives_at_the_baseline_rates` |
| Neutral outputs while disarmed | BENCH (board) | `test_disarmed_outputs_are_neutral` |
| Mode changes confirmed by heartbeat | BENCH (board) | `test_mode_round_trip_while_disarmed` |
| An arming refusal carries the board's reason | BENCH (board) | `test_an_arm_refusal_carries_the_boards_reason` |
| Arming the real board | **BLOCKED** | compass calibration needs a person to rotate the board: `compass_cal` (start/cancel checked on the board; a fit needs the person) |

## 2. Control (verbs on `/rupsha/move`)

| Capability | State | Evidence |
|---|---|---|
| `arm`, `disarm`, `set_mode`, `hold`, `stop`, `abort` | BENCH (SITL) | `test_sitl_scenarios.py`, `test_rupsha_verbs.py` |
| `goto_gnss` arrival, checked against simulator truth | BENCH (SITL) | `test_sitl_scenarios.py` (SIMSTATE) |
| `move_forward` / `move_back` / `pivot` / `arc` velocity streams, 3 s expiry | BENCH (SITL, Gazebo) | `test_sitl_scenarios.py`, `test_gazebo_lockstep.py` |
| `turn_to` / `turn` without overshoot | BENCH (Gazebo) | `test_turn_to_settles_without_overshoot` |
| `drive` arriving inside the board's `WP_RADIUS` | BENCH (SITL) | `board_wp_radius` widening, `test_rupsha_verbs.py` |
| `search_spiral`, `vision_approach` | BUILT | fake-board tests; no camera in a closed loop yet |
| `set_ekf_source`, `set_origin`, `led`, `wait` | BUILT | `test_rupsha_verbs.py` |
| Companion-loss failsafe → HOLD | BENCH (SITL) | `test_sitl_scenarios.py`: board acts at 3.5 s, seen at 3.6 s, bound 4.5 s |

## 3. Autonomy (verbs on `/rupsha/autonomy`)

| Capability | State | Evidence |
|---|---|---|
| Leg queue: position, marker, object legs | BUILT | `test_executive.py` |
| Stuck detection and the recovery ladder | BUILT | `test_executive.py` (rock, ladder scenarios) |
| Stuck point learned as an obstacle; detour routing | BUILT | `test_executive.py`, `routing.py` |
| Verify before success (GNSS averaging, frame majority) | BUILT | `test_executive.py` |
| Missions in the DSL (`square_drive`, `urc_autonomy`) | BENCH (PIL) for `square_drive` | `tools/pil.sh`: 10/10 verbs, 0.73 m truth closure |

## 4. Perception and estimation

| Capability | State | Evidence |
|---|---|---|
| ArUco 4x4_50 posts: id, bearing, range with IPPE pose and flip disambiguation | BUILT | `rupsha_vision/test/test_aruco_truth.py` on rendered scenes |
| ArUco on OpenCV 4.6 (the Pi's and CI's apt build) | BUILT | legacy `detectMarkers` path below 4.7; whole suite green in `ros:jazzy-ros-base` with apt 4.6 (B-21) |
| YOLO objects via ONNX Runtime | BUILT | `test_object_detector.py` |
| Shadow right-invariant EKF cross-checking EKF3 | BUILT | `rupsha_localization/test/` |
| ExtNav uplink for GNSS-denied legs | BUILT (opt-in, off) | `rupsha_extnav.parm`, `bringup_check` source check |
| Sensors table → drivers, topics, rate check | BUILT | `rupsha_sensors/test/test_sensors_table.py` |

## 5. Simulation and the hardware chain

<p align="center"><img src="assets/diagrams/range.svg" alt="Detection range of a 20 cm tag: never at 8 m, edge at 6 m, reliable and about 3 percent long at 4.5 m and 3 m." width="100%"></p>

<p align="center"><img src="assets/diagrams/approach.svg" alt="The marker leg finished with the camera 1.69 m from the tag, looking 17 degrees up." width="100%"></p>

| Capability | State | Evidence |
|---|---|---|
| ArduRover SITL with the baseline | BENCH | `test_sitl_scenarios.py` |
| Gazebo Harmonic + SITL JSON lockstep, 6WD model | BENCH | `test_gazebo_lockstep.py` (3 tests, truth = Gazebo pose) |
| Whole stack in Gazebo (`sim.launch.py`: manager, vision, localization, executive) on a marker leg: GNSS, search, approach, verify | BENCH (Gazebo) | `test_gazebo_marker_leg.py`, one headless run: `all 1 legs complete` in 30 s, camera 1.69 m from the tag face, body 2.09 m from the post ([B-32](https://github.com/fh1m/rupsha_ws/blob/main/.claude/context/BUGS.md)) |
| ArUco range against Gazebo truth, rover driving itself to stations | BENCH | `test_gazebo_aruco.py`: ~3 % long; reliable to 4.5 m, edge at 6 m for 20 cm tags ([measured-bars](https://github.com/fh1m/rupsha_ws/blob/main/.claude/context/measured-bars.md)) |
| Seeded worlds (`flat_ci`, `playpen`) with ArUco posts | BUILT | `test_sim_contract.py` (fresh, deterministic, tags decode) |
| Board-in-the-loop | BENCH | `pytest -m board` |
| Pi-in-the-loop | BENCH | `tools/pil.sh` |
| Sim-on-Hardware (SITL physics on the Pixhawk) | **BLOCKED** | fits (757 KB flash free), boots, simulated GPS + EKF3 run; simulated IMUs never healthy, will not arm — [record](https://github.com/fh1m/rupsha_ws/blob/main/.claude/context/platform/records/sim_on_hw_2026-10-21.json) |

## 6. Blocked, and what unblocks it

| Item | Unblocked by |
|---|---|
| Arming the real board | compass calibration (a person rotating the board) |
| Anything FIELD | the chassis, motors and drivers |
| Vision rates on the Pi | the front camera on the Pi (ArUco and YOLO loop meters) |
| RTK heading and truth | two u-blox F9P receivers (moving baseline) |
