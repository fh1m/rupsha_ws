# Measured bars — every number the stack ships, and what measured it

A number appears in code or docs only if it is in this ledger with its source, or it is labelled
an assumption. Each row says **where** it was measured (board / SITL / Gazebo / Pi) and **how**.
When a field run disagrees, the field run wins and this page records both.

Sections 1 to 5 are the **previous body**: a six-wheel skid, a Pixhawk 2.4.8, and a Pi 4.
They are not claimed for the Ackermann chassis.

## The chassis being built

Ackermann, Gazebo Harmonic, no ArduRover. One headless pass of `test_gazebo_ackermann.py`
on 2026-10-23. The test's bars, not a taped distance:

| What the test required | Bar |
|---|---|
| Straight, 400 mm/s, about 2 s | more than 0.4 m east on Gazebo's pose |
| Encoder | ticks within 35 % of that distance times 1873 per metre |
| Steer 0.35 rad | compass heading moved more than 8° |
| Camera | 640×480, not a flat frame |
| GPS | a fix, latitude near the world home |
| Heartbeat stopped | `link_fresh` became false |

200 Hz is the aim of the inner loop. A scope has not seen it. The turn radius is unmeasured.
The sim battery, 11.1 V, is the nominal pack, not a cell.

## 1. The board on the desk (Pixhawk 2.4.8, ArduRover 4.7.2, USB)

Measured 2026-10-21 by `test_board_bench.py` and the manager's stream counters, nothing on the outputs.

| Quantity | Value | Asked for | Notes |
|---|---|---|---|
| `GLOBAL_POSITION_INT` | 10 Hz | 10 | |
| `RAW_IMU` | 20.1 Hz | 20 | |
| `SERVO_OUTPUT_RAW` | 5 Hz | 5 | |
| `SYS_STATUS` | 2 Hz | 2 | bar ≥ 1.5 Hz |
| `HEARTBEAT` | 1 Hz | — | link declared dead after 2 s |
| Disarmed outputs | at trim | neutral | `test_disarmed_outputs_are_neutral` |
| Arming | refused: "Arm: Compass not calibrated" | — | reason reaches the caller verbatim; BLOCKED on calibration |
| `param_audit` diff after load | empty | empty | |

## 2. SITL (`rover-skid` physics, baseline parameters)

| Quantity | Value | Bar | Test |
|---|---|---|---|
| Companion silent → HOLD seen | 3.6 s | ≤ 4.5 s: the board acts at `FS_GCS_TIMEOUT` 2 s + `FS_TIMEOUT` 1.5 s (`Rover.cpp`, `failsafe.cpp`), seen in the next 1 Hz HEARTBEAT | `test_companion_heartbeat_loss_holds_the_rover` |
| `goto_gnss` arrival, by SIMSTATE | inside `WP_RADIUS + 0.25 m` | same | `test_goto_gnss_arrives_by_truth` |
| Velocity stream expiry | stops after the last command | 3 s (`Rover/mode_guided.cpp`) | `test_velocity_stream_drives_and_expires` |
| `CRUISE_SPEED` changed alone (the B-17 run) | 1.0 m/s asked → 1.43 m/s, 4 s coast | — | why the pair moves together |

## 3. Gazebo Harmonic + SITL lockstep (`rupsha_skid`, `flat_ci`)

Truth is Gazebo's model pose over gz-transport. Four tests sharing one server, 55 s wall time.

| Quantity | Value | Bar | Test |
|---|---|---|---|
| `move_forward` speed | 1.00 m/s | ±0.15 m/s | `test_forward_holds_heading_and_speed` |
| Heading drift while driving | < 1° | < 3° | same |
| `turn_to 90` time | 3.3 s | < 8 s | `test_turn_to_settles_without_overshoot` |
| `turn_to 90` error | 0.5°, no overshoot | < 5° final, < 10° overshoot | same |
| Board yaw vs Gazebo yaw | agree | < 5° | `test_board_yaw_agrees_with_gazebo` |
| Skid scrub factor (actual / ideal turn rate) | 0.85 | — | measured on the model; feeds `ATC_STR_RAT_FF 0.16` |
| Cruise pair | 1.0 m/s at 50 % | — | multiplier 40 × 0.10 m radius ⇒ ±2 m/s at ±1 output |
| With the cruise pair from `rover-skid.parm` (B-11) | 0.55 m/s | — | why Gazebo has its own pair |

**ArUco range against truth** (`test_gazebo_aruco.py`). The rover drives itself to stations
facing tag 3 in `flat_ci`. The camera is 640x480 with a 70° horizontal FOV, the tag is 20 cm, and
the median is taken over 10 frames. Truth runs from the camera to the centre of the cube face. The
bar, |error| ≤ 5 % + 5 cm, was set before the first run.

| Station | Tag side | Seen | Error (four runs) |
|---|---|---|---|
| 3 m | 29–31 px | 100 % | +0.08 to +0.11 m (+2.5 to +3.5 %) |
| 4.5 m | 20 px | 100 % | +0.17 to +0.18 m (+3.7 to +3.8 %) |
| 6 m | 16 px | 100 % in one run, 0 % in the next | −0.33 m when seen |
| 8 m | 11 px | 0 % | — |

- Range reads about 3 % long and has not been explained yet. The candidates are sub-pixel shrink
  of the corners on a rendered edge, and a small error in the camera's height.
- Detection stops at the detector's 16 px minimum side. At this resolution that is about 6 m for
  a 20 cm tag, so a URC-sized approach needs more pixels: a higher resolution, a narrower lens,
  or a bigger tag.
- The test asserts only at 3 m and 4.5 m. The 6 m and 8 m rows are the detection limit.
- Re-measured 2026-10-23 after the camera was pitched 17° up (it had been 8° down, and the tag
  left the top of the frame at the standoff). One run, same stations: 4.5 m truth 4.710 m,
  ArUco 4.878 m (+0.168 m, +3.6 %, 19.2 px, seen in 100 %); 3.0 m truth 3.056 m, ArUco 3.148 m
  (+0.093 m, +3.0 %, 29.3 px, seen in 100 %). Still about 3 % long.

**One marker leg, the whole stack** (`test_gazebo_marker_leg.py`, headless, one run,
2026-10-23). GNSS to a point 6 m short of post 3, then search, approach and verify.

| Quantity | Value | Bar |
|---|---|---|
| Executive detail | `all 1 legs complete` | the string the test requires |
| Wall time of the leg | 30 s | budget 420 s |
| Camera to the tag face, at the end | 1.69 m | ≤ standoff 1.5 + slack 1.0 = 2.5 m |
| Closest the body came to the post centre | 2.09 m | ≥ 0.8 m |

## 4. Pi-in-the-loop (manager + mission on the Pi 4, SITL on the laptop)

`tools/pil.sh`, mission `square_drive` (four 5 s sides at 1 m/s, 90° turns), two runs.

| Quantity | Value | Notes |
|---|---|---|
| Verbs succeeded | 10 / 10 | |
| Turn error | ≤ 1.2° | |
| Closure by truth (SIMSTATE) | 0.72–0.73 m | the square does not close exactly: arrival radius and S-curves |
| Closure by EKF | 0.95–0.99 m | the estimate the rover steered by; reported, not scored |
| Farthest point from start | 7.33 m | |
| Manager `/rupsha/state` rate | 9.98 Hz | asked 10 |
| IMU relay rate | 19.74 Hz | asked 20 |
| Pi container CPU | 11.5–12.4 % | of 4 cores, `docker stats` |
| Pi container memory | 114 MiB | |
| Workspace build on the Pi | 1 min 59 s | `rupsha_pi:jazzy`, without `ardupilot_gazebo` |

## 5. Sim-on-Hardware (SITL physics on the Pixhawk's STM32F427)

| Quantity | Value | Notes |
|---|---|---|
| Flash used / free | 1,323,304 B / 757,440 B | `rover-skid` build, 4.7.2 |
| CPU load at 400 / 200 / 50 Hz loop | 26.7 % / 17.5 % / 8.1 % | `SYS_STATUS.load`, disarmed |
| Arms | no | "PreArm: Gyros not healthy" at every rate |

## 6. Not measured yet — shipped as assumptions, labelled BUILT

| Quantity | Assumed | Measured by |
|---|---|---|
| YOLO rate on the Pi 4 CPU at 320 px | 5–15 Hz (research) | the vision node's loop meter on the Pi |
| ArUco range on the real camera | ~3 % long, ~6 m reach (Gazebo, §3) | the front camera on the Pi against a tape-measured post |
| Stuck-detection thresholds | executive defaults | the first outdoor run |
| `WP_SPEED 1.0`, `WP_RADIUS 1.0` on the real rover | conservative first values | the chassis |
| Real rover cruise pair | not set | the chassis, on flat ground |
