# Rupsha — the rover autonomy stack

Continuing someone else's session: [`.claude/context/HANDOFF.md`](.claude/context/HANDOFF.md) first.

> **Project**: Rupsha — a ROS 2 Jazzy control / mission / perception stack for an
> autonomous ground rover, built on the fundamentals of [Mongla](https://github.com/fh1m/mongla_ws).
> **This codebase**: `rupsha_ws`. **Platform today**: a **Pixhawk 2.4.8** (`Pixhawk1` target)
> running **ArduRover 4.7.2** + a **Raspberry Pi 4** companion. The chassis is not built yet:
> everything that moves, moves in simulation or on a desk.

**Read first, in this order:**

| Document | What it gives you |
|---|---|
| [`docs/the-shift.md`](docs/the-shift.md) | the architecture from first principles: why reflexes live on the Pixhawk and thinking on the Pi |
| [`docs/capability-map.md`](docs/capability-map.md) | every capability with its evidence and an honest state |
| [`.claude/context/ROADMAP.md`](.claude/context/ROADMAP.md) | **the one status file** — where we are, what is left, what is blocked |
| [`.claude/context/BUGS.md`](.claude/context/BUGS.md) | **the single defect register** |
| [`.claude/context/measured-bars.md`](.claude/context/measured-bars.md) | every number the stack ships, and what measured it |
| [`.claude/context/README.md`](.claude/context/README.md) | **the book** — every context doc, in reading order |

**Precedence:** if anything here contradicts `src/`, the code wins. If a doc contradicts a
measurement, the measurement wins — and both belong in `measured-bars.md`.

**Four states, never blurred.** Say which one you mean:
**FIELD** (on the real rover, outdoors) · **BENCH** (real hardware on a desk, or real code in a
closed loop with SITL/Gazebo) · **BUILT** (code and tests exist, never closed a loop) ·
**BLOCKED** (on hardware, a person, or a decision). **Nothing is FIELD yet.**

---

## 1. The vehicle

| Part | What |
|---|---|
| Flight controller | **Pixhawk 2.4.8** clone, STM32F427 rev 3 (no 1 MB flaw), `Pixhawk1` target, one IMU (MPU6000), one reliable compass (IST8310; the internal LSM303D enumerates intermittently) |
| Firmware | **ArduRover 4.7.2** (git `fb773567`), flashed 2026-10-21 over ArduSub 4.5.1 (backup in [`platform/records/`](.claude/context/platform/records/)) |
| Companion | **Raspberry Pi 4B** 8 GB, Ubuntu 24.04, container `rupsha_pi:jazzy` |
| Link | USB today, MAVLink 2 at 115200; companion identity **sysid 245 / compid 191** |
| Drive | frame-agnostic **skid steer**: `SERVO1_FUNCTION=73` (ThrottleLeft), `SERVO3_FUNCTION=74` (ThrottleRight) |
| Sim stand-in | `rupsha_skid`, a 6WD skid model generated from one table (`rupsha_sim/model_gen.py`) — *not* Rupsha's geometry |
| Cameras / GNSS | one front camera planned (USB UVC or Pi CSI); GNSS is the board's receiver; RTK later |

Hardware detail and the bring-up record: [`platform/ardurover-pixhawk.md`](.claude/context/platform/ardurover-pixhawk.md).

---

## 2. Working with the board — the traps that already cost us runs

⛔ **Rover's `type_mask` ignores a whole group if any one bit is set.** ArduRover tests
`mask & GROUP`, so the folk value `1511` (velocity + yaw rate "with z ignored") **drops the
velocity**. Use `1479` for velocity + yaw rate, `3576` position, `1535` yaw-rate only, `2559`
heading only. They live in `fc/ardurover.py` and `test_board_wire.py` pins them.

⛔ **GUIDED stops at the board's own `WP_RADIUS`, not ours.** A `drive 3 m` with `WP_RADIUS=2`
stopped 2 m short and reported success. The facade reads `WP_RADIUS` at startup and widens every
arrival radius to `WP_RADIUS + 0.25 m`.

⛔ **`CRUISE_SPEED` and `CRUISE_THROTTLE` are one measured pair** — the speed feed-forward.
Changing one alone overshot a 1.0 m/s command to 1.43 m/s with a 4 s coast in SITL, and in
Gazebo crawled at 0.55 m/s. Set `WP_SPEED` for speed; set the pair only from a measurement.

⛔ **The GCS failsafe counts only the GCS sysid's HEARTBEAT / MANUAL_CONTROL / RC override.**
`MAV_GCS_SYSID=245` is the Pi. A base-station GCS must **not** use 245, or it masks a dead Pi.
With the Pi silent the board holds at `FS_GCS_TIMEOUT` + `FS_TIMEOUT` = 3.5 s (seen in SITL at
3.6 s, through the 1 Hz HEARTBEAT). It latches **even disarmed**: a tool that speaks as 245 and
exits leaves the board in HOLD, flashing yellow. Bench tools use `Companion.TOOLS_SYSID` 246.

⛔ **After a parameter wipe, persist `INS_ACC_ID` / `INS_GYR_ID`.** RAM shows the detected ID,
EEPROM holds 0, and pre-arm says "3D Accel calibration needed" with the offsets intact.

⛔ **ExtNav as EKF source without `VISO_TYPE` fails pre-arm.** `rupsha_extnav.parm` sets both;
`bringup_check` names the mismatch.

⛔ **4.7 renamed parameters**: `SRx_` → `MAVn_`, `SYSID_MYGCS` → `MAV_GCS_SYSID`, `ARMING_CHECK` →
`ARMING_SKIPCHK`. A parm file from a forum post is probably the old names.

⛔ **The board refuses to arm: "Compass not calibrated".** BLOCKED on a person rotating the
board: `ros2 run rupsha_control compass_cal --reboot`. Accel calibration was restored from the
ArduSub backup (same IMU id 1246218). The refusal reaches the caller verbatim — that path is tested.

---

## 3. Architecture — the rules

1. **Exactly one node touches the board**: `rupsha_control`'s manager, through
   `fc/ardurover.py`, the only module that imports pymavlink (a repo guard enforces it).
2. **One action, verbs.** `/rupsha/move` (manager) and `/rupsha/autonomy` (executive) share
   the `Move` type. `stop`, `disarm`, `abort`, `disable_autonomy` bypass the busy gate.
3. **One manifest.** `manifest/rupsha.yaml` is the single source for verbs, fields, enums,
   state, topics. `tools/gen_manifest.py` emits the messages, `manifest.py` and the command
   reference; `--check` fails CI when stale. Code uses `Topics.X`, never a literal.
4. **NaN means absent.** No default ever masquerades as a reading. ABSENT is not PASS.
5. **Reflexes on the board, thinking on the Pi.** The body being built puts the speed loop,
   the steer and the stop on the ESP32. The Pi sends speed, steer and a heartbeat. The
   Pixhawk and ArduRover are the previous body. The SITL launch is deleted.
6. **Neutral on startup; heartbeat on its own thread; cooperative abort** checked every tick.
7. **Truth tests, not agreement tests.** Sim tests check Gazebo's pose or SITL's SIMSTATE,
   never the EKF the rover steered by. A test that cannot fail is deleted.

---

## 4. Running it

```bash
./build_rupsha.sh                         # colcon via the venv python; skips the gz plugin on aarch64
source install/setup.bash
pytest                                    # unit + guards; sitl/gazebo/board markers skip when absent
pytest -m sitl src/rupsha_sim             # ArduRover SITL, built-in skid physics
pytest -m gazebo src/rupsha_sim           # Ackermann plant, plus the previous skid lockstep
pytest -m board src/rupsha_control        # the previous Pixhawk on USB

ros2 launch rupsha_sim sim.launch.py headless:=true    # Ackermann plant and board_sim
ros2 launch rupsha_planner rupsha.launch.py            # the real rover, when it exists
ros2 run rupsha_planner rupsha move_forward --duration 2 --speed 0.5
ros2 run rupsha_planner mission square_drive [--dry-run]
ros2 run rupsha_control bringup_check --bench          # before the manager: is the board ready?
tools/pil.sh                                           # Pi-in-the-loop against SITL here
```

Environment: distrobox `rupsha_dev` (ROS 2 Jazzy, `ROS_DOMAIN_ID=43`, CycloneDDS), venv
`/opt/rupsha-venv` (pymavlink lives there, not in `/usr/bin/python3`). ArduPilot at
`~/tmp/ardupilot` (tag `Rover-4.7.2`), found beside the workspace or via `ARDUPILOT_HOME`.
Traps: [`platform/box-and-pi.md`](.claude/context/platform/box-and-pi.md).

---

## 5. Simulation and the hardware chain

- **Ackermann plant** (the default): Gazebo Harmonic, `gz_ros2_control`, `board_sim`. No ArduRover.
- **SITL** (previous body, built-in skid physics): fastest truth for the old control logic.
- **Gazebo Harmonic + SITL JSON lockstep** (previous body): camera, IMU and the 6WD model; `ArduPilotPlugin`
  forwards gyro/accel **unrotated**, so the IMU sensor is mounted FRD (rolled 180°). Left z-up,
  yaw rate arrives negated and the rover spins at full output while its attitude looks right.
- **Board-in-the-loop**: the real Pixhawk on the desk (`pytest -m board`).
- **Pi-in-the-loop**: the real stack on the Pi, SITL on the laptop dialling *out* to it.
- ArduPilot has no classic HIL any more. **Sim-on-Hardware** fits the Pixhawk1 and boots, but its
  simulated IMUs never report healthy, so it will not arm — recorded, not pursued.

Detail: [`platform/sim-and-hitl.md`](.claude/context/platform/sim-and-hitl.md).

---

## 6. How we work

- **A number ships with its measurement** in `measured-bars.md`, or it is labelled BUILT.
- **One defect register**: `BUGS.md`. Fixed bugs keep their entry and the test that pins them.
- **Commits** are authored by Fahim Faisal only — no co-author trailers.
- **Do not edit generated files** (`Move.action`, `RupshaState.msg`, `AutonomyStatus.msg`,
  `manifest.py`, `reference/commands.md`, everything under `rupsha_sim/{models,worlds,scenarios}`
  and `config/bridge.yaml`, `config/ackermann_bridge.yaml`, `config/ackermann_controllers.yaml`).
  Edit the manifest or the generator and regenerate.
