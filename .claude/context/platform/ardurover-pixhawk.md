# ArduRover on the Pixhawk 2.4.8 — the board, its record, its traps

The board is the rover's spinal cord: rate and speed loops, skid mixing, S-curves, EKF3 and every
failsafe. This page is what we know about *this* board, how it got to its current state, and the
traps the stack is built around.

## 1. Identity

| | Value | Source |
|---|---|---|
| USB | `1209:5741 Generic Pixhawk1`, `/dev/serial/by-id/usb-ArduPilot_Pixhawk1_2D0052000451333239393630-if00` | `lsusb`, udev |
| Silicon | STM32F427 **rev 3** (no 1 MB flaw) → `Pixhawk1` target, 2 MB flash usable | `uploader.py --identify` |
| Board version | `589824` = `0x90000`, fmu-v3 | `AUTOPILOT_VERSION` |
| Firmware | ArduRover **4.7.2** official (`flight_sw_version 0x040702FF`), git `fb773567` | [`records/ardurover_4.7.2_identity.json`](records/ardurover_4.7.2_identity.json) |
| Heartbeat | `type 10` (ground rover), `autopilot 3` (ArduPilot) | same |
| Parameters | 858 | same |
| Sensors | one IMU (MPU6000, id 1246218); one reliable compass (IST8310 on I2C, id 658945); baro | `SYS_STATUS`, `INS_*_ID`, `COMPASS_DEV_ID*` |
| Second compass | internal LSM303D (SPI, id 131594) enumerates **intermittently**: never in 613 ArduSub boots, once on Rover's first boot, not since | the records' `COMPASS_DEV_ID2` |

**Before Rupsha** the board ran ArduSub 4.5.1 with Mongla's parameters. That state is preserved:
[`records/ardusub_4.5.1_identity.json`](records/ardusub_4.5.1_identity.json) and
[`records/ardusub_4.5.1_params.parm`](records/ardusub_4.5.1_params.parm). The factory Rover
parameters right after flashing: [`records/ardurover_4.7.2_factory.parm`](records/ardurover_4.7.2_factory.parm).

## 2. How it was flashed (2026-10-21)

1. Backed up ArduSub's parameters and `AUTOPILOT_VERSION`.
2. `uploader.py --identify` — rev 3 silicon, so `Pixhawk1` rather than `Pixhawk1-1M`.
3. Flashed `Rover/stable/Pixhawk1/ardurover.apj` (4.7.2). Bootloader left alone.
4. Factory reset, then loaded [`rupsha_baseline.parm`](../../../src/rupsha_control/config/ardupilot/rupsha_baseline.parm).
5. Restored the accelerometer calibration from the ArduSub backup — the same physical IMU
   (`INS_ACC_ID` 1246218), so the offsets carry over.
6. `param_audit` read every baseline parameter back: empty diff.

To redo it: `ros2 run rupsha_control param_audit --apply`, reboot, then `param_audit` alone —
exit 0 is an empty diff.

## 3. The baseline, line by line

| Parameter | Value | Why |
|---|---|---|
| `MAV_GCS_SYSID` | 245 | the Pi is the GCS the board watches |
| `FS_GCS_ENABLE` / `FS_GCS_TIMEOUT` | 1 / 2 s | a silent Pi is an emergency |
| `FS_ACTION` / `FS_TIMEOUT` | 2 (Hold) / 1.5 s | failsafes hold, never wander |
| `FS_THR_ENABLE` | 0 | no RC receiver on the desk; revisit when the radio is fitted |
| `FS_EKF_ACTION` / `FS_CRASH_CHECK` | 1 / 1 | a lost estimate or a crash holds |
| `SERVO1_FUNCTION` / `SERVO3_FUNCTION` | 73 / 74 | skid steering: ThrottleLeft / ThrottleRight |
| `GUID_OPTIONS` | 64 | S-curves for GUIDED position targets |
| `WP_SPEED` / `WP_RADIUS` | 1.0 m/s / 1.0 m | conservative first values; arrival radius the facade widens to |
| `BRD_RTC_TYPES` | 3 | accept time from GNSS and from the companion |
| `MAV1_*` stream rates | 10 / 10 / 10 / 2 / 2 / 5 Hz | the manager also sets intervals by `MAV_CMD_SET_MESSAGE_INTERVAL` so a factory reset cannot starve it |

`rupsha_extnav.parm` is the opt-in overlay for GNSS-denied legs: source set 2 with
`EK3_SRC2_POSXY=6` and the matching `VISO_TYPE`.

## 4. Traps

**`type_mask` groups.** ArduRover tests `mask & GROUP`; *any* bit set ignores the whole group.
The widely quoted velocity + yaw-rate mask `1511` sets `VZ_IGNORE` and the board drops the
velocity entirely. Rupsha's masks: `1479` velocity + yaw rate, `3576` position, `1535` yaw rate
only, `2559` heading only. The acceleration group must always be fully set. Pinned by
`test_board_wire.py`.

**The board's `WP_RADIUS` wins.** GUIDED declares arrival at `WP_RADIUS`, whatever the host
asked. A 3 m `drive` with `WP_RADIUS` 2 stopped 2 m short and reported success. The facade reads
`WP_RADIUS` at startup and never asks for a tighter arrival than `WP_RADIUS + 0.25 m`.

**The cruise pair.** `CRUISE_SPEED` and `CRUISE_THROTTLE` together are the speed
feed-forward; setting one alone made SITL overshoot 1.0 m/s to 1.43 m/s with a 4 s coast, and
Gazebo crawl at 0.55 m/s. Change them together, from a measurement. They are deliberately **not**
in the hardware baseline: there is no chassis to measure.

**The GCS failsafe keys on one sysid.** Only HEARTBEAT / MANUAL_CONTROL / RC override from
`MAV_GCS_SYSID` reset the timer. A base-station GCS on 245 would mask a dead Pi.

**Velocity streams expire after 3 s** (`Rover/mode_guided.cpp`). That is the host-loss reflex for
streamed verbs; position targets keep going until `FS_GCS` holds the rover.

**4.7 renames.** `SRx_` → `MAVn_`, `SYSID_THISMAV` → `MAV_SYSID`, `SYSID_MYGCS` →
`MAV_GCS_SYSID`, `ARMING_CHECK` → `ARMING_SKIPCHK`. Old parm files load silently wrong.

**No Lua, no DDS.** The Pixhawk1 has a 43 KB scripting heap and no AP_DDS in stable F4 builds.

## 5. What the bench proves (`pytest -m board`)

Six tests against the real board, nothing attached to the outputs:

| Test | What it proves |
|---|---|
| `test_firmware_is_the_pinned_ardurover` | 4.7.2 or newer, ground-rover heartbeat |
| `test_baseline_is_loaded` | every baseline parameter reads back equal |
| `test_telemetry_arrives_at_the_baseline_rates` | `ATTITUDE` ≥ 8 Hz, `SYS_STATUS` ≥ 1.5 Hz |
| `test_disarmed_outputs_are_neutral` | `SERVO_OUTPUT_RAW` at trim while disarmed |
| `test_mode_round_trip_while_disarmed` | HOLD ↔ MANUAL, confirmed by heartbeat |
| `test_an_arm_refusal_carries_the_boards_reason` | "Arm: Compass not calibrated" reaches the caller |

Rates measured during the bench run are in [measured-bars](../measured-bars.md).

## 6. Blocked

**Compass calibration.** The board refuses to arm until someone rotates it through the onboard
compass calibration (`MAV_CMD_DO_START_MAG_CAL`, or Mission Planner's Onboard Mag Calibration).
It is a two-minute job for a person and impossible for software. Until then, every arming test
runs in SITL.

**Not attempted, deliberately:** arming with anything on the outputs.
