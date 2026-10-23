# ROADMAP — the one status file

Where Rupsha is, what is left, what is blocked. If this disagrees with
[capability-map](../../docs/capability-map.md), fix whichever is wrong in the same change.

**Last updated 2026-10-23.** Nothing is FIELD. The chassis is being built and has not been outdoors.

## Done

| Phase | What | State |
|---|---|---|
| A. Scaffold | seven packages, manifest + codegen, facade, manager, executive, DSL, CLI, repo guards | BUILT, tests green |
| B. Firmware | ArduSub 4.5.1 backed up; ArduRover 4.7.2 flashed; baseline loaded and audited; accel calibration restored | BENCH |
| C. Simulation | SITL scenarios; Gazebo Harmonic + SITL JSON lockstep on a 6WD model; seeded worlds | BENCH, the previous body |
| C. Ackermann plant | Gazebo Harmonic, one driven axle, steer, camera, IMU, GPS, no lidar. `board_sim` speaks the contract. `test_gazebo_ackermann.py` | BENCH (Gazebo) |
| C. ArUco vs truth | the rover drives to stations in Gazebo; range ~3 % long, reach ~6 m for a 20 cm tag ([measured-bars §3](measured-bars.md#3-gazebo-harmonic--sitl-lockstep-rupsha_skid-flat_ci)) | BENCH |
| D1. Board-in-the-loop | six bench tests against the real Pixhawk | BENCH |
| D3. Pi-in-the-loop | the stack on the Pi 4 drives SITL on the laptop, scored against truth | BENCH |
| C. Whole stack in Gazebo | the previous body: one marker leg finished at the post. That launch is deleted | BENCH (Gazebo), [B-32](BUGS.md). Not this chassis |
| CI | ruff, manifest freshness, build and pytest on apt OpenCV 4.6 like the Pi; ArduRover SITL scenarios built from source | green on GitHub Actions |
| D2. Sim-on-Hardware | built, flashed, booted; simulated IMUs never healthy, so it does not arm; Rover 4.7.2 + parameters restored and verified | BLOCKED, recorded ([sim-and-hitl §5](platform/sim-and-hitl.md#5-sim-on-hardware)) |

## In progress

- **This chassis.** Geometry and the board contract are written. The plant is in Gazebo.
  The ESP32 firmware is not written. A second, larger hull waits until this contract is
  measured. See [chassis.md](platform/chassis.md).

## Blocked

| Item | On | Unblocks |
|---|---|---|
| Arming the real board | a person: onboard compass calibration | GUIDED verbs on the real board with outputs disconnected |
| Everything FIELD | the chassis on the ground | first outdoor run |
| RTK heading and field truth | two u-blox F9P receivers | moving-baseline yaw, centimetre truth for scoring |

## Next, in order

1. **Measure this chassis** when it is on the bench: turn radius, tick calibration against a
   tape, and whether the magnetometer is usable next to the motor driver. No PlatformIO until
   that measurement says the contract's numbers are the real ones. Do not flash.
2. **A serial node** in place of `board_sim`. The planner does not change.
3. **Hailo**, when the hat is on a Pi 5. The sim keeps the Gazebo camera until then.
4. The previous Pixhawk's compass calibration only with a person holding that board.
   The `urc_autonomy` course was next for the skid body. It waits.
5. **First outdoor run** — the first FIELD row.

## Later, with reasons

| Item | Why later |
|---|---|
| Gazebo Jetty + ROS 2 Lyrical | Harmonic is LTS to 2029; switch after the contract tests pass on a second box |
| A second hull (F1/10) | same contract, after this hull is measured |
| H7 Pixhawk | the previous computer's upgrade path. This body is the ESP32 |
| ncnn runtime on the Pi | faster on ARM than ONNX Runtime per the research; switch on measurement |
| DEM terrain, Chrono SCM slip curves | after the flat-ground loop is FIELD |
| Webots backend, Isaac Sim data | only if a second engine or synthetic data earns its cost |
| Nav2 (Regulated Pure Pursuit) | opt-in once there is a map worth planning on |
