# ROADMAP — the one status file

Where Rupsha is, what is left, what is blocked. If this disagrees with
[capability-map](../../docs/capability-map.md), fix whichever is wrong in the same change.

**Last updated 2026-10-22.** Nothing is FIELD: the chassis does not exist.

## Done

| Phase | What | State |
|---|---|---|
| A. Scaffold | seven packages, manifest + codegen, facade, manager, executive, DSL, CLI, repo guards | BUILT, tests green |
| B. Firmware | ArduSub 4.5.1 backed up; ArduRover 4.7.2 flashed; baseline loaded and audited; accel calibration restored | BENCH |
| C. Simulation | SITL scenarios; Gazebo Harmonic + SITL JSON lockstep on a 6WD model; seeded worlds | BENCH |
| C. ArUco vs truth | the rover drives to stations in Gazebo; range ~3 % long, reach ~6 m for a 20 cm tag ([measured-bars §3](measured-bars.md#3-gazebo-harmonic--sitl-lockstep-rupsha_skid-flat_ci)) | BENCH |
| D1. Board-in-the-loop | six bench tests against the real Pixhawk | BENCH |
| D3. Pi-in-the-loop | the stack on the Pi 4 drives SITL on the laptop, scored against truth | BENCH |
| C. Whole stack in Gazebo | `sim.launch.py` brings up manager, vision, localization and executive; one marker leg finishes at the post (GNSS, search, approach, verify) | BENCH (Gazebo), [B-32](BUGS.md) |
| CI | ruff, manifest freshness, build and pytest on apt OpenCV 4.6 like the Pi; ArduRover SITL scenarios built from source | green on GitHub Actions |
| D2. Sim-on-Hardware | built, flashed, booted; simulated IMUs never healthy, so it does not arm; Rover 4.7.2 + parameters restored and verified | BLOCKED, recorded ([sim-and-hitl §5](platform/sim-and-hitl.md#5-sim-on-hardware)) |

## In progress

- **A full course in Gazebo** — the executive has run one marker leg. The seeded `urc_autonomy`
  course has not been run end to end.
- **The marker survey** is written up in [sota §9](sota/README.md#9-how-the-teams-that-win-urc-approach-a-marker)
  for MRDT, MRover and Waterloo's older public branch. BYU, Alberta and the Polish teams publish
  no autonomy code that could be read.

## Blocked

| Item | On | Unblocks |
|---|---|---|
| Arming the real board | a person: onboard compass calibration | GUIDED verbs on the real board with outputs disconnected |
| Everything FIELD | the chassis, motors, drivers, battery | first outdoor run |
| RTK heading and field truth | two u-blox F9P receivers | moving-baseline yaw, centimetre truth for scoring |

## Next, in order

1. **Compass calibration**, then the board-in-the-loop GUIDED run: `SERVO_OUTPUT_RAW` showing skid
   mixing respond to each verb, disarmed-on-loss verified by pulling USB.
2. **Front camera on the Pi**: `sensors_check` PASS; intrinsics calibrated into
   `rupsha_sensors/config/calibration/`; YOLO and ArUco rates measured on the Pi.
3. **Executive in Gazebo**: a seeded URC-style course end to end (`urc_autonomy`), headless, scored
   on Gazebo truth.
4. **Chassis**: measure the real cruise pair, steering feed-forward and stuck thresholds; the Gazebo
   model regenerated with Rupsha's geometry.
5. **First outdoor run** — the first FIELD row.

## Later, with reasons

| Item | Why later |
|---|---|
| Gazebo Jetty + ROS 2 Lyrical | Harmonic is LTS to 2029; switch after the contract tests pass on a second box |
| H7 board (Pixhawk 6C/6X) | brings Lua, AP_DDS, QuikTune; when the Orin arrives |
| OAK-D, then Orin + TensorRT | the Pi 4 CPU limits YOLO; no PCIe for Hailo |
| ncnn runtime on the Pi | faster on ARM than ONNX Runtime per the research; switch on measurement |
| DEM terrain, Chrono SCM slip curves | after the flat-ground loop is FIELD |
| Webots backend, Isaac Sim data | only if a second engine or synthetic data earns its cost |
| Nav2 (Regulated Pure Pursuit) | opt-in once there is a map worth planning on |
