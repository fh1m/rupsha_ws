# rupsha_control — the only thing that touches the board

**State: BENCH** (real board on the desk; SITL; Gazebo; Pi-in-the-loop).

## Modules

| Module | What it does |
|---|---|
| `fc/base.py` | the `FlightController` contract and `Telemetry` (NaN for absent) |
| `fc/ardurover.py` | ArduRover over MAVLink — **the only pymavlink import in the workspace**; one reader thread, sends under a lock, the `type_mask` groups |
| `fc/fake.py` | `FakeRover`: kinematic skid model in simulated time, for every test that does not need a simulator |
| `rupsha.py` | the facade: one method per verb, cooperative abort, `WP_RADIUS` widening, results with outcome + message |
| `dispatch.py` | verb registry → facade calls; refuses unknown verbs and fields before dispatch |
| `rover_manager_node.py` | the ROS node: serves `/rupsha/move`, publishes `/rupsha/state` (NaN when absent), relays the board IMU / GNSS, optional ExtNav uplink |
| `heartbeat.py` | the companion heartbeat on its own thread — `FS_GCS` depends on it never starving |
| `loop_meter.py` | exact / average / 1 %-low loop rates, published as diagnostics |
| `bringup_check.py` | before the manager: link, firmware, baseline, EKF source, sensors — PASS / FAIL / ABSENT with reasons |
| `param_audit.py` | diff the board against the repo's parm files; `--apply` to load. Speaks as `Companion.TOOLS_SYSID`, never 245 |
| `compass_cal.py` | onboard compass calibration for a person holding the board: progress, a saved fit, `--reboot`. Ctrl-C cancels it on the board |
| `manifest.py` | **generated** from `manifest/rupsha.yaml` — never edit |

## Configuration

- `config/manager.yaml` — URL (`/dev/serial/by-id/usb-ArduPilot_Pixhawk1*`), baud, state rate,
  `extnav_uplink: false`.
- `config/ardupilot/rupsha_baseline.parm` — the board baseline
  ([explained line by line](../platform/ardurover-pixhawk.md#3-the-baseline-line-by-line)).
- `config/ardupilot/rupsha_extnav.parm` — opt-in overlay for GNSS-denied legs.

## Launch

`ros2 launch rupsha_control bringup.launch.py url:=<mavlink url>` — the manager with
`respawn=True`. The same node runs on USB, TELEM2, SITL over TCP/UDP and Pi-in-the-loop; only
`url` changes.

## Tests

`test_board_wire.py` (masks, frames, version parsing) · `test_rupsha_verbs.py` (every verb on
`FakeRover`) · `test_dispatch.py` · `test_bringup_and_params.py` · `test_board_bench.py`
(`-m board`, the real Pixhawk) · `test_compass_cal.py` · `test_manager_link.py` (the manager
retries a board that is not up yet).
