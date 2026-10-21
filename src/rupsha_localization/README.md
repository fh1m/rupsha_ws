# rupsha_localization: a second opinion on where the rover is

A shadow estimator on the Pi: a planar right-invariant EKF on SE(2) that
cross-checks the Pixhawk's EKF3 and, only when the operator opts in, offers an
ExtNav odometry for GNSS-denied legs.

## Why the board is the authority

ArduRover 4.7.2's EKF3 navigates. It runs at IMU rate next to the sensors,
owns the failsafes, and is what the autopilot's guidance actually consumes.
A Pi-side filter that tried to replace it would add a USB hop, a Linux
scheduler and a second set of tuning to the one loop that must not stall.

So this package does two narrower things:

1. **Health cross-check.** It estimates the pose independently of EKF3's logic
   and publishes an `OK` / `DIVERGED` / `ABSENT` verdict on `/diagnostics`.
   `ABSENT` is its own state: missing data is never reported as agreement.
2. **GNSS-denied legs (opt-in).** With `extnav_enabled:=true` and position std
   under `extnav_max_std_m`, the same estimate goes out on
   `/rupsha/extnav/odometry`. `rupsha_control`, the only node that talks to
   the board, uplinks it as MAVLink `ODOMETRY` for EK3 source set 2 (ExtNav).
   This package never touches the board.

What the shadow is independent of, stated plainly: on the current hardware the
GNSS fix is the board's receiver and, with no wheel encoders, the speed is the
board's groundspeed. The shadow is independent of EKF3's **fusion**, not of its
**sensors**. A divergence means one fusion is wrong; agreement does not prove
the shared sensors right.

## The ExtNav pre-arm trap

ArduPilot's pre-arm checks validate **every** configured EK3 source set, not
just the active one. Configuring source set 2 as ExtNav without `VISO_TYPE`
set blocks arming even while source set 1 (GPS) is in use. That is why ExtNav
is **off by default** here and in `rupsha_control`: enabling it is a deliberate,
per-event change to both the board parameters and this node's
`extnav_enabled`.

## What is in it

| Module | Role |
|---|---|
| `se2.py` | SE(2) exp/log, adjoint, compose, `wrap_pi`; compass/ENU yaw conversion |
| `riekf.py` | Right-invariant EKF (Barrau & Bonnabel, IEEE TAC 2017) with gyro bias; GNSS, heading and zero-velocity updates; chi-square 95% gate on every update, with a ledger of the results |
| `icr_odometry.py` | Skid-steer ICR kinematics (Mandow et al., IROS 2007), least-squares ICR fit, and the gyro-versus-tracks slip gate |
| `geodesy.py` | WGS84 lat/lon to and from a local ENU tangent plane via ECEF |
| `health.py` | Pure shadow-versus-board verdict |
| `shadow.py` | Every wiring decision (origin, velocity source, gaps, ZUPT, ExtNav gate), tested without ROS |
| `localization_node.py` | The `localization` node: message conversion only |

Topics: subscribes `/rupsha/imu`, `/rupsha/navsat`, `/rupsha/state` and the
optional `/rupsha/wheel_speeds` (`Float32MultiArray [vL, vR]`, m/s). It publishes
`/rupsha/localization/odometry` (frame `odom` to `base_link`, ENU origin at the
first 3D fix), `/diagnostics`, and `/rupsha/extnav/odometry` only when gated
in. Before the first fix with board `fix_type >= 3`, it publishes **no
odometry**: there is no frame to put it in, and a pose of zeros would look like
a real one.

Contract with the IMU publisher: `/rupsha/imu` angular velocity is REP-103 FLU
(z up, counter-clockwise positive). The node checks this against the board's
`yaw_rate_dps` and raises an ERROR diagnostic if the signs mostly disagree.

## Honest state per capability

BUILT means implemented and passing seeded synthetic truth tests. FIELD means
validated on the rover against independent ground truth. **Nothing here is
FIELD yet.**

| Capability | State | Evidence / gap |
|---|---|---|
| RIEKF pose, GNSS + speed + gyro | BUILT (synthetic-tested) | Figure-eight, 1 Hz 1.5 m GNSS: RMS < 1.0 m (worst of 40 seeds 0.76 m) |
| Consistency | BUILT | Ungated ANEES 3.04 for 3 dof; gated 3.37 (12% overconfident, caused by the gate's selection effect, measured) |
| Heading from unknown | BUILT | Converges from ±90°/170° on GNSS + motion alone; about 1 run in 5 needs a gate-lock-out relocalisation, which is counted |
| GNSS outlier gate | BUILT | A 50 m jump is rejected and counted |
| Gyro bias by ZUPT | BUILT | Learns a 0.008 rad/s bias at standstill |
| ICR skid-steer odometry | BUILT, **UNMEASURED** | `icr_chi = 1.0` is ideal diff-drive; fit on logged ground truth before use |
| Slip gate | BUILT | Needs wheel speeds, which do not exist on current hardware |
| Health verdict | BUILT | Thresholds (5 m, 15°) are GNSS-noise placeholders, not field-derived |
| ExtNav uplink | BUILT (publisher side only) | Not flown; board-side source switching untested |

Known limits: VFR_HUD groundspeed is unsigned, so without wheel speeds a
reversing rover is propagated forwards. The GNSS gate catches it and the
filter relocalises, which is visible in the ledger. Inputs are stamped on
arrival with the node clock, because board-time stamps are not yet published.

## Upgrade path

1. **RTK GNSS with moving-baseline yaw on the board.** This gives a
   centimetre-level fix and a magnetometer-free heading to EKF3 directly. It is
   the biggest win for the least Pi work, and makes this package's role mostly
   cross-check.
2. **OAK-D + OpenVINS VIO.** A real ExtNav source for GNSS-denied legs, fused
   here first as a body-velocity or pose update and gated like everything else.
3. **Jetson Orin: LiDAR-inertial odometry** (GLIM / FAST-LIO2 family), with
   **elevation_mapping_cupy** for traversability. At that point the ExtNav
   candidate is LiDAR-inertial, and this filter becomes the arbiter between it
   and GNSS.

## Running

```bash
ros2 launch rupsha_localization localization.launch.py              # shadow only
ros2 launch rupsha_localization localization.launch.py extnav_enabled:=true
```

Tests, from the workspace root (pure-Python cores; the node test runs only
where `rclpy` and `rupsha_interfaces` are built):

```bash
python3 -m pytest src/rupsha_localization -q
```
