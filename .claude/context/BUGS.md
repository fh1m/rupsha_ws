# BUGS — the single defect register

Every defect found, open or fixed. A fixed bug keeps its entry and names what now pins it, so the
same mistake cannot come back quietly. Newest first within each section.

## Open

None in software. The ArUco range reads ~3 % long in Gazebo, which is recorded and not yet
explained ([measured-bars §3](measured-bars.md#3-gazebo-harmonic--sitl-lockstep-rupsha_skid-flat_ci)).

## Hardware observations

| # | Observation | Consequence |
|---|---|---|
| H-02 | Sim-on-Hardware on the F427: both simulated IMUs report unhealthy at 50, 200 and 400 Hz, with and without the documented parameter wipe | it does not arm; the rung is recorded as BLOCKED ([record](platform/records/sim_on_hw_2026-10-21.json)) |
| H-01 | The clone's internal LSM303D compass enumerates intermittently (0 of 613 ArduSub boots, 1 of ≥ 6 Rover boots) | treat the IST8310 as the only compass; an external GNSS/compass module is the fix |

## Fixed — the board and ArduRover

| # | Bug | Fix | Pinned by |
|---|---|---|---|
| B-23 | The failsafe test's 3.5 s bound ignored that HOLD is only *seen* in the next 1 Hz HEARTBEAT; it passed on heartbeat phase and failed at 3.6 s | bound derived from the source: `FS_GCS_TIMEOUT` + `FS_TIMEOUT` + one HEARTBEAT period = 4.5 s | `test_companion_heartbeat_loss_holds_the_rover` |
| B-22 | The Pi-in-the-loop truth recorder imported pymavlink itself, breaking the one-boundary rule | it reads SITL through `ArduRover` (sysid 250) with a listener | `test_only_the_driver_imports_pymavlink` |
| B-20 | `localization_node.py` held `'/rupsha/wheel_speeds'` as a literal | `Topics.WHEEL_SPEEDS` | `test_no_topic_literals_outside_the_manifest` |
| B-19 | The folk velocity + yaw-rate `type_mask` 1511 sets `VZ_IGNORE`; Rover ignores the whole velocity group and the rover does not move | masks built from whole groups: 1479 / 3576 / 1535 / 2559 | `test_board_wire.py` |
| B-18 | `drive 3 m` stopped 2 m short and reported success: GUIDED arrives at the board's `WP_RADIUS` | facade reads `WP_RADIUS`, widens arrival to `WP_RADIUS + 0.25 m` | `test_rupsha_verbs.py` |
| B-17 | Changing `CRUISE_SPEED` alone: 1.0 m/s commanded, 1.43 m/s driven with a 4 s coast | the cruise pair is set together, from a measurement | `test_sim_contract.py` (pair vs model) |
| B-16 | Parameter files with pre-4.7 names load silently wrong | baseline uses `MAVn_`, `MAV_GCS_SYSID`, `ARMING_SKIPCHK` | `param_audit` empty diff |
| B-15 | ExtNav as EKF source without `VISO_TYPE` fails pre-arm with an unhelpful message | `rupsha_extnav.parm` sets both; `bringup_check` names the mismatch | `test_bringup_and_params.py` |

## Fixed — simulation

| # | Bug | Fix | Pinned by |
|---|---|---|---|
| B-14 | The Gazebo rover spun at full output; steering looked mirrored. Real cause: `ArduPilotPlugin` forwards IMU data unrotated and the IMU was z-up, so yaw rate arrived negated — positive feedback. The "mirroring" was snapshot aliasing at ~200°/s | IMU sensor mounted FRD: `<pose degrees="true">0 0 0 180 0 0</pose>` | `test_sim_contract.py` (pose); z-up mutation fails 2 of 3 lockstep tests |
| B-13 | "Accels inconsistent" in SITL JSON — SITL expects the second IMU the model does not have | `INS_ENABLE_MASK 1` | `test_sim_contract.py` (one IMU) |
| B-12 | After B-13, arming asked for "3D Accel calibration" | neutral `INS_ACC2OFFS_*` 0 / `INS_ACC2SCAL_*` 1 | `rupsha_gazebo.parm` |
| B-11 | Gazebo rover crawled at 0.55 m/s: `rover-skid.parm`'s `CRUISE_SPEED 3` did not match the model | the model's own pair, 1.0 m/s at 50 % | `test_gazebo_lockstep.py` (1.00 m/s) |
| B-10 | SITL home drifted ~20 m: `--home` formatted with `:g` truncated latitude to 6 significant digits | `repr(float(v))` | `sitl.py` |
| B-09 | SITL did not create its log directory | `mkdir(parents=True)` | `sitl.py` |
| B-08 | Worldgen PNGs differed byte-for-byte between the laptop and the Pi (different zlib/libpng) | a deterministic PNG writer; freshness compares decoded pixels | `test_sim_contract.py` |

## Fixed — the stack

| # | Bug | Fix | Pinned by |
|---|---|---|---|
| B-21 | `cv2.aruco.ArucoDetector` segfaults on OpenCV 4.6 (the Pi's and CI's apt build); the laptop's 4.13 hid it | below 4.7, detection goes through the legacy `detectMarkers` + `DetectorParameters_create` | `test_aruco_legacy_api.py`; the full suite in `ros:jazzy-ros-base` with apt 4.6 (390 pass) |
| B-07 | Missions aborted at exit with "terminate called without an active exception" | the client shuts down its executor, joins its spin thread, destroys the node, then `try_shutdown` | mission runs in `tools/pil.sh` |
| B-06 | `mission` exited 0 even when a verb failed | returns 1 on any failure | `test_mission_exit_code_reports_a_failed_verb` |

## Fixed — tooling and environment

| # | Bug | Fix | Pinned by |
|---|---|---|---|
| B-25 | `param_audit --apply` aborted on the first parameter the board refuses (e.g. `BARO1_GND_PRESS`, re-measured at boot), leaving the rest unapplied | refusals are reported and the apply continues | `param_audit` |
| B-24 | `param_audit --dump` wrote 6 significant digits: `INS_ACC_ID` 1246218 became `1.24622e+06` | 9 significant digits (float32 round-trips) | `param_audit` |
| B-05 | colcon descended into `third_party/opencv` | `--base-paths src` in `build_rupsha.sh` | the script |
| B-04 | `ardupilot_gazebo` fails to build on the Pi | `RUPSHA_NO_SIM` on aarch64 skips it | the script |
| B-03 | A stray `--` before `"$@"` made colcon reject every argument | removed | the script |
| B-02 | ROS's pytest plugins (`launch_testing`, `launch_ros`, `anyio`) broke plain pytest runs | disabled in `pytest.ini` | `pytest.ini` |
| B-01 | Root-owned `~/.ros` and build files after running tools as root; launch failed, two rclpy tests errored | run everything as `fh1m`; `chown -R` | [box-and-pi](platform/box-and-pi.md) |

## Fixed — Pi-in-the-loop

| # | Bug | Fix |
|---|---|---|
| P-04 | The laptop firewall dropped the Pi's inbound MAVLink | direction reversed: the Pi listens, SITL dials out |
| P-03 | SITL dialled before the Pi's listener existed | `pil.sh` waits for the socket; the manager respawns |
| P-02 | Docker flags after the image name; a multi-line options string broke the remote command | flags first, one line |
| P-01 | `find_ardupilot` failed under the box's `HOME` | also searches beside each parent of the package |
