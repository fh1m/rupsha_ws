# Against the field — the research behind each choice

The research pass before the first line of code: **216 web searches across seven research
threads, plus a code read of 15 MissouriMRDT repositories**. This page keeps the decisions and the
reasons; where a claim was later *measured* here, the measurement is linked and wins.

Each section ends with **Revisit when** — the event that would change the verdict.

## The chassis, read and not copied

The body being built reuses the shape of Rakibul Islam's 2025 WRO chassis
([LazyGo](https://github.com/A-N-M-Noor/LazyGo_WRO2025)). The repository was read. The code
was not ported.

Taken: the 20 cm by 10 cm footprint, one 25GA370 through a differential, 1873 ticks per metre
as the average of the two driven wheels, wheelbase 0.12 m from the xacro, hinge limit ±45°,
wheel radius 0.02 m. GPS is added (u-blox M8N). The lidar is left off.

Not taken: the one-byte serial map, the 12.5 Hz speed PI, the lidar disparity extender, the
parking state machine, BNO055 Euler angles as heading, the four independent wheel joints in
their Gazebo model, and the sonars listed in a yaml the robot does not have. Their three
steer numbers disagree (30°, 45°, about 39°). The plant uses the xacro and says the real
radius is unmeasured.

The sim stays Gazebo Harmonic with `gz_ros2_control`. Leaving ArduRover did not force another
engine. Webots was the only close alternative. F1TENTH Gym is a 2D kinematic racer with no
camera, so it cannot test this stack. Isaac and CARLA are the wrong size, and the Hailo-8 is
not an NVIDIA GPU. There is no official Hailo ROS package. The sim does not pretend to be one.

Section 1 below is the **previous** computer, a Pixhawk running ArduRover. It stays because
that body is still in the repo.

## 1. Firmware: ArduRover 4.7.2 on the Pixhawk 2.4.8

**Chosen:** ArduPilot Rover 4.7.2 stable, `Pixhawk1` target (the chip identified as STM32F427
rev 3, so the full 2 MB build is safe; record in
[`platform/records/`](../platform/records/)).

**Why not PX4:** no release Rover build for fmu-v3; fmu-v3 sits at its flash limit, so uXRCE-DDS
would have to be squeezed in by hand; PX4's HITL covers multirotor and VTOL airframes, not
ground vehicles.

**Traps found in the research, all now encoded in the baseline or a test:** the 4.7 parameter
renames (`SRx_` → `MAVn_`, `SYSID_THISMAV` → `MAV_SYSID`, `ARMING_CHECK` → `ARMING_SKIPCHK`);
`FS_GCS` keys on the GCS sysid; GUIDED velocity commands expire after 3 s.

Sources: [Rover stable for Pixhawk1](https://firmware.ardupilot.org/Rover/stable/Pixhawk1/) ·
[Rover parameter list](https://ardupilot.org/rover/docs/parameters.html) ·
[Rover MAVLink commands](https://ardupilot.org/dev/docs/mavlink-rover-commands.html) ·
[Rover failsafes](https://ardupilot.org/rover/docs/rover-failsafes.html)

**Revisit when:** the board becomes an H7 (Pixhawk 6C/6X, Cube Orange+), which brings Lua, AP_DDS
and QuikTune.

## 2. The board link: one pymavlink node

**Chosen:** a single pymavlink manager, Mongla's choice. **Not MAVROS** (a second, generic
abstraction over the same link, and a second owner of the board), **not AP_DDS** (not in stable
F4 builds; ArduPilot's own ROS 2 docs point at boards with more headroom; a 2026 forum report of
Pixhawk1 + Pi 4B DDS showed no topics). The Pixhawk1's 43 KB Lua heap rules out on-board scripting.

Sources: [ArduPilot ROS 2 / DDS](https://ardupilot.org/dev/docs/ros2.html)

**Revisit when:** the H7 board lands — AP_DDS then becomes a candidate for the high-rate streams,
behind the same manager.

## 3. The control split

**Chosen:** the board closes turn-rate and speed (`ATC_STR_RAT_*`, `ATC_SPEED_*`), mixes skid
steering (`SERVO1/3_FUNCTION` 73/74), follows S-curves and owns every failsafe. The Pi sends
GUIDED intent: `SET_POSITION_TARGET_GLOBAL_INT` for GNSS legs; `SET_POSITION_TARGET_LOCAL_NED`
in body frame for vision and search verbs at 10 Hz. The research draft carried type_mask 1511;
SITL showed ArduRover wants **1479** for velocity + yaw rate — see the
[board traps](../platform/ardurover-pixhawk.md).

Nav2 (Regulated Pure Pursuit) stays an opt-in; MPPI waits for the Orin.

Sources: [steering-rate tuning](https://ardupilot.org/rover/docs/rover-tuning-steering-rate.html) ·
[motor and servo connections](https://ardupilot.org/rover/docs/rover-motor-and-servo-connections.html) ·
[Nav2](https://docs.nav2.org/)

## 4. What the Missouri S&T Mars Rover Design Team teaches

URC first place 2025 and 2026. Their wins come from discipline, not exotic algorithms.

| Adopted | Where it lives in Rupsha |
|---|---|
| One manifest as the single source of truth (`RoveComm_Base/manifest.json`) | [`manifest/rupsha.yaml`](../../../manifest/rupsha.yaml) + codegen, freshness test |
| Legs as first-class commands | `add_position_leg`, `add_marker_leg`, `add_object_leg`, `start_autonomy` |
| An explicit `AutonomyState` machine | `rupsha_planner/executive.py` |
| A stuck ladder as data | reverse; +65° and reverse; −65° and reverse; STUCK |
| Verify before success | GNSS averaging; multi-frame tag and object confirmation |
| Safety on firmware | ArduRover failsafes, `FS_GCS` keyed to the Pi |

| Not copied | Why |
|---|---|
| RoveComm, a custom protocol with five bindings | ROS 2 and MAVLink already exist |
| String-keyed manifest lookups | their own drift bug: code still referenced `ACCURACYDATA` and `CELLVOLTAGE` after the header dropped them — Rupsha's constants are static attributes |
| A recompile-to-tune constants file | tunables are ROS parameters |
| A 40 GB UE5 simulator as the primary test bed | no headless CI; speaks RoveComm, not ArduPilot |

Sources: [Autonomy_Software](https://github.com/MissouriMRDT/Autonomy_Software) ·
[RoveComm_Base](https://github.com/MissouriMRDT/RoveComm_Base) ·
[DriveBoard_Software](https://github.com/MissouriMRDT/DriveBoard_Software) ·
[Smart_MoCo](https://github.com/MissouriMRDT/Smart_MoCo) ·
[RoveSoSimulator](https://github.com/MissouriMRDT/RoveSoSimulator) ·
[URC](https://urc.marssociety.org/) · for comparison, Michigan's
[MRover](https://github.com/umrover/mrover-ros2)

## 5. The mission layer

**Chosen:** plain Python — a DSL over the verbs, and a small executive under it. No behaviour-tree
or state-machine framework: the URC teams read (MRover, Missouri S&T, BRACU) use plain state
machines, BYU puts plain mission logic over Nav2, and Mongla retired YASMIN for the same reason.

## 6. Localization

**Chosen:** EKF3 on the board is the navigation authority. The Pi runs a **shadow
right-invariant EKF** (the Hartley line of work, carried over from Mongla's `inekf.py`) that can
only advise; skid-steer odometry uses the ICR model with gyro-versus-encoder slip gating. On a
GNSS-denied leg the Pi may feed `ODOMETRY` into EKF3 through source set 2 (`EK3_SRC2_POSXY=6`),
switched by `MAV_CMD_SET_EKF_SOURCE_SET` — opt-in, off by default.

Later: dual u-blox F9P moving-baseline RTK (`GPS1_TYPE=17`, `GPS2_TYPE=18`, `EK3_SRC1_YAW=3`) as
heading and as field truth; OpenVINS; on the Orin, GLIM-class lidar odometry with KISS-ICP as a
cross-check and `elevation_mapping_cupy`.

Sources: Hartley et al.,
[Contact-Aided Invariant Extended Kalman Filtering for Robot State Estimation](https://arxiv.org/abs/1904.09251)
(IJRR 2020), [reference code](https://github.com/RossHartley/invariant-ekf) · Mandow et al.,
*Experimental kinematics for wheeled skid-steer mobile robots* (IROS 2007) · Chauchat et al. on
invariant filtering for non-holonomic wheeled robots (CDC 2024) ·
[ArduPilot non-GPS navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html) ·
[GPS for yaw](https://ardupilot.org/rover/docs/common-gps-for-yaw.html) ·
[OpenVINS](https://github.com/rpng/open_vins) · [GLIM](https://github.com/koide3/glim) ·
[KISS-ICP](https://github.com/PRBonn/kiss-icp) ·
[elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)

## 7. Vision on a Pi 4

**Chosen:** ArUco 4x4_50 at full resolution in grayscale with `SOLVEPNP_IPPE_SQUARE` and
temporal flip disambiguation (Mongla's); YOLO at 320 px on the CPU, with detectors scheduled by
mission phase so they never compete. The research pointed at ncnn as the fastest ARM runtime;
the first cut ships ONNX Runtime (one wheel on both architectures) and the Pi rate is **not yet
measured** — it is BUILT until it is.

Hailo cannot attach to a Pi 4 (no PCIe); Coral's stack is archived. The upgrade is an OAK-D over
USB 3, then Orin + TensorRT.

**Found the hard way:** the Pi's and CI's apt OpenCV is 4.6, where `cv2.aruco.ArucoDetector`
segfaults; the legacy `detectMarkers` API works. See [BUGS](../BUGS.md).

Sources: [OpenCV ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) ·
[Ultralytics on Raspberry Pi](https://docs.ultralytics.com/guides/raspberry-pi/) ·
[ncnn](https://github.com/Tencent/ncnn)

## 8. Simulation

Compared on this box (RTX 2060 6 GB, 15 GB RAM) with ArduPilot SITL in the loop:

| Option | Verdict | Why |
|---|---|---|
| **Gazebo Harmonic** | **primary** | REP-2000's pairing for Jazzy, LTS to 2029; `ardupilot_gazebo` JSON lockstep; headless EGL rendering for CI; DEM terrain; a WildThumper 6WD example |
| Gazebo Jetty + ROS 2 Lyrical | migration path | plugins already written against the unversioned `gz_*_vendor` targets |
| Webots | documented option behind the contract | well engineered and deterministic, `webots_ros2` exists; but no stable release since R2025a, a separate ArduPilot bridge, a second physics engine to tune |
| Isaac Sim | deferred | best synthetic data, but needs far more GPU than a 2060 — for YOLO training data only, on borrowed hardware |
| Project Chrono (SCM) | offline | the best wheel–soil physics; used to fit slip curves for Gazebo, not in the loop |
| UE5 (RoveSoSimulator, Colosseum, AirSim) | not chosen | heavy, weak headless CI, no ArduPilot lockstep |
| O3DE, MuJoCo, Genesis, CARLA, Unity | not chosen | no ArduPilot bridge, wrong focus, or no LTS |

The contract layer (fixed `/clock`, `/sim/<cam>/…`, `/sim/ground_truth/…`, the same tests for
any backend) and seeded worldgen are MRDT's procedural-maps idea made engine-neutral.

**ArduPilot has no real HITL** — classic HIL was removed — so the honest hardware chain is
board-in-the-loop, Pi-in-the-loop and Sim-on-Hardware. See
[sim-and-hitl](../platform/sim-and-hitl.md).

Sources: [REP-2000](https://www.ros.org/reps/rep-2000.html) ·
[Gazebo releases](https://gazebosim.org/docs/harmonic/releases) ·
[ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) ·
[simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) ·
[Webots releases](https://github.com/cyberbotics/webots/releases) ·
[webots_ros2](https://github.com/cyberbotics/webots_ros2) ·
[Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) ·
[Project Chrono](https://github.com/projectchrono/chrono) ·
[HIL removal, ArduPilot PR #17721](https://github.com/ArduPilot/ardupilot/pull/17721) ·
[Sim-on-Hardware](https://ardupilot.org/dev/docs/sim-on-hardware.html)

## 9. How the teams that win URC approach a marker

Read on 2026-10-22, from the current code, because [B-32](../BUGS.md) was exactly this problem:
Rupsha found the tag at the edge of detection and then lost it. The fix is in the tree; the
headless marker leg now finishes. Every claim below was read in the
team's own source on that date; nothing was copied. MRDT is the team Rupsha measures itself
against: its states, leg queue and stuck ladder are already the shape of
[`executive.py`](../../../src/rupsha_planner/rupsha_planner/executive.py).

**Missouri S&T MRDT** ([Autonomy_Software](https://github.com/MissouriMRDT/Autonomy_Software)).
- *Keep where the tag is, not where it was seen.* Each ArUco detection carries a geolocated UTM
  position. When the camera loses the tag, the rover drives to that position; only with no
  position history does it coast on the last heading
  ([`ApproachingMarkerState.cpp`](https://github.com/MissouriMRDT/Autonomy_Software/blob/main/src/states/ApproachingMarkerState.cpp)):

  ```text
  [FALLBACK 1] Visual lost. Driving to last known geolocated UTM
  [FALLBACK 2] Visual lost & no geolocation history. Coasting along last known heading.
  ```
- *The pose it uses is the pose now.* The geolocation takes the ZED's stereo point cloud and
  `SmartRetrieveRoverPose()` at processing time
  ([`TagDetector.cpp`](https://github.com/MissouriMRDT/Autonomy_Software/blob/main/src/vision/aruco/TagDetector.cpp)).
  Stereo depth and a short frame latency make that tolerable for them.
- *A detection must live before it counts.* `BBOX_MIN_LIFETIME_THRESHOLD = 0.15` s, and a KCF
  multi-tracker carries the box across decode dropouts for 1 s, dropping it after 3 s
  ([`BoundingBoxTracking.cpp`](https://github.com/MissouriMRDT/Autonomy_Software/blob/main/src/util/vision/BoundingBoxTracking.cpp)).
- *Patience.* The approach waits a 2 s buffer before it calls the tag lost, gives up 15 s after
  that, finishes at 2.0 m, and then watches the tag for 5 s to verify
  ([`AutonomyConstants.cpp`](https://github.com/MissouriMRDT/Autonomy_Software/blob/main/src/AutonomyConstants.cpp)).
- *Search is waypoints.* A spiral of geodetic points (57° angular step, 1.0 m spacing, 2.0 m
  arrival), followed by pure pursuit. A failed spiral is retried reversed, then as a zigzag and a
  snake ([`SearchPattern.hpp`](https://github.com/MissouriMRDT/Autonomy_Software/blob/main/src/algorithms/SearchPattern.hpp)).
- They test in their own simulator, built in Unreal Engine 5
  ([RoveSoSimulator](https://github.com/MissouriMRDT/RoveSoSimulator)).

**Michigan MRover** ([mrover-ros2](https://github.com/umrover/mrover-ros2)).
- *Hits before a position.* The tag detector counts hits per id; a hit raises the count and a
  missed frame lowers it. A tag is published as a TF frame in the map only above
  `mMinTagHitCountBeforePublish`, with `max_hits: 10`
  ([`tag_detector.processing.cpp`](https://github.com/umrover/mrover-ros2/blob/main/perception/tag_detector/tag_detector.processing.cpp)).
  The TF is stamped `now()`, so the pose is the latest one, as at MRDT.
- *The map position outlives the pixels.* The approach reads the tag's map position and
  forgets it only after `target_expiration_duration: 60.0` s
  ([`approach_target.py`](https://github.com/umrover/mrover-ros2/blob/main/navigation/approach_target.py),
  [`config/navigation.yaml`](https://github.com/umrover/mrover-ros2/blob/main/config/navigation.yaml)).
- *Far away, bearing only, and never an old one.* A tag seen only by the long-range camera is
  projected `long_range.distance_ahead: 3.0` m along its bearing. Once its hit count has decayed,
  the bearing is replaced by straight ahead rather than steered on stale
  ([`long_range.py`](https://github.com/umrover/mrover-ros2/blob/main/navigation/long_range.py)).
  This is B-32's failure, avoided by rule.
- *Search is waypoints too.* `spiral_traj` builds segments around the GPS point
  (`distance_between_spirals: 3.0`, `coverage_radius: 20.0`); they stop `tag_stop_threshold: 1.75` m
  from the tag. Waypoints let the board close each leg while the camera looks.

**BYU** won the 2025 URC autonomous navigation task (first of 114 teams, fourth overall,
[BYU Magazine](https://magazine.byu.edu/article/byu-mars-rover-team/)). Their autonomy code is
not public: the newest repository in [BYUMarsRover](https://github.com/BYUMarsRover) is a ZED
wrapper for a custom detector. Their 2025 telemetry paper
([UA repository](https://repository.arizona.edu/handle/10150/679589)) refused automated reads
(HTTP 403) and is unread.

**Waterloo, an older public branch.** Current
[Sparky-CIRC-2026](https://github.com/uwrobotics/Sparky-CIRC-2026) has no marker code.
[uwrt_mars_rover](https://github.com/uwrobotics/uwrt_mars_rover) branch
`azumnanji/basic_drive_to_ar` does, and it is ROS 1.
[`drive_to_ar_server.cpp`](https://github.com/uwrobotics/uwrt_mars_rover/blob/azumnanji/basic_drive_to_ar/uwrt_mars_rover_autonomy/uwrt_mars_rover_drive_to_ar/src/drive_to_ar_server.cpp)
takes the tag pose (or the midpoint of two) and streams `cmd_vel` until the odometry is inside
`DISTANCE_TOLERANCE` 0.5 m. Linear speed is `3.0 * distance` and yaw rate is `1.5 * heading error`
(those two defaults).
[`spiralSearch_server.cpp`](https://github.com/uwrobotics/uwrt_mars_rover/blob/azumnanji/basic_drive_to_ar/uwrt_mars_rover_autonomy/uwrt_mars_rover_spiral/src/spiralSearch_server.cpp)
opens an arc whose radius is `linear / angular` until the goal radius. Not copied.

**Not public, or not found.** Waterloo's URC 2025 technical report is on a Confluence that was not
read. Alberta's team (SPEAR) has no repository we could find. AGH Space Systems, the ERC 2022
winner, publishes no rover code ([AGH-Space-Systems](https://github.com/AGH-Space-Systems)).
PCz, Kalman and the other Polish teams: no public organisation found. Those gaps stay open.

**What Rupsha took, and where it goes further.** It took the geolocated fallback, the hit gate
and the long hold, as ideas, in our own code
([`rupsha.py`](../../../src/rupsha_control/rupsha_control/rupsha.py)):
- an estimate in the local frame, the 1/range²-weighted mean of the last 10 fixes, held 30 s;
- 3 distinct frames before the search hands over, closing to 4 m of the estimate to collect them;
- the approach steers on pixels inside the 4.5 m reliable range, and otherwise hands the board
  a point `standoff − WP_RADIUS` short of the estimate, so the board's own stop is the standoff.
  It finishes when the camera is within `standoff + 0.25` m and the tag was seen this pass;
- a search that already holds 3 fixes drives back to them instead of spiraling;
- the coast survives only for the case with no local position.

Where it differs: each fix uses the rover pose **at the image's capture time**, interpolated
from a 3 s history, not the pose now. Both teams have stereo depth and less latency. Rupsha has
one monocular camera on a Pi 4 and steers on detections up to 0.5 s old while pivoting at up to
30°/s. On the fake board, the pose now puts a 7 m fix 1.2 m off. The headless marker leg now
finishes: camera 1.69 m from the tag face.

**Revisit when:** a camera with more pixels, or a bigger tag, moves the reliable range past the
GPS uncertainty of the leg. Until then the estimate is what closes the gap.
