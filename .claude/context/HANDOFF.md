# Handoff — read this if you were told to continue

Written 2026-10-23, after the sim moved to the Ackermann chassis. The repo is
[github.com/fh1m/rupsha_ws](https://github.com/fh1m/rupsha_ws). This file is the status a new
session does not have. The rules and the traps live in [`CLAUDE.md`](../../CLAUDE.md); the
standing state lives in [ROADMAP](ROADMAP.md), [BUGS](BUGS.md) and
[capability-map](../../docs/capability-map.md). If this file disagrees with those, those win
and this file is stale.

## 1. Where you are

Inside the distrobox `rupsha_dev`, workspace `/home/fh1m/tmp/rupsha_ws`. The shell you get may be
root. **git, colcon and pytest run as `fh1m`** (`sudo -u fh1m`); git refuses the repo otherwise.
Source ROS and the workspace before anything else:

```bash
sudo -u fh1m bash -lc 'cd /home/fh1m/tmp/rupsha_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash'
```

The venv is `/opt/rupsha-venv` (pymavlink, ruff). `python3` from apt does not have them.
`ROS_DOMAIN_ID=43`. ArduPilot is `~/tmp/ardupilot`, tag `Rover-4.7.2`, or `ARDUPILOT_HOME`.

## 2. What is true right now

| Thing | State |
|---|---|
| The body being built | Ackermann, 20 cm by 10 cm, one driven axle, GPS on, lidar off. Geometry and the disagreement about the steer limit are in [chassis.md](platform/chassis.md). The real radius is unmeasured |
| The board contract | speed in mm/s, steer in radians, ticks, a quaternion, a GPS fix. [firmware-contract.md](platform/firmware-contract.md). `board_sim` fills it. There is no PlatformIO tree yet |
| Default sim | Gazebo Harmonic. `sim.launch.py` spawns the Ackermann plant and `board_sim`. No ArduRover. `test_gazebo_ackermann.py` passed once, scored on Gazebo truth: speed, steer, ticks, camera, GPS, and a dead link |
| Pixhawk 2.4.8, ArduRover 4.7.2, Pi 4 | the **previous** computer. The board is still on the desk. Pre-arm fails on compass and the safety switch. The Pi 4 is down. Do not SSH it |
| Compass calibration | only with a person, and only for that previous Pixhawk: `ros2 run rupsha_control compass_cal --reboot`. Do not start it and walk away. This chassis does not use that compass |
| Previous body in Gazebo | the SITL launch is gone. One marker leg passed once — camera 1.69 m from the tag face, body 2.09 m from the post ([B-32](BUGS.md)). That number is not this chassis |
| The public page | [fh1m.github.io/rupsha_ws](https://fh1m.github.io/rupsha_ws/), red and black. The pictures are drawings. Outdoors is 0 |
| CI | green on the last push: lint, build, pytest on apt OpenCV 4.6, and the SITL scenarios. The new Gazebo test skips when `gz` or `gz_ros2_control` is missing |
| Anything outdoors | still 0. The chassis is being built. It is not here |

## 3. The next task, in order

1. **This hull's contract, on the real chassis, when it is here.** Measure the turn radius.
   The xacro says ±45°, the old bridge said 30°, and the CAD note wants about 39°. Do not
   write a PlatformIO tree, do not flash, and do not start a compass calibration for this body.
2. **A serial node** replaces `board_sim` when the ESP32 exists. The planner keeps the same
   messages. Hailo stays out of the sim until the hat is on a bench.
3. The previous Pixhawk's compass calibration only when a person is holding that board.
   The `urc_autonomy` course was the next Gazebo run for the skid body. It waits until the
   executive speaks this contract.

## 4. How to run the stack

```bash
# tests. Gazebo and SITL are slow; the GUI and the tests do not run together (15 GB RAM, ~2 GB free)
sudo -u fh1m bash -lc 'cd /home/fh1m/tmp/rupsha_ws && source /opt/ros/jazzy/setup.bash && ./build_rupsha.sh && source install/setup.bash && /opt/rupsha-venv/bin/ruff check . && /opt/rupsha-venv/bin/python -m pytest -q -m "not gazebo"'
sudo -u fh1m bash -lc 'cd /home/fh1m/tmp/rupsha_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export GZ_VERSION=harmonic && /opt/rupsha-venv/bin/python -m pytest -q -m gazebo src/rupsha_sim'

# the Ackermann plant. This is the default sim. Localhost discovery, or the
# controller manager never sees the robot description on this subnet.
sudo -u fh1m bash -lc 'cd /home/fh1m/tmp/rupsha_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export GZ_VERSION=harmonic ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && ros2 launch rupsha_sim sim.launch.py headless:=true'

# the previous Pixhawk, on the desk, outputs free. ArduRover is not the sim.
ros2 run rupsha_control param_audit          # exit 0 = the baseline matches
ros2 run rupsha_control bringup_check --bench
pytest -m board src/rupsha_control
```

`sim.launch.py` starts Gazebo, the Ackermann model, the joint controllers, and `board_sim`.
It does not start ArduRover. The old SITL launch is deleted.

## 5. Rules that have already cost a session

- **Never arm with anything on the servo outputs.**
- **Do not change the Pi's boot config. Do not flash firmware. Do not wipe parameters.**
  Sim-on-Hardware is recorded BLOCKED; restoring the board afterwards is how B-27 happened.
- **The bootloader port sorts first.** While the board reboots it appears as
  `usb-ArduPilot_Pixhawk1-BL…`, and a glob of `Pixhawk1*` opens that instead of the flight
  firmware. Tools skip `-BL`. If only the `-BL` port is present, something is holding the
  board in the bootloader: close it and wait. Do not flash.
- **sysid 245 is the manager alone.** `MAV_GCS_SYSID=245`, so a process that heartbeats as 245
  and exits latches the GCS failsafe: the board goes to HOLD and flashes yellow, disarmed,
  until it reboots. Tools and the board tests use `Companion.TOOLS_SYSID` (246). A repo guard
  fails if a new `ArduRover(` call does not pass `sysid=` and is not the manager. A hand-written
  pymavlink script must use 255 or 246, never 245.
- **After any parameter wipe, persist `INS_ACC_ID` 1246218 and `INS_GYR_ID` 2163722** before
  trusting pre-arm. RAM shows the detected ID while EEPROM holds 0, and the message is
  "3D Accel calibration needed" (B-27).
- **The LED.** Flashing yellow: a failsafe (on the desk, the GCS one). Double flash yellow:
  pre-arm failing. Flashing blue: disarmed, pre-arm ok, no GPS. Green: disarmed with a fix.
  Blue needs the compass calibrated and the safety switch pressed. Full table in
  [ardurover-pixhawk](platform/ardurover-pixhawk.md#4-traps).
- **One node touches the board**, through `fc/ardurover.py`. The guard checks the import.
- **NaN means absent. Truth tests, not agreement tests.** Score Gazebo runs on
  `/sim/ground_truth/odometry`, never on `/rupsha/state`.
- **Unmeasured work is BUILT, not working.** Update ROADMAP, BUGS and the capability map in the
  same change as the code. `test_doc_drift.py` fails if a new file under `.claude/context/` is
  not linked from [the book](README.md), or a relative link breaks.
- **Commits are Fahim Faisal's and nobody else's.** No `Co-authored-by`, no trailer, and do not
  change git config:

```bash
sudo -u fh1m git -C /home/fh1m/tmp/rupsha_ws add -A
sudo -u fh1m env GIT_AUTHOR_NAME="Fahim Faisal" GIT_AUTHOR_EMAIL="fahim.2002.faisal@gmail.com" \
  GIT_COMMITTER_NAME="Fahim Faisal" GIT_COMMITTER_EMAIL="fahim.2002.faisal@gmail.com" \
  git -C /home/fh1m/tmp/rupsha_ws -c commit.gpgsign=false commit --no-verify -m "..."
sudo -u fh1m git -C /home/fh1m/tmp/rupsha_ws push
```

  Push only to `fh1m/rupsha_ws`. Never to another repo.

## 6. What changed on 2026-10-22

| # | What |
|---|---|
| B-32 | fixed. The marker leg finishes at the post. Section 2 |
| B-33 | fixed. The vision node in sim uses sim time, so a detection's age is real |
| B-31 | the link is alive on any board message, not the 1 Hz heartbeat, so a slow sim is not "link lost" |
| B-30 | `vision_approach` coasts up to half the last-seen gap instead of stopping at the first dropout |
| B-29 | `sim.launch.py` actually starts: SITL's log dir is created, the manager retries, the executive waits |
| B-28 | bench tools speak as sysid 246 |
| B-27 | accelerometer and gyro IDs written back; pre-arm no longer asks for a 3D accel calibration |
| B-26 | vision publishes tags as `Detection.msg` says (`label "tag"`, confidence 1.0). Before this, no marker leg could ever see its post |

`compass_cal` is new: `src/rupsha_control/rupsha_control/compass_cal.py`.
