# Handoff — read this if you were told to continue

Written 2026-10-22, after the first whole-stack run in Gazebo. The repo is
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
| Pixhawk 2.4.8 on USB, ArduRover 4.7.2, baseline audit | on the desk, 0 parameters differ. Pre-arm fails on **compass not calibrated** and **hardware safety switch** only |
| Compass calibration | needs a person: `ros2 run rupsha_control compass_cal --reboot`, then rotate the board. Do not start it and walk away; it holds the compass in calibration until it finishes or you cancel |
| The Pi 4 | **down.** Do not SSH it, do not run `tools/pil.sh`, do not treat Pi-in-the-loop as runnable |
| Whole stack in Gazebo | a marker leg finishes: GNSS leg, search, approach, verify. Headless `test_gazebo_marker_leg.py` passed once — camera 1.69 m from the tag face, body 2.09 m from the post, `all 1 legs complete` in 30 s ([B-32](BUGS.md), [B-33](BUGS.md)) |
| The public page | [fh1m.github.io/rupsha_ws](https://fh1m.github.io/rupsha_ws/), built from `docs/` in Mongla's red and black, with the same faces |
| CI | green: lint, build, pytest on apt OpenCV 4.6, and the SITL scenarios |
| Anything outdoors | impossible. There is no chassis |

## 3. The next task, in order

1. **Compass calibration** only when a person is holding the board. Pre-arm is still the compass
   and the safety switch. Do not start `compass_cal` and walk away.
2. **A full course in Gazebo.** `urc_autonomy` is the seeded course the executive has not run
   end to end. Score it on Gazebo truth, headless, the same way as the marker leg.
3. The marker survey in [sota §9](sota/README.md#9-how-the-teams-that-win-urc-approach-a-marker)
   has MRDT, MRover, and Waterloo's older `drive_to_ar` branch. BYU, Alberta and the Polish
   teams have no public autonomy code that could be read. Do not copy anyone's code.

## 4. How to run the stack

```bash
# tests. Gazebo and SITL are slow; the GUI and the tests do not run together (15 GB RAM, ~2 GB free)
sudo -u fh1m bash -lc 'cd /home/fh1m/tmp/rupsha_ws && source /opt/ros/jazzy/setup.bash && ./build_rupsha.sh && source install/setup.bash && /opt/rupsha-venv/bin/ruff check . && /opt/rupsha-venv/bin/python -m pytest -q -m "not gazebo"'
sudo -u fh1m bash -lc 'cd /home/fh1m/tmp/rupsha_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export GZ_VERSION=harmonic && /opt/rupsha-venv/bin/python -m pytest -q -m gazebo src/rupsha_sim'

# the whole stack, headless. This is how you test.
sudo -u fh1m bash -lc 'cd /home/fh1m/tmp/rupsha_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export GZ_VERSION=harmonic && ros2 launch rupsha_sim sim.launch.py headless:=true'

# the same stack with the Gazebo window, only when a person wants to watch.
# The window drops the sim to ~0.4x real time. Do not score timing against it.
ros2 launch rupsha_sim sim.launch.py

# one marker leg, once the stack is up and /rupsha/state prearm_ok is true
ros2 run rupsha_planner rupsha arm
ros2 run rupsha_planner rupsha add_marker_leg --lat <lat> --lon <lon> --tag_id 3 --radius 20
ros2 run rupsha_planner rupsha start_autonomy

# the real board
ros2 run rupsha_control param_audit          # exit 0 = the baseline matches
ros2 run rupsha_control bringup_check --bench
pytest -m board src/rupsha_control
```

`sim.launch.py` starts gz, the bridge, SITL (`--model JSON`), then the manager, vision,
localization and the executive. The manager retries its link forever (`link_wait_s` 0) and the
executive waits for the manager. Both used to die on startup (B-29).

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
