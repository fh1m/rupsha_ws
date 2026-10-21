# The box and the Pi — where the code runs, and the traps of each

Two machines run the same ROS 2 Jazzy stack: the laptop's distrobox for development and
simulation, and the Raspberry Pi 4 for the rover. Host-level history (mirrors, overclocking,
the Pi's boot config) lives in [`NOTES.md`](../../../NOTES.md); this page is what the code needs.

## 1. The laptop box: `rupsha_dev`

| | |
|---|---|
| Image | `rupsha_dev:jazzy-cuda128` — Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, CUDA 12.8 ([`docker/Dockerfile`](../../../docker/Dockerfile)) |
| Create | [`docker/create_rupsha_dev.sh`](../../../docker/create_rupsha_dev.sh) (re-run safe; **wipes the container layer**) |
| After a rebuild | [`docker/restore_box_after_rebuild.sh`](../../../docker/restore_box_after_rebuild.sh) — OpenCV 4.13 CUDA from the kept build tree, venv links, tools |
| Home | `~/Envs/dockers/rupsha_dev` on the host; the workspace at `~/tmp/rupsha_ws` |
| ROS | `ROS_DOMAIN_ID=43`, CycloneDDS. Mongla's box is Humble on domain 42 — never mix them |
| Python | the venv `/opt/rupsha-venv` holds pymavlink, onnxruntime, ruff, pytest; ROS's `/usr/bin/python3` does not |
| ArduPilot | `~/tmp/ardupilot` at tag `Rover-4.7.2`; SITL built with `./waf configure --board sitl && ./waf rover` |
| Devices | `init-hook.sh` recreates the host's GIDs (uucp 985, video 984, render 988, i2c 951) so the Pixhawk and cameras open |

### Traps in the box

- **Run git, colcon and pytest as `fh1m`, never as root.** Root-owned files under `build/`,
  `install/` or `~/.ros` break later runs as `fh1m` — a root-owned `~/.ros` made launch fail
  and two rclpy tests error. Fix: `sudo chown -R fh1m:fh1m ~/tmp/rupsha_ws ~/.ros`.
- **colcon through the venv python, with `--base-paths src`.** `build_rupsha.sh` does both;
  without the second, colcon descends into `third_party/opencv` and tries to build it.
- **pytest plugins from ROS collide.** `pytest.ini` disables `anyio`, `launch_testing` and
  `launch_ros` (that is the entry-point name of `launch_testing_ros`'s plugin).
- **Two OpenCVs.** The box has 4.13 (CUDA, built here); the Pi and CI have Ubuntu's apt 4.6.
  APIs added after 4.6 must be guarded — `cv2.aruco.ArucoDetector` segfaults on 4.6,
  `generateImageMarker` does not exist there (`drawMarker` does).
- **`GZ_VERSION=harmonic`** must be set when building `ardupilot_gazebo`; `build_rupsha.sh`
  exports it.

## 2. The Pi: `rupsha-pi`

| | |
|---|---|
| Hardware | Raspberry Pi 4B, 8 GB, 4 cores, aarch64 |
| OS | Ubuntu 24.04.5, Docker 29 |
| Address | `192.168.0.102` (`rupsha-pi.local` once mDNS works); key `~/.ssh/id_ed25519_rupsha_pi` |
| Image | `rupsha_pi:jazzy` from [`docker/Dockerfile.pi`](../../../docker/Dockerfile.pi) — the same Jazzy stack, no CUDA |
| Launcher | [`docker/run_rupsha_pi.sh`](../../../docker/run_rupsha_pi.sh): privileged, host network/IPC/PID, `/dev` mounted, hardware GIDs |
| Workspace | `~/rupsha_ws` on the Pi, mounted at `~/tmp/rupsha_ws` inside the container (same path as the laptop) |
| udev | [`docker/99-rupsha-pixhawk.rules`](../../../docker/99-rupsha-pixhawk.rules): `/dev/pixhawk` for `1209:5741`, and ModemManager told to leave it alone |

Measured on the Pi (Pi-in-the-loop run): build ≈ 2 min; manager + mission container at
11.5–12.4 % CPU and 114 MiB.

### Traps on the Pi

- **`ardupilot_gazebo` does not build on aarch64 here** and is not needed on the rover.
  `build_rupsha.sh` sets `RUPSHA_NO_SIM` on aarch64 and skips it.
- **The Pi's `~/rupsha_ws` holds scratch files.** Sync with `rsync` *without* `--delete`.
- **Docker options spliced into an ssh command must be one line** — a newline ends `docker run`.
  Flags go before the image name.
- **Thermals:** the stock heatsink reached 75.9 °C at 1.8 GHz under load. A fan comes before any
  overclock. The boot config is not changed by this stack.

## 3. Between the two

The laptop's firewall drops inbound connections and is left alone. Anything that crosses
machines dials **from the laptop to the Pi**: Pi-in-the-loop runs the manager on the Pi as a TCP
listener and has SITL connect out to it. See [sim-and-hitl §4](sim-and-hitl.md#4-pi-in-the-loop).
