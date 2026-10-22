# rupsha_ws agent notes

**If you were told to continue the work, read [`.claude/context/HANDOFF.md`](.claude/context/HANDOFF.md)
first.** It has the open task, the board's state today, and the rules that have already cost a
session. Then [`CLAUDE.md`](CLAUDE.md) for the vehicle, the traps and the architecture, and the
rest of the docs in order: [`.claude/context/README.md`](.claude/context/README.md).
Host and box history: [`NOTES.md`](NOTES.md).

Dev env: distrobox `rupsha_dev` (Ubuntu 24.04, ROS 2 Jazzy, CUDA 12.8, cuDNN 9, torch cu128).
Image: `rupsha_dev:jazzy-cuda128`, built from `docker/Dockerfile`. Recreate with `docker/create_rupsha_dev.sh`.

Run all build/test commands inside the container, as `fh1m` (never root):

    distrobox enter rupsha_dev -- bash -lc 'cd ~/tmp/rupsha_ws && ./build_rupsha.sh && pytest'

Env: ROS_DISTRO=jazzy, RMW=rmw_cyclonedds_cpp, ROS_DOMAIN_ID=43 (auv-ros2 uses 42; keep them apart).
Hardware: Pixhawk 2.4.8 (ArduRover 4.7.2, MAVLink over USB), Raspberry Pi 4B (8 GB), Jetson Orin later.
Mongla reference: `~/tmp/mongla_ws` (ROS 2 Humble on dev boxes, Jazzy on Pi 5).
