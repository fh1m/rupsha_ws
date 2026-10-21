# rupsha_ws agent notes

**Start at [`CLAUDE.md`](CLAUDE.md)** — the vehicle, the board traps, the architecture rules and how
to run everything. The full document set, in reading order: [`.claude/context/README.md`](.claude/context/README.md).
Host and box history: [`NOTES.md`](NOTES.md).

Dev env: distrobox `rupsha_dev` (Ubuntu 24.04, ROS 2 Jazzy, CUDA 12.8, cuDNN 9, torch cu128).
Image: `rupsha_dev:jazzy-cuda128`, built from `docker/Dockerfile`. Recreate with `docker/create_rupsha_dev.sh`.

Run all build/test commands inside the container, as `fh1m` (never root):

    distrobox enter rupsha_dev -- bash -lc 'cd ~/tmp/rupsha_ws && ./build_rupsha.sh && pytest'

Env: ROS_DISTRO=jazzy, RMW=rmw_cyclonedds_cpp, ROS_DOMAIN_ID=43 (auv-ros2 uses 42; keep them apart).
Hardware: Pixhawk 2.4.8 (ArduRover 4.7.2, MAVLink over USB), Raspberry Pi 4B (8 GB), Jetson Orin later.
Mongla reference: `~/tmp/mongla_ws` (ROS 2 Humble on dev boxes, Jazzy on Pi 5).
