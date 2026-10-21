#!/usr/bin/env bash
# Run the rupsha_pi container on the Raspberry Pi with full hardware access.
# Usage (on the Pi):  ~/rupsha_ws/docker/run_rupsha_pi.sh [command...]
# Default command: interactive bash inside the container.
set -euo pipefail
IMG=rupsha_pi:jazzy
NAME=rupsha_pi
WS="$HOME/rupsha_ws"

mkdir -p "$WS"

# Host GIDs for hardware groups, passed into the container so the user can open devices
gids=()
for g in dialout i2c gpio spi video render plugdev docker; do
  gid=$(getent group "$g" | cut -d: -f3 || true)
  [ -n "$gid" ] && gids+=(--group-add "$gid")
done

docker rm -f "$NAME" >/dev/null 2>&1 || true
exec docker run -it --rm --name "$NAME" \
  --privileged --network host --ipc host --pid host \
  --shm-size=512m \
  -u "$(id -u):$(id -g)" \
  "${gids[@]}" \
  -e HOME="$HOME" \
  -e ROS_DOMAIN_ID=43 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /dev:/dev \
  -v /sys:/sys:ro \
  -v "$HOME":"$HOME" \
  -v "$WS":"$HOME/tmp/rupsha_ws" \
  -w "$HOME/tmp/rupsha_ws" \
  "$IMG" "${@:-bash}"
