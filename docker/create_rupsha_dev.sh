#!/usr/bin/env bash
# Build the image and create the distrobox rupsha_dev. Re-run safe.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
IMG=rupsha_dev:jazzy-cuda128
NAME=rupsha_dev
HOME_DIR="$HOME/Envs/dockers/rupsha_dev"

mkdir -p "$HOME_DIR"
docker build -t "$IMG" "$HERE"

distrobox rm -f "$NAME" 2>/dev/null || true
distrobox create --yes --name "$NAME" --image "$IMG" \
  --nvidia \
  --home "$HOME_DIR" \
  --init-hooks "bash $HERE/init-hook.sh" \
  --additional-flags "--privileged --ipc host --shm-size=4g --device-cgroup-rule='c 166:* rwm' --device-cgroup-rule='c 188:* rwm' --device-cgroup-rule='c 189:* rwm' --device-cgroup-rule='c 81:* rwm' --device-cgroup-rule='c 226:* rwm'"

echo "Created. Enter: distrobox enter $NAME"
