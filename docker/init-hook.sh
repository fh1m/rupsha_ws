#!/usr/bin/env bash
# Runs as root inside the container once at distrobox create time.
# Recreates the host's hardware-access groups with the SAME GIDs so the
# container user can open /dev/ttyACM* (Pixhawk), /dev/video*, /dev/dri, /dev/i2c*.
# Host GIDs (CachyOS): uucp 985, video 984, render 988, i2c 951.
set -u
ensure_group() {
  local name="$1" gid="$2"
  # Free the GID if an Ubuntu group already uses it under another name
  local existing
  existing=$(getent group "$gid" | cut -d: -f1)
  if [ -n "$existing" ] && [ "$existing" != "$name" ]; then
    groupmod -n "${existing}_old" "$existing" 2>/dev/null || true
  fi
  if getent group "$name" >/dev/null; then
    groupmod -g "$gid" "$name" 2>/dev/null || true
  else
    groupadd -g "$gid" "$name"
  fi
}
ensure_group uucp 985
ensure_group video 984
ensure_group render 988
ensure_group i2c 951
for u in $(getent passwd | awk -F: '$3>=1000 && $3<65534 {print $1}'); do
  usermod -aG uucp,video,render,i2c "$u"
done
