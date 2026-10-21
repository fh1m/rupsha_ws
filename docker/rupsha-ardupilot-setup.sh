#!/usr/bin/env bash
# Clone ArduPilot (if missing) and install its build prereqs inside rupsha_dev.
# Run inside the container:  rupsha-ardupilot-setup [~/tmp/ardupilot]
set -euo pipefail
DEST="${1:-$HOME/tmp/ardupilot}"
if [ ! -d "$DEST/.git" ]; then
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$DEST"
fi
cd "$DEST"
Tools/environment_install/install-prereqs-ubuntu.sh -y
echo "ArduPilot prereqs installed in $DEST. Rover build: ./waf configure --board fmuv2 && ./waf rover"
