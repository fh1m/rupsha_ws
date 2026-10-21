#!/usr/bin/env bash
# Build rupsha_ws. Same command on the laptop box, the Pi and CI.
#
#   ./build_rupsha.sh                       # everything
#   ./build_rupsha.sh --packages-select rupsha_control
#
# colcon runs under the rupsha venv's python when it exists, so console-script
# shebangs point at the interpreter that has pymavlink (system python3 does not).
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash
export GZ_VERSION=harmonic

# The Gazebo plugin is only built where the simulator runs. The vehicle
# computers (Pi, later Jetson) are aarch64 and never host the sim.
SKIP=()
if [ -z "${RUPSHA_NO_SIM:-}" ] && [ "$(uname -m)" = aarch64 ]; then RUPSHA_NO_SIM=1; fi
if [ -n "${RUPSHA_NO_SIM:-}" ]; then
    SKIP=(--packages-skip ardupilot_gazebo)
elif [ ! -d src/external/ardupilot_gazebo ]; then
    mkdir -p src/external
    vcs import src/external < rupsha.repos
fi

PY=python3
[ -x /opt/rupsha-venv/bin/python3 ] && PY=/opt/rupsha-venv/bin/python3

"$PY" tools/gen_manifest.py --check || {
    echo "manifest outputs are stale: run $PY tools/gen_manifest.py" >&2
    exit 1
}

"$PY" -m colcon build --base-paths src --symlink-install "${SKIP[@]}" \
    "$@" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF

echo "built. source $WS/install/setup.bash"
