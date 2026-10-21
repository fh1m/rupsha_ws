#!/usr/bin/env bash
# Pi-in-the-loop: the real stack on the Pi, ArduRover SITL on this machine.
#
#   tools/pil.sh                       # square_drive against 192.168.0.102
#   PI=rupsha-pi.local MISSION=square_drive tools/pil.sh
#
# 1. sync the workspace to the Pi and build it in rupsha_pi:jazzy
# 2. start the manager on the Pi listening on tcpin:0.0.0.0:5760
# 3. start SITL here dialling the Pi, with a SIMSTATE truth recorder
# 4. run the mission on the Pi through /rupsha/move, then score it against truth
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PI="${PI:-192.168.0.102}"
PI_USER="${PI_USER:-fh1m}"
KEY="${KEY:-$HOME/.ssh/id_ed25519_rupsha_pi}"
MISSION="${MISSION:-square_drive}"
OUT="${OUT:-$WS/logs/pil_$(date +%Y%m%d_%H%M%S)}"
PY=python3; [ -x /opt/rupsha-venv/bin/python3 ] && PY=/opt/rupsha-venv/bin/python3
SSH=(ssh -i "$KEY" -o BatchMode=yes -o ConnectTimeout=5 "$PI_USER@$PI")
# one line: it is spliced into a remote command, where a newline ends `docker run`
OPTS='--network host --ipc host -u $(id -u):$(id -g) -e HOME=$HOME -e ROS_DOMAIN_ID=43 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
OPTS+=' -v $HOME:$HOME -v $HOME/rupsha_ws:$HOME/tmp/rupsha_ws -w $HOME/tmp/rupsha_ws'
IMG='rupsha_pi:jazzy bash -lc'
ROS='source /opt/ros/jazzy/setup.bash && source install/setup.bash'
mkdir -p "$OUT"

echo "[PIL] sync + build on $PI"
rsync -a --exclude build --exclude install --exclude log --exclude logs --exclude src/external \
    --exclude third_party --exclude __pycache__ --exclude '*.jpg' -e "ssh -i $KEY -o BatchMode=yes" \
    "$WS/" "$PI_USER@$PI:rupsha_ws/"
"${SSH[@]}" "cd ~/rupsha_ws && docker run --rm $OPTS $IMG './build_rupsha.sh' | tail -1"

cleanup() {
    touch /tmp/rupsha_pil.stop
    "${SSH[@]}" 'docker rm -f rupsha_pil >/dev/null 2>&1' || true
    [ -n "${SERVE:-}" ] && wait "$SERVE" 2>/dev/null || true
}
trap cleanup EXIT

echo "[PIL] manager on the Pi"
"${SSH[@]}" "cd ~/rupsha_ws && docker rm -f rupsha_pil >/dev/null 2>&1; docker run -d --name rupsha_pil $OPTS $IMG \
    '$ROS && ros2 launch rupsha_control bringup.launch.py url:=tcpin:0.0.0.0:5760' >/dev/null;
    for i in \$(seq 60); do ss -ltn | grep -q ':5760 ' && exit 0; sleep 1; done; exit 1"

echo "[PIL] SITL here, dialling the Pi"
(cd "$WS/src/rupsha_sim" && PYTHONPATH="$WS/src/rupsha_sim:$WS/src/rupsha_control:$PYTHONPATH" \
    "$PY" -m rupsha_sim.pil serve --pi "$PI" --out "$OUT/truth.jsonl" --log-dir "$OUT/sitl" \
    > "$OUT/serve.log" 2>&1) &
SERVE=$!
for i in $(seq 60); do
    grep -q 'truth recorder up' "$OUT/serve.log" 2>/dev/null && break
    kill -0 "$SERVE" 2>/dev/null || { echo "[PIL] SITL side died:"; tail -5 "$OUT/serve.log"; exit 1; }
    sleep 1
done

echo "[PIL] waiting for EKF, then $MISSION on the Pi"
"${SSH[@]}" "docker exec rupsha_pil bash -lc '$ROS && for i in \$(seq 60); do
    timeout 4 ros2 topic echo --once /rupsha/state 2>/dev/null | grep -q \"ekf_ok: true\" && break; sleep 2; done;
    ros2 run rupsha_planner mission $MISSION'" | tee "$OUT/mission.log"
STATUS=${PIPESTATUS[0]}
"${SSH[@]}" 'docker stats --no-stream --format "{{.Name}} cpu {{.CPUPerc}} mem {{.MemUsage}}" rupsha_pil' \
    | tee "$OUT/pi_load.txt"

touch /tmp/rupsha_pil.stop; wait "$SERVE" || true
SCORE=$(PYTHONPATH="$WS/src/rupsha_sim:$WS/src/rupsha_control" "$PY" -m rupsha_sim.pil score "$OUT/truth.jsonl")
echo "$SCORE" | tee "$OUT/score.json"
echo "[PIL] mission exit $STATUS; logs in $OUT"
exit "$STATUS"
