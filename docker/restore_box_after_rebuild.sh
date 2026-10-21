#!/usr/bin/env bash
# Run INSIDE rupsha_dev after any rebuild of the box (create_rupsha_dev.sh does `distrobox rm -f`,
# which wipes the box layer: /usr/local installs, venv .pth fix). Home and workspace survive.
# Reinstalls OpenCV 4.13 CUDA from the KEPT build folder (no recompile), routes the venv to it,
# checks gh. Idempotent.
set -euo pipefail
W="$HOME/tmp/rupsha_ws"
BUILD="$W/third_party/opencv/build-cuda"

if [ ! -f "$BUILD/build.ninja" ]; then
  echo "No OpenCV build folder at $BUILD. Rebuild with: $W/docker/build_opencv.sh (about an hour)." >&2
  exit 1
fi

# 1. OpenCV 4.13 CUDA: install from the kept build tree (ninja only re-links/copies)
( cd "$BUILD" && sudo ninja install )
sudo ldconfig

# 2. Venv must import this build, not the pip wheel (pip wheel is in the image layer)
sudo /opt/rupsha-venv/bin/pip uninstall -y opencv-python >/dev/null 2>&1 || true
echo /usr/local/lib/python3.12/site-packages | sudo tee /opt/rupsha-venv/lib/python3.12/site-packages/zz-opencv-cuda.pth >/dev/null

# 3. Check the result
cd /tmp
python3 -c "import cv2; print('cv2', cv2.__version__, 'cuda', cv2.cuda.getCudaEnabledDeviceCount())"
gh auth status 2>&1 | grep -E "Logged in" || echo "gh: not logged in (run: gh auth token | gh auth login --with-token on the host pipe)"
echo "restore done"
