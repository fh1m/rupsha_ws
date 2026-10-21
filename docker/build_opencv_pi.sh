#!/usr/bin/env bash
# OpenCV 4.13.0 for Raspberry Pi 4 (aarch64, Cortex-A72): NEON, TBB, GStreamer, FFmpeg, V4L2. CPU only.
# Run ON the Pi host. Runs at nice 19 so it yields to other work. Log: ~/opencv-build.log
set -euo pipefail
VER=4.13.0
SRC="$HOME/opencv-src"
mkdir -p "$SRC" && cd "$SRC"
[ -d opencv ] || git clone --depth 1 --branch "$VER" https://github.com/opencv/opencv.git
[ -d opencv_contrib ] || git clone --depth 1 --branch "$VER" https://github.com/opencv/opencv_contrib.git
mkdir -p build && cd build
ARCH_FLAGS="-O3 -mcpu=cortex-a72+crc -mtune=cortex-a72"
nice -n 19 cmake "$SRC/opencv" -G Ninja \
  -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D CMAKE_C_FLAGS="$ARCH_FLAGS" -D CMAKE_CXX_FLAGS="$ARCH_FLAGS" \
  -D ENABLE_NEON=ON \
  -D OPENCV_EXTRA_MODULES_PATH="$SRC/opencv_contrib/modules" \
  -D OPENCV_ENABLE_NONFREE=ON -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D WITH_CUDA=OFF -D WITH_GTK=OFF -D WITH_QT=OFF -D WITH_OPENGL=OFF \
  -D WITH_TBB=ON -D WITH_GSTREAMER=ON -D WITH_FFMPEG=ON -D WITH_V4L=ON \
  -D BUILD_opencv_python3=ON -D PYTHON3_EXECUTABLE=/usr/bin/python3 \
  -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_DOCS=OFF -D BUILD_opencv_apps=OFF
nice -n 19 ninja -j3
sudo ninja install
sudo ldconfig
python3 -c "import cv2; print('cv2', cv2.__version__); print(cv2.getBuildInformation())" | grep -E "^cv2|NEON|TBB|GStreamer|FFMPEG|Baseline"
echo OPENCV_BUILD_DONE
