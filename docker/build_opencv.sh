#!/usr/bin/env bash
# Build OpenCV 4.13.0 (+ opencv_contrib) from source, inside the dev container.
# Laptop (rupsha_dev):  ./build_opencv.sh            -> CUDA 12.8, sm_75 (RTX 2060), cuDNN, GStreamer, FFmpeg
# Pi (rupsha_pi):       ./build_opencv.sh --cpu      -> CPU/NEON only (aarch64), GStreamer, FFmpeg
# Installs to /usr/local. Python bindings go to the default python3 dist-packages, which the venv sees.
# Why 4.13.0 and not 5.0.0: cv_bridge / vision_opencv on ROS 2 Jazzy and ultralytics target the 4.x API.
set -euo pipefail

VER=4.13.0
CPU_ONLY=0
[ "${1:-}" = "--cpu" ] && CPU_ONLY=1

ROOT="$HOME/tmp/rupsha_ws/third_party/opencv"
mkdir -p "$ROOT" && cd "$ROOT"
[ -d opencv ] || git clone --depth 1 --branch "$VER" https://github.com/opencv/opencv.git
[ -d opencv_contrib ] || git clone --depth 1 --branch "$VER" https://github.com/opencv/opencv_contrib.git

BUILD="$ROOT/build-$([ $CPU_ONLY = 1 ] && echo cpu || echo cuda)"
mkdir -p "$BUILD" && cd "$BUILD"

CUDA_FLAGS=(-D WITH_CUDA=ON -D CUDA_ARCH_BIN=7.5 -D CUDA_ARCH_PTX="" -D WITH_CUDNN=ON
            -D OPENCV_DNN_CUDA=ON -D WITH_CUBLAS=ON -D ENABLE_FAST_MATH=ON -D CUDA_FAST_MATH=ON)
[ $CPU_ONLY = 1 ] && CUDA_FLAGS=(-D WITH_CUDA=OFF)

cmake "$ROOT/opencv" \
  -G Ninja \
  -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D OPENCV_EXTRA_MODULES_PATH="$ROOT/opencv_contrib/modules" \
  "${CUDA_FLAGS[@]}" \
  -D WITH_TBB=ON -D WITH_OPENMP=ON \
  -D WITH_GSTREAMER=ON -D WITH_FFMPEG=ON -D WITH_V4L=ON \
  -D WITH_GTK=ON -D WITH_OPENGL=ON \
  -D BUILD_opencv_python3=ON -D BUILD_opencv_python_bindings_generator=ON \
  -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_DOCS=OFF \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D OPENCV_ENABLE_NONFREE=ON

ninja -j"$(nproc)"
sudo ninja install
sudo ldconfig

python3 -c "import cv2; print('cv2', cv2.__version__); print('cuda devices:', cv2.cuda.getCudaEnabledDeviceCount() if hasattr(cv2,'cuda') else 'n/a')"
