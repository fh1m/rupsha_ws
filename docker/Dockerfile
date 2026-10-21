# rupsha_dev -- Ubuntu 24.04 + ROS 2 Jazzy + CUDA 12.8 (cuDNN 9) dev image
# Matches mongla_ws vehicle side (Pi 5 / Jazzy) and the auv-ros2 CUDA 12.8 stack.
FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=43 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1 \
    QT_QPA_PLATFORM=xcb \
    VIRTUAL_ENV=/opt/rupsha-venv \
    PATH=/opt/rupsha-venv/bin:$PATH

# Free UID/GID 1000 for the host user (distrobox creates the real user at enter time)
RUN (userdel -r ubuntu || true) && (groupdel ubuntu || true)

# apt retries: Ubuntu mirrors occasionally serve half-synced indexes
RUN printf 'Acquire::Retries "10";\nAcquire::http::Timeout "30";\nAcquire::https::Timeout "30";\n' > /etc/apt/apt.conf.d/80-retries \
  && sed -i 's|http://archive.ubuntu.com/ubuntu|http://ftp.jaist.ac.jp/pub/Linux/ubuntu|g; s|http://security.ubuntu.com/ubuntu|http://ftp.jaist.ac.jp/pub/Linux/ubuntu|g' /etc/apt/sources.list.d/ubuntu.sources

# Base tools, universe repo, locale, ROS 2 apt key and source
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg locales software-properties-common \
  && locale-gen en_US.UTF-8 \
  && add-apt-repository -y universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list \
  && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy desktop + autonomy/vehicle stack + build/dev tools + hardware + GPU/video tools
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-desktop \
      ros-jazzy-ros-base \
      ros-dev-tools \
      python3-colcon-common-extensions \
      python3-rosdep python3-vcstool python3-venv python3-pip python3-opencv \
      ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cyclonedds \
      ros-jazzy-mavros ros-jazzy-mavros-extras \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-robot-localization \
      ros-jazzy-ros-gz \
      ros-jazzy-cv-bridge ros-jazzy-image-transport-plugins \
      ros-jazzy-rtabmap-ros \
      ros-jazzy-joy ros-jazzy-teleop-twist-joy \
      ros-jazzy-diagnostic-updater ros-jazzy-tf2-tools \
      gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-gl \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      v4l-utils usbutils pciutils can-utils minicom screen picocom \
      udev i2c-tools libi2c-dev \
      mesa-utils mesa-utils-extra vulkan-tools \
      git git-lfs cmake ninja-build ccache gdb valgrind \
      libeigen3-dev libyaml-cpp-dev libboost-all-dev \
  && rosdep init 2>/dev/null || true \
  && rm -rf /var/lib/apt/lists/*

# Python: torch/torchvision CUDA 12.8 wheels, ML + MAVLink tools, in a venv that sees ROS's python packages
RUN python3 -m venv --system-site-packages ${VIRTUAL_ENV} \
  && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir --upgrade pip \
  && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cu128 \
  && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir \
      ultralytics onnx onnxruntime-gpu \
      pymavlink MAVProxy pyserial \
      numpy scipy filterpy matplotlib pandas pyyaml \
      pytest ruff

# MAVROS needs geographiclib datasets at runtime or it crashes on start
RUN /opt/ros/jazzy/share/mavros/install_geographiclib_datasets.sh || true

# ArduPilot build prereqs helper (run by the user later; clones ArduPilot only on demand)
COPY rupsha-ardupilot-setup.sh /usr/local/bin/rupsha-ardupilot-setup
RUN chmod +x /usr/local/bin/rupsha-ardupilot-setup

# Shell environment for every login/interactive shell
RUN printf '%s\n' \
  'source /opt/ros/jazzy/setup.bash' \
  '[ -f "$HOME/tmp/rupsha_ws/install/setup.bash" ] && source "$HOME/tmp/rupsha_ws/install/setup.bash"' \
  'export VIRTUAL_ENV=/opt/rupsha-venv' \
  'export PATH=/opt/rupsha-venv/bin:$PATH' \
  'export __NV_PRIME_RENDER_OFFLOAD=1' \
  'export __GLX_VENDOR_LIBRARY_NAME=nvidia' \
  > /etc/profile.d/rupsha.sh

# Project must-haves: SLAM/localization/control/sensors/geometry/optimisation (rover + AUV autonomy)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-slam-toolbox ros-jazzy-rtabmap-ros ros-jazzy-octomap-ros \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-foxglove-bridge ros-jazzy-rosbridge-server \
      ros-jazzy-image-pipeline ros-jazzy-vision-opencv ros-jazzy-pcl-ros \
      ros-jazzy-geographic-msgs ros-jazzy-robot-localization ros-jazzy-imu-filter-madgwick \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      libgtsam-dev libceres-dev libgeographiclib-dev geographiclib-tools \
      libpcl-dev libsuitesparse-dev libfmt-dev libspdlog-dev libzmq3-dev \
      libtbb-dev libjsoncpp-dev libproj-dev libgdal-dev libhdf5-dev \
      libopenblas-dev liblapack-dev libavformat-dev libswscale-dev libv4l-dev \
      libprotobuf-dev protobuf-compiler libgtk-3-dev \
      fzf ripgrep fd-find neovim \
  && rm -rf /var/lib/apt/lists/*

# GitHub CLI (gh) from the official apt repo
RUN mkdir -p /etc/apt/keyrings \
  && curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg -o /etc/apt/keyrings/githubcli-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" > /etc/apt/sources.list.d/github-cli.list \
  && apt-get update && apt-get install -y --no-install-recommends gh \
  && rm -rf /var/lib/apt/lists/*

RUN ${VIRTUAL_ENV}/bin/pip install --no-cache-dir scikit-image shapely pyproj open3d

CMD ["bash"]
