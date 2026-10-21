#!/usr/bin/env bash
# Set up the rupsha_dev home (run INSIDE the box): same zsh prompt and oh-my-zsh as auv-ros2,
# ROS 2 Jazzy + CUDA on PATH, git identity shared with the host. Read-only towards the host
# except oh-my-zsh cache. Does NOT touch the auv-ros2 box or its home.
# Idempotent: re-run after edits.
set -euo pipefail
HOST_HOME=/home/fh1m
H="$HOME"
mkdir -p "$H/.config"

# Shared with host (single source of truth)
ln -sfn "$HOST_HOME/.oh-my-zsh"          "$H/.oh-my-zsh"
ln -sfn "$HOST_HOME/.config/zsh"         "$H/.config/zsh"
ln -sfn "$HOST_HOME/.gitconfig"          "$H/.gitconfig"
[ -d "$HOST_HOME/.config/fzf" ] && ln -sfn "$HOST_HOME/.config/fzf" "$H/.config/fzf"

# Workspace path (container HOME is ~/Envs/dockers/rupsha_dev, so link the real workspace)
mkdir -p "$H/tmp"
ln -sfn "$HOST_HOME/tmp/rupsha_ws" "$H/tmp/rupsha_ws"

cat > "$H/.zshrc" <<'ZSHRC'
# rupsha_dev zsh. Mirrors auv-ros2 (oh-my-zsh + sensei prompt); ROS 2 JAZZY (auv-ros2 is Humble).
export ZSH="$HOME/.oh-my-zsh"
ZSH_THEME=""
DISABLE_UNTRACKED_FILES_DIRTY="true"
plugins=(git sudo fzf fzf-tab zsh-lazyload zsh-autosuggestions zsh-syntax-highlighting shrink-path)
source $ZSH/oh-my-zsh.sh

export LANG=en_US.UTF-8 EDITOR=nvim KEYTIMEOUT=1
export FZF_DEFAULT_COMMAND='fdfind --type file --hidden --exclude .git'

# CUDA 12.8 (RTX 2060, sm_75)
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# Python venv: torch cu128, ultralytics, pymavlink, MAVProxy
[ -f /opt/rupsha-venv/bin/activate ] && source /opt/rupsha-venv/bin/activate

# ROS 2 Jazzy. Domain 43 (auv-ros2 uses 42 on the same network).
source /opt/ros/jazzy/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=43 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
[ -f "$HOME/tmp/rupsha_ws/install/setup.zsh" ] && source "$HOME/tmp/rupsha_ws/install/setup.zsh"

# Prompt: shared sensei prompt from the host (shows box, py env, ROS, CUDA, git, exit, duration)
[ -f ~/.config/zsh/prompt.zsh ] && source ~/.config/zsh/prompt.zsh

alias ls='ls --color=auto'
alias ws='cd ~/tmp/rupsha_ws'
alias rb='colcon build --symlink-install'
alias rs='source install/setup.zsh'
alias gpu='nvidia-smi'
alias cams='v4l2-ctl --list-devices'
alias mav='mavproxy.py --master=/dev/pixhawk --baudrate 115200'
alias py=python3
alias reload='exec zsh'
ZSHRC

# gh + git: gh credential helper for https; identity comes from the shared ~/.gitconfig
if command -v gh >/dev/null 2>&1; then
  gh auth setup-git 2>/dev/null || true
fi

echo "rupsha_dev home ready. Open a new shell: exec zsh"
