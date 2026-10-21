# rupsha_ws handoff notes (for the next agent)

Last updated: 2026-10-21. Read this first, then `AGENTS.md`.

## Hosts and envs

| Host | Role | Access | Env |
|---|---|---|---|
| Laptop `arete` (RTX 2060 Mobile, 6 GB, driver 615) | main dev, ML, GPU | local | distrobox `rupsha_dev` (image `rupsha_dev:jazzy-cuda128`, Ubuntu 24.04, ROS 2 Jazzy, CUDA 12.8, cuDNN 9, torch cu128) |
| Raspberry Pi 4B+ `rupsha-pi` (aarch64, 8 GB, Ubuntu 24.04.5) | vehicle/companion, field tests | `ssh -i ~/.ssh/id_ed25519_rupsha_pi fh1m@192.168.0.102` (or `rupsha-pi.local` once mDNS works) | Docker 29.x, image `rupsha_pi:jazzy` (same ROS 2 Jazzy stack, no CUDA) |
| Pixhawk 2.4.8 | flight controller (ArduRover 4.7.2) | USB serial `/dev/serial/by-id/usb-ArduPilot_Pixhawk1*` | MAVLink via the `rupsha_control` manager (pymavlink); MAVROS is installed but not used |

## Box setup (rupsha_dev, verified 2026-10-21)

- Container HOME is `~/Envs/dockers/rupsha_dev` (distrobox `--home`). Workspace is linked at `~/tmp/rupsha_ws` inside the box.
- Shell: zsh, same sensei prompt and oh-my-zsh as auv-ros2 (symlinked from host `~/.config/zsh`, `~/.oh-my-zsh`). `.zshrc` written by `docker/setup_rupsha_home.sh`.
- ROS 2 Jazzy sourced, `ROS_DOMAIN_ID=43`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. auv-ros2 is Humble on domain 42. Do not mix.
- CUDA 12.8 on PATH (`CUDA_HOME=/usr/local/cuda-12.8`). Torch sees `sm_75`.
- Git identity from host `~/.gitconfig` (symlink). GitHub: `gh` logged in as `fh1m` (token copied from host via pipe, stored in box home). `gh auth setup-git` makes https git use it. SSH keys are NOT shared into the box.
- auv-ros2 box and its home were NOT modified. Do not touch them while they run.
- **Rebuilding the box wipes its layer** (`create_rupsha_dev.sh` runs `distrobox rm -f`): `/usr/local` OpenCV, venv `.pth`, apt tools. Home and workspace survive, and `gh` login is in the host keyring. After any rebuild run `docker/restore_box_after_rebuild.sh` inside the box. It reinstalls OpenCV 4.13 CUDA from the kept build tree (`third_party/opencv/build-cuda`) without recompiling. fzf/rg/fd/nvim/gh are now in the Dockerfile.

## Build files

- `docker/Dockerfile` (laptop, CUDA), `docker/Dockerfile.pi` (Pi, arm64 CPU-only).
- `docker/create_rupsha_dev.sh` builds the laptop image and creates the distrobox.
- `docker/init-hook.sh` recreates host GIDs (uucp 985, video 984, render 988, i2c 951) inside the container.
- Pi copy of the build context: `~/rupsha_docker/` on the Pi. Build log: `~/rupsha_docker/build-pi.log`.

## Known issues and fixes

- Ubuntu mirrors on this network: `archive.ubuntu.com` / `security.ubuntu.com` resolve to a local mirror (103.154.159.242) that is mid-sync. Builds fail with "File has unexpected size". Laptop Dockerfile now uses `ftp.jaist.ac.jp`. Pi uses `ports.ubuntu.com` directly.
- Pi host key changed after reflash. Fix was `ssh-keygen -R 192.168.0.102`. Only do this when you know the card was re-flashed.
- NTFS games drive `UUID=680578CA25BB3343` (`/mnt/fh1m`) is commented out in `/etc/fstab` on the laptop (backup in `~/claude-harness-backups/fstab-20261021/`). Re-enable when the drive is attached.
- Cursor hang on laptop was the stale `/mnt/fh1m` automount plus broken Cursor hooks/MCP. Hooks and MCP config were emptied (backup in `~/claude-harness-backups/cursor-20261021/`).
- `ros2-engineering` plugin PreToolUse hook blocks `dd` to block devices. Flash from a user terminal, not from the agent.
- Sudo on the laptop is locked after failed attempts (`faillock --user fh1m`). Do not guess passwords. Ask the user.

## Pi state (2026-10-21)

- Docker running, `fh1m` in docker group.
- CPU governor `performance` (`cpu-performance.service`).
- zram swap 256 MB (`zramswap`). Small. Raise `/etc/default/zramswap` if memory pressure.
- Clock: `arm_freq=1800`, `arm_boost=1` (stock). Original config backup: `/boot/firmware/config.txt.bak-20261021`.
- EEPROM update available (`sudo rpi-eeprom-update -a`). Not applied. Needs reboot.
- Overclock: see the section below for status and procedure.

## Pi runtime (set up 2026-10-21)

- Launcher on Pi: `~/rupsha_ws/run_rupsha_pi.sh` (source: `docker/run_rupsha_pi.sh`). Runs `rupsha_pi:jazzy` privileged, host network/IPC/PID, `/dev` mounted, workspace at `~/tmp/rupsha_ws` inside the container, user uid 1000 with hardware GIDs.
- udev: `/etc/udev/rules.d/99-rupsha-pixhawk.rules` gives `/dev/pixhawk` for the Pixhawk (ArduPilot firmware enumerates as `1209:5741`, verified with `lsusb`) and stops ModemManager from grabbing it.
- Cooling: stock heatsink reached 75.9 C at 1.8 GHz under load. Get a fan/active case before overclocking. Target: under 70 C sustained.
- Pi build log: `~/rupsha_docker/build-pi.log`.

## Overclock procedure (Pi 4B)

1. Measure baseline: `stress-ng --cpu 4 --cpu-method matrixprod --timeout 120s --metrics-brief`, then `vcgencmd measure_temp`, `vcgencmd get_throttled`, `vcgencmd measure_clock arm`.
2. Change `/boot/firmware/config.txt`: `arm_freq=2000` (or lower if unstable), remove or adjust `over_voltage_avs`, then reboot.
3. Verify: 10+ min `stress-ng --cpu 4 --verify`. Pass means no `stress-ng: fail` lines. `get_throttled` must stay `0x0` (bit 0x50000 = throttled since boot). Temp under 80 C.
4. Revert on failure: restore `/boot/firmware/config.txt.bak-20261021`, reboot.

Never raise voltage beyond the firmware default without verified cooling. Keep the backup.

## Next steps

- Laptop: verify `rupsha_dev` (`nvidia-smi`, torch `sm_75` in arch list, `ros2 run demo_nodes_cpp talker`, `ls /dev/ttyACM*` after plugging Pixhawk).
- Pi: finish `rupsha_pi:jazzy` build, then run it with `--privileged --network host --ipc host` and the serial/i2c/spi/gpio devices.
- Keep ROS_DOMAIN_ID=43 for rupsha (auv-ros2 uses 42 on the laptop).
- Cursor Remote-SSH to the Pi uses the same key.
