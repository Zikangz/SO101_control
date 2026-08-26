#!/usr/bin/env bash
# Shared helpers for SO101 shell scripts. Source this file; do not execute it.

SO101_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SO101_ROOT="${SO101_ROOT:-$(cd "${SO101_COMMON_DIR}/.." && pwd)}"
export SO101_ROS_DISTRO="${SO101_ROS_DISTRO:-noetic}"

so101_ros_setup_path() {
  printf '%s\n' "${ROS_SETUP:-/opt/ros/${SO101_ROS_DISTRO}/setup.bash}"
}

so101_source_ros() {
  local setup
  setup="$(so101_ros_setup_path)"
  if [ ! -f "$setup" ]; then
    echo "[SO101][ERROR] ROS setup not found: $setup" >&2
    echo "[SO101] Install ROS ${SO101_ROS_DISTRO} or set ROS_SETUP=/path/to/setup.bash" >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "$setup"
}

so101_source_ws() {
  local setup="$SO101_ROOT/ros1_ws/devel/setup.bash"
  if [ ! -f "$setup" ]; then
    echo "[SO101][ERROR] Workspace is not built: $setup" >&2
    echo "[SO101] Run: scripts/bootstrap_noetic.sh" >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "$setup"
}

so101_python() {
  if [ -n "${SO101_PYTHON:-}" ]; then
    printf '%s\n' "$SO101_PYTHON"
  elif [ -x "$SO101_ROOT/.conda-so101-noetic/bin/python" ]; then
    printf '%s\n' "$SO101_ROOT/.conda-so101-noetic/bin/python"
  else
    command -v python3
  fi
}

so101_prepare_third_party_python() {
  local env_dir="$SO101_ROOT/.conda-so101-noetic"
  export PYTHONNOUSERSITE=1
  export PYTHONPATH=
  if [ -d "$env_dir" ]; then
    export LD_LIBRARY_PATH="$env_dir/lib:${LD_LIBRARY_PATH:-}"
    if [ -d "$env_dir/lib/python3.8/site-packages/PySide6/Qt/plugins" ]; then
      export QT_PLUGIN_PATH="$env_dir/lib/python3.8/site-packages/PySide6/Qt/plugins"
    fi
  fi
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
}

so101_sanitize_ros_python() {
  if command -v conda >/dev/null 2>&1; then
    conda deactivate >/dev/null 2>&1 || true
  fi
  unset PYTHONHOME
  unset PYTHONPATH
  export PYTHONNOUSERSITE=1
  export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/opt/ros/${SO101_ROS_DISTRO}/bin:${PATH}"
}

so101_require_seeed_repo() {
  local repo="$SO101_ROOT/third_party/Seeed_RoboController"
  if [ ! -d "$repo" ]; then
    echo "[SO101][ERROR] Missing third-party Seeed controller repo: $repo" >&2
    echo "[SO101] Run: scripts/bootstrap_noetic.sh --with-third-party" >&2
    return 1
  fi
}

so101_check_serial_port() {
  local port="$1"
  if [ ! -e "$port" ]; then
    echo "[SO101][ERROR] Serial port does not exist: $port" >&2
    echo "[SO101] Available serial ports:" >&2
    ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  none" >&2
    if [ -d /dev/serial/by-id ]; then
      echo "[SO101] Stable serial aliases:" >&2
      ls -l /dev/serial/by-id 2>/dev/null || true
    fi
    return 2
  fi
  if [ ! -r "$port" ] || [ ! -w "$port" ]; then
    echo "[SO101][ERROR] No read/write permission for $port" >&2
    echo "[SO101] Temporary fix: sudo chmod 666 $port" >&2
    echo "[SO101] Permanent fix: sudo usermod -aG dialout \$USER ; then log out/in" >&2
    return 3
  fi
  if command -v fuser >/dev/null 2>&1 && fuser "$port" >/dev/null 2>&1; then
    echo "[SO101][ERROR] Serial port is already in use: $port" >&2
    fuser -v "$port" >&2 || true
    return 4
  fi
}
