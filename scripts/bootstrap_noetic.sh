#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

WITH_THIRD_PARTY=0
SKIP_PIP=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --with-third-party)
      WITH_THIRD_PARTY=1
      ;;
    --skip-pip)
      SKIP_PIP=1
      ;;
    -h|--help)
      cat <<EOF
Usage: scripts/bootstrap_noetic.sh [--with-third-party] [--skip-pip]

Builds the ROS Noetic workspace and installs Python dependencies for the SO101
ROS1 bridge. Use --with-third-party to clone the Seeed controller helper repo
used by calibration utilities.
EOF
      exit 0
      ;;
    *)
      echo "[SO101][ERROR] Unknown option: $1" >&2
      exit 2
      ;;
  esac
  shift
done

cd "$SO101_ROOT"

if [ ! -f "$(so101_ros_setup_path)" ]; then
  cat >&2 <<EOF
[SO101][ERROR] ROS Noetic was not found at $(so101_ros_setup_path)
Install ROS Noetic first on Ubuntu 20.04, then rerun this script.
EOF
  exit 1
fi

if [ "$WITH_THIRD_PARTY" -eq 1 ]; then
  mkdir -p "$SO101_ROOT/third_party"
  if [ ! -d "$SO101_ROOT/third_party/Seeed_RoboController" ]; then
    git clone https://github.com/Seeed-Projects/Seeed_RoboController.git \
      "$SO101_ROOT/third_party/Seeed_RoboController"
  else
    echo "[SO101] third_party/Seeed_RoboController already exists"
  fi
fi

if [ "$SKIP_PIP" -eq 0 ]; then
  python3 -m pip install --user -r requirements-noetic.txt
fi

so101_sanitize_ros_python
so101_source_ros

cd "$SO101_ROOT/ros1_ws"
catkin_make

echo "[SO101] Bootstrap complete. Next commands:"
echo "  source $SO101_ROOT/ros1_ws/devel/setup.bash"
echo "  roslaunch so101_ros1_bridge mock_bridge.launch"
