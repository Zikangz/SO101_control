#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

PORT="${1:-/dev/ttyACM0}"

# ROS Noetic should run under system Python. A Conda environment can make xacro
# use the wrong interpreter and fail with "No module named rospkg".
so101_sanitize_ros_python
set +e
so101_check_serial_port "$PORT"
rc=$?
set -e
if [ "$rc" -ne 0 ]; then
  echo "[SO101] Reconnect the SO101 controller, power the follower, then run:" >&2
  echo "  dmesg -w | grep -iE 'tty|usb|cdc|acm|ch34|cp210'" >&2
  echo "  scripts/run_ros_hardware_bridge.sh /dev/<actual-port>" >&2
  exit "$rc"
fi

so101_source_ros
cd "$SO101_ROOT/ros1_ws"
catkin_make
source devel/setup.bash

/usr/bin/python3 - <<'PY'
import rospkg
import rospy
import serial
from so101_ros1_bridge.hardware import FeetechSO101Backend

backend = FeetechSO101Backend("/dev/null", 1000000, ["shoulder_pan"], {"shoulder_pan": 1}, {"shoulder_pan": {}}, True)
scservo_sdk = backend._import_scservo_sdk()
print("[SO101] ROS Python environment OK")
print("[SO101] scservo_sdk:", scservo_sdk.__file__)
PY

roslaunch so101_ros1_bridge hardware_bridge.launch port:="$PORT"
