#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

echo "SO101 root: $SO101_ROOT"
echo "Python: $(python3 --version)"
echo "ROS distro: ${ROS_DISTRO:-not sourced}"
if [ -f "$(so101_ros_setup_path)" ]; then
  echo "ROS setup: $(so101_ros_setup_path)"
else
  echo "ROS setup: missing ($(so101_ros_setup_path))"
fi
command -v roscore >/dev/null && echo "roscore: $(command -v roscore)"
command -v catkin_make >/dev/null && echo "catkin_make: $(command -v catkin_make)"
python3 - <<'PY'
try:
    import scservo_sdk
    print("scservo_sdk: installed")
except Exception as exc:
    print("scservo_sdk: missing (%s)" % exc)
try:
    import serial
    print("pyserial: installed")
except Exception as exc:
    print("pyserial: missing (%s)" % exc)
PY

echo "Candidate serial ports:"
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
