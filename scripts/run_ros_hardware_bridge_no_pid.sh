#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

PORT="${1:-/dev/ttyACM0}"

# Keep the servo's current EEPROM PID values. This wrapper only enables torque
# before launching the bridge, then prevents the bridge from configuring motors
# on connect. The speed/acceleration values below are sent as WritePosEx command
# parameters; they are not EEPROM PID changes.
COMMAND_RATE_HZ="${SO101_COMMAND_RATE_HZ:-80.0}"
PUBLISH_RATE_HZ="${SO101_PUBLISH_RATE_HZ:-50.0}"
FEEDBACK_READ_RATE_HZ="${SO101_FEEDBACK_READ_RATE_HZ:-25.0}"
TELEMETRY_RATE_HZ="${SO101_TELEMETRY_RATE_HZ:-2.0}"
TRAJECTORY_INTERPOLATION="${SO101_TRAJECTORY_INTERPOLATION:-cubic}"
LPF_ALPHA="${SO101_LPF_ALPHA:-1.0}"
COMMAND_DEADBAND_RAD="${SO101_COMMAND_DEADBAND_RAD:-0.0}"
SERVO_SPEED="${SO101_SERVO_SPEED:-500}"
SERVO_ACCELERATION="${SO101_SERVO_ACCELERATION:-25}"

so101_sanitize_ros_python
so101_check_serial_port "$PORT"

echo "[SO101] Enabling torque only; not writing PID registers"
"$SO101_ROOT/scripts/so101_servo_status.py" "$PORT" --ids 1-6 --torque on

so101_source_ros
cd "$SO101_ROOT/ros1_ws"
catkin_make
source devel/setup.bash

export ROS_HOME="${ROS_HOME:-/tmp/so101_ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/so101_ros_logs}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

echo "[SO101] Starting hardware bridge with current servo PID values:"
echo "  port:                     $PORT"
echo "  command_rate_hz:          $COMMAND_RATE_HZ"
echo "  trajectory_interpolation: $TRAJECTORY_INTERPOLATION"
echo "  command_lpf_alpha:        $LPF_ALPHA"
echo "  servo_speed:              $SERVO_SPEED"
echo "  servo_acceleration:       $SERVO_ACCELERATION"
echo "  configure_motors_on_connect: false"
echo "  feedback_position_assist: disabled"
echo "  backlash_compensation: disabled"

roslaunch so101_ros1_bridge hardware_bridge.launch \
  port:="$PORT" \
  command_rate_hz:="$COMMAND_RATE_HZ" \
  publish_rate_hz:="$PUBLISH_RATE_HZ" \
  feedback_read_rate_hz:="$FEEDBACK_READ_RATE_HZ" \
  telemetry_rate_hz:="$TELEMETRY_RATE_HZ" \
  trajectory_interpolation:="$TRAJECTORY_INTERPOLATION" \
  command_lpf_alpha:="$LPF_ALPHA" \
  command_deadband_rad:="$COMMAND_DEADBAND_RAD" \
  servo_speed:="$SERVO_SPEED" \
  servo_acceleration:="$SERVO_ACCELERATION" \
  configure_motors_on_connect:=false \
  feedback_position_assist_gain:=0.0 \
  feedback_position_assist_integral_gain:=0.0 \
  backlash_compensation_enabled:=false \
  with_servo:=false \
  with_trajectory_action:=false
