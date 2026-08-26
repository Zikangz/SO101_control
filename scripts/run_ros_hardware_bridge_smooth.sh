#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

PORT="${1:-/dev/ttyACM0}"

# Baseline profile for real hardware trajectory diagnostics. The timed cubic
# trajectory and the STS3215 position loop already smooth the command; an
# outer low-pass filter adds phase lag and must not be enabled by default.
COMMAND_RATE_HZ="${SO101_COMMAND_RATE_HZ:-80.0}"
PUBLISH_RATE_HZ="${SO101_PUBLISH_RATE_HZ:-50.0}"
FEEDBACK_READ_RATE_HZ="${SO101_FEEDBACK_READ_RATE_HZ:-25.0}"
TELEMETRY_RATE_HZ="${SO101_TELEMETRY_RATE_HZ:-2.0}"
TRAJECTORY_INTERPOLATION="${SO101_TRAJECTORY_INTERPOLATION:-cubic}"
LPF_ALPHA="${SO101_LPF_ALPHA:-1.0}"
COMMAND_DEADBAND_RAD="${SO101_COMMAND_DEADBAND_RAD:-0.0}"
FEEDBACK_POSITION_ASSIST_GAIN="${SO101_FEEDBACK_POSITION_ASSIST_GAIN:-0.0}"
FEEDBACK_POSITION_ASSIST_INTEGRAL_GAIN="${SO101_FEEDBACK_POSITION_ASSIST_INTEGRAL_GAIN:-0.0}"
FEEDBACK_POSITION_ASSIST_MAX_OFFSET_RAD="${SO101_FEEDBACK_POSITION_ASSIST_MAX_OFFSET_RAD:-0.04}"
FEEDBACK_POSITION_ASSIST_LIMIT_MARGIN_RAD="${SO101_FEEDBACK_POSITION_ASSIST_LIMIT_MARGIN_RAD:-0.03}"
FEEDBACK_POSITION_ASSIST_ALPHA="${SO101_FEEDBACK_POSITION_ASSIST_ALPHA:-0.15}"
FEEDBACK_POSITION_ASSIST_MAX_FEEDBACK_AGE_S="${SO101_FEEDBACK_POSITION_ASSIST_MAX_FEEDBACK_AGE_S:-0.12}"
FEEDBACK_POSITION_ASSIST_TRAJECTORY_ONLY="${SO101_FEEDBACK_POSITION_ASSIST_TRAJECTORY_ONLY:-true}"
BACKLASH_COMPENSATION_ENABLED="${SO101_BACKLASH_COMPENSATION_ENABLED:-false}"
BACKLASH_COMPENSATION_PROFILE="${SO101_BACKLASH_COMPENSATION_PROFILE:-}"
BACKLASH_COMPENSATION_TRAJECTORY_ONLY="${SO101_BACKLASH_COMPENSATION_TRAJECTORY_ONLY:-true}"
BACKLASH_COMPENSATION_MAX_ABS_BIAS_RAD="${SO101_BACKLASH_COMPENSATION_MAX_ABS_BIAS_RAD:-}"
BACKLASH_COMPENSATION_BIAS_SLEW_RAD_S="${SO101_BACKLASH_COMPENSATION_BIAS_SLEW_RAD_S:-}"
BACKLASH_COMPENSATION_VELOCITY_THRESHOLD_RAD_S="${SO101_BACKLASH_COMPENSATION_VELOCITY_THRESHOLD_RAD_S:-}"
BACKLASH_COMPENSATION_POSITION_HYSTERESIS_RAD="${SO101_BACKLASH_COMPENSATION_POSITION_HYSTERESIS_RAD:-}"
BACKLASH_COMPENSATION_LIMIT_MARGIN_RAD="${SO101_BACKLASH_COMPENSATION_LIMIT_MARGIN_RAD:-}"
CONFIGURE_MOTORS_ON_CONNECT="${SO101_CONFIGURE_MOTORS_ON_CONNECT:-true}"
CONFIGURATION_WRITE_ACK="${SO101_CONFIGURATION_WRITE_ACK:-true}"
SERVO_SPEED="${SO101_SERVO_SPEED:-500}"
SERVO_ACCELERATION="${SO101_SERVO_ACCELERATION:-25}"
SERVO_MAXIMUM_ACCELERATION="${SO101_SERVO_MAXIMUM_ACCELERATION:-254}"
SERVO_PID_P="${SO101_SERVO_PID_P:-16}"
SERVO_PID_I="${SO101_SERVO_PID_I:-0}"
SERVO_PID_D="${SO101_SERVO_PID_D:-32}"
SHOULDER_LIFT_PID_P="${SO101_SHOULDER_LIFT_PID_P:-32}"
SHOULDER_LIFT_PID_I="${SO101_SHOULDER_LIFT_PID_I:-$SERVO_PID_I}"
SHOULDER_LIFT_PID_D="${SO101_SHOULDER_LIFT_PID_D:-$SERVO_PID_D}"
ELBOW_FLEX_PID_P="${SO101_ELBOW_FLEX_PID_P:-28}"
ELBOW_FLEX_PID_I="${SO101_ELBOW_FLEX_PID_I:-$SERVO_PID_I}"
ELBOW_FLEX_PID_D="${SO101_ELBOW_FLEX_PID_D:-$SERVO_PID_D}"
WRIST_FLEX_PID_P="${SO101_WRIST_FLEX_PID_P:-$SERVO_PID_P}"
WRIST_FLEX_PID_I="${SO101_WRIST_FLEX_PID_I:-$SERVO_PID_I}"
WRIST_FLEX_PID_D="${SO101_WRIST_FLEX_PID_D:-$SERVO_PID_D}"
WITH_TRAJECTORY_ACTION="${SO101_WITH_TRAJECTORY_ACTION:-true}"
TRAJECTORY_ACTION_RESAMPLE_HZ="${SO101_TRAJECTORY_ACTION_RESAMPLE_HZ:-80.0}"

so101_sanitize_ros_python
so101_check_serial_port "$PORT"

so101_source_ros
cd "$SO101_ROOT/ros1_ws"
catkin_make
source devel/setup.bash

echo "[SO101] Starting smooth hardware bridge:"
echo "  port:                     $PORT"
echo "  command_rate_hz:          $COMMAND_RATE_HZ"
echo "  trajectory_interpolation: $TRAJECTORY_INTERPOLATION"
echo "  command_lpf_alpha:        $LPF_ALPHA"
echo "  servo_speed:              $SERVO_SPEED"
echo "  servo_acceleration:       $SERVO_ACCELERATION"
echo "  servo_maximum_accel:      $SERVO_MAXIMUM_ACCELERATION"
echo "  servo_pid default:        P=$SERVO_PID_P I=$SERVO_PID_I D=$SERVO_PID_D"
echo "  servo_pid shoulder_lift:  P=$SHOULDER_LIFT_PID_P I=$SHOULDER_LIFT_PID_I D=$SHOULDER_LIFT_PID_D"
echo "  servo_pid elbow_flex:     P=$ELBOW_FLEX_PID_P I=$ELBOW_FLEX_PID_I D=$ELBOW_FLEX_PID_D"
echo "  servo_pid wrist_flex:     P=$WRIST_FLEX_PID_P I=$WRIST_FLEX_PID_I D=$WRIST_FLEX_PID_D"
echo "  feedback_position_assist_gain: $FEEDBACK_POSITION_ASSIST_GAIN"
echo "  feedback_position_assist_integral_gain: $FEEDBACK_POSITION_ASSIST_INTEGRAL_GAIN"
echo "  backlash_compensation: enabled=$BACKLASH_COMPENSATION_ENABLED profile=${BACKLASH_COMPENSATION_PROFILE:-<none>}"
echo "  configure_motors_on_connect: $CONFIGURE_MOTORS_ON_CONNECT"
echo "  configuration_write_ack:     $CONFIGURATION_WRITE_ACK"
echo "  follow_joint_trajectory:  $WITH_TRAJECTORY_ACTION"
echo "  action_resample_hz:       $TRAJECTORY_ACTION_RESAMPLE_HZ"

roslaunch so101_ros1_bridge hardware_bridge.launch \
  port:="$PORT" \
  command_rate_hz:="$COMMAND_RATE_HZ" \
  publish_rate_hz:="$PUBLISH_RATE_HZ" \
  feedback_read_rate_hz:="$FEEDBACK_READ_RATE_HZ" \
  telemetry_rate_hz:="$TELEMETRY_RATE_HZ" \
  trajectory_interpolation:="$TRAJECTORY_INTERPOLATION" \
  command_lpf_alpha:="$LPF_ALPHA" \
  command_deadband_rad:="$COMMAND_DEADBAND_RAD" \
  feedback_position_assist_gain:="$FEEDBACK_POSITION_ASSIST_GAIN" \
  feedback_position_assist_integral_gain:="$FEEDBACK_POSITION_ASSIST_INTEGRAL_GAIN" \
  feedback_position_assist_max_offset_rad:="$FEEDBACK_POSITION_ASSIST_MAX_OFFSET_RAD" \
  feedback_position_assist_limit_margin_rad:="$FEEDBACK_POSITION_ASSIST_LIMIT_MARGIN_RAD" \
  feedback_position_assist_alpha:="$FEEDBACK_POSITION_ASSIST_ALPHA" \
  feedback_position_assist_max_feedback_age_s:="$FEEDBACK_POSITION_ASSIST_MAX_FEEDBACK_AGE_S" \
  feedback_position_assist_trajectory_only:="$FEEDBACK_POSITION_ASSIST_TRAJECTORY_ONLY" \
  backlash_compensation_enabled:="$BACKLASH_COMPENSATION_ENABLED" \
  backlash_compensation_profile:="$BACKLASH_COMPENSATION_PROFILE" \
  backlash_compensation_trajectory_only:="$BACKLASH_COMPENSATION_TRAJECTORY_ONLY" \
  backlash_compensation_max_abs_bias_rad:="$BACKLASH_COMPENSATION_MAX_ABS_BIAS_RAD" \
  backlash_compensation_bias_slew_rad_s:="$BACKLASH_COMPENSATION_BIAS_SLEW_RAD_S" \
  backlash_compensation_velocity_threshold_rad_s:="$BACKLASH_COMPENSATION_VELOCITY_THRESHOLD_RAD_S" \
  backlash_compensation_position_hysteresis_rad:="$BACKLASH_COMPENSATION_POSITION_HYSTERESIS_RAD" \
  backlash_compensation_limit_margin_rad:="$BACKLASH_COMPENSATION_LIMIT_MARGIN_RAD" \
  configure_motors_on_connect:="$CONFIGURE_MOTORS_ON_CONNECT" \
  configuration_write_ack:="$CONFIGURATION_WRITE_ACK" \
  servo_speed:="$SERVO_SPEED" \
  servo_acceleration:="$SERVO_ACCELERATION" \
  servo_maximum_acceleration:="$SERVO_MAXIMUM_ACCELERATION" \
  servo_pid_p:="$SERVO_PID_P" \
  servo_pid_i:="$SERVO_PID_I" \
  servo_pid_d:="$SERVO_PID_D" \
  shoulder_lift_pid_p:="$SHOULDER_LIFT_PID_P" \
  shoulder_lift_pid_i:="$SHOULDER_LIFT_PID_I" \
  shoulder_lift_pid_d:="$SHOULDER_LIFT_PID_D" \
  elbow_flex_pid_p:="$ELBOW_FLEX_PID_P" \
  elbow_flex_pid_i:="$ELBOW_FLEX_PID_I" \
  elbow_flex_pid_d:="$ELBOW_FLEX_PID_D" \
  wrist_flex_pid_p:="$WRIST_FLEX_PID_P" \
  wrist_flex_pid_i:="$WRIST_FLEX_PID_I" \
  wrist_flex_pid_d:="$WRIST_FLEX_PID_D" \
  with_trajectory_action:="$WITH_TRAJECTORY_ACTION" \
  trajectory_action_resample_hz:="$TRAJECTORY_ACTION_RESAMPLE_HZ"
