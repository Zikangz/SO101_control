#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

OUT_DIR="${1:-$SO101_ROOT/logs/rosbags}"
mkdir -p "$OUT_DIR"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$OUT_DIR/so101_arm_${STAMP}.bag"

so101_source_ros
so101_source_ws

echo "[SO101] Recording arm bag:"
echo "  $OUT"

exec rosbag record -O "$OUT" \
  /so101/joint_states \
  /so101/status \
  /so101/servo_status \
  /so101/end_effector_pose \
  /so101/kinematics_status \
  /so101/command_joint_positions \
  /so101/command_joint_deltas \
  /so101/cartesian_target \
  /tf \
  /tf_static
