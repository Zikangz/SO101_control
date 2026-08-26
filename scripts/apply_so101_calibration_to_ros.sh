#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

CALIBRATION_ID="${1:-aerial_so101_follower}"
CAL="$SO101_ROOT/calibration/${CALIBRATION_ID}.json"
CFG="$SO101_ROOT/ros1_ws/src/so101_ros1_bridge/config/so101_simplified_4dof.yaml"
PYTHON_BIN="$(so101_python)"

cd "$SO101_ROOT"

PYTHONNOUSERSITE=1 "$PYTHON_BIN" \
  scripts/export_lerobot_calibration_to_ros.py \
  --path "$CAL" \
  --write-config "$CFG"

echo "[SO101] ROS bridge config updated: $CFG"
