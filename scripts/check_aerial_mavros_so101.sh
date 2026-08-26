#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

so101_source_ros
so101_source_ws

topics=(
  /mavros/state
  /mavros/local_position/pose
  /mavros/local_position/velocity_local
  /mavros/imu/data
  /mavros/battery
  /so101/joint_states
  /so101/status
  /so101/end_effector_pose
  /aerial_manipulation/state
)

echo "[SO101] Checking required aerial manipulation topics"
missing=0
for topic in "${topics[@]}"; do
  if rostopic list | grep -qx "$topic"; then
    echo "  OK      $topic"
  else
    echo "  MISSING $topic"
    missing=1
  fi
done

echo
echo "[SO101] One-shot summary, if topics are active:"
timeout 3 rostopic echo -n 1 /aerial_manipulation/state || true

exit "$missing"
