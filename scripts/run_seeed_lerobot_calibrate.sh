#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

PORT="${1:-/dev/ttyACM0}"
CALIBRATION_ID="${2:-aerial_so101_follower}"
OUT="$SO101_ROOT/calibration/${CALIBRATION_ID}.json"
PYTHON_BIN="$(so101_python)"

mkdir -p "$SO101_ROOT/calibration"
so101_require_seeed_repo
cd "$SO101_ROOT/third_party/Seeed_RoboController"

so101_prepare_third_party_python
export SO101_CALIBRATION_STATUS="${SO101_CALIBRATION_STATUS:-/tmp/so101_current_motor_status.txt}"
export SO101_WRIST_ROLL_LIMITED="${SO101_WRIST_ROLL_LIMITED:-1}"

echo "[SO101] Passive monitor in another terminal:"
echo "watch -n 0.1 'cat ${SO101_CALIBRATION_STATUS} 2>/dev/null || echo waiting-for-calibration'"
echo "[SO101] wrist_roll limited mode: ${SO101_WRIST_ROLL_LIMITED}"

"$PYTHON_BIN" -m src.tools.lerobot_calibrate \
  "$PORT" \
  --arm-type follower \
  --id "$CALIBRATION_ID" \
  --output "$OUT"

echo "[SO101] Calibration saved: $OUT"
