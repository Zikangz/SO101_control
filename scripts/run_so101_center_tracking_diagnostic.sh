#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

# This is intentionally a static test. It pre-positions to the Cartesian
# center, records encoder/FK feedback, and exits without starting a path.
OUT_ROOT="${1:-$SO101_ROOT/logs/center_tracking}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/${STAMP}"
mkdir -p "$OUT_DIR"

CENTER_X="${SO101_CENTER_X:-0.380}"
CENTER_Y="${SO101_CENTER_Y:-0.000}"
CENTER_Z="${SO101_CENTER_Z:-0.160}"
X_AMPLITUDE="${SO101_X_AMPLITUDE:-0.060}"
Z_AMPLITUDE="${SO101_Z_AMPLITUDE:-0.030}"
FREQUENCY="${SO101_FREQUENCY:-0.030}"
MOVE_DURATION="${SO101_CENTER_MOVE_DURATION:-12.0}"
RECORD_DURATION="${SO101_CENTER_RECORD_DURATION:-15.0}"

so101_source_ros
so101_source_ws

STATUS_FILE="$OUT_DIR/status_start.json"
rosrun so101_ros1_bridge so101_control_cli.py status > "$STATUS_FILE"
if grep -Eq '"enabled"[[:space:]]*:[[:space:]]*true' "$STATUS_FILE"; then
  echo "[SO101][ERROR] Outer feedback P/PI is enabled. Restart the bridge with both assist gains set to 0.0." >&2
  exit 2
fi

echo "[SO101] Static center diagnostic output: $OUT_DIR"
echo "[SO101] No Cartesian path will be started."
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --center-diagnostic-only \
  --center-diagnostic-duration "$RECORD_DURATION" \
  --center-diagnostic-rate 20 \
  --pattern xz_sine \
  --center "$CENTER_X" "$CENTER_Y" "$CENTER_Z" \
  --x-amplitude "$X_AMPLITUDE" \
  --z-amplitude "$Z_AMPLITUDE" \
  --frequency "$FREQUENCY" \
  --move-to-center-duration "$MOVE_DURATION" \
  --center-joint-tolerance 0.03 \
  --center-max-start-error 0.04 \
  --z-feedforward-bias 0.0 \
  --csv "$OUT_DIR/center_tracking.csv"

rosrun so101_ros1_bridge so101_control_cli.py status > "$OUT_DIR/status_end.json"
rosrun so101_ros1_bridge so101_analyze_csv.py "$OUT_DIR/center_tracking.csv" | tee "$OUT_DIR/analysis.txt"
echo "[SO101] Done: $OUT_DIR"
