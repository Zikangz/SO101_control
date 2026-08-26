#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

OUT_ROOT="${1:-$SO101_ROOT/logs/precision}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/${STAMP}"
mkdir -p "$OUT_DIR"

so101_source_ros
so101_source_ws

echo "[SO101] Precision suite output:"
echo "  $OUT_DIR"
echo "[SO101] Hardware or mock bridge must already be running."

rosrun so101_ros1_bridge so101_control_cli.py status > "$OUT_DIR/status_start.json"
rosrun so101_ros1_bridge so101_control_cli.py named ready --duration 2.5
sleep 1

echo "[SO101] 1/7 hold drift at ready pose"
rosrun so101_ros1_bridge so101_precision_check.py \
  --sequence ready \
  --cycles 1 \
  --duration 2.5 \
  --settle 1.0 \
  --sample-window 30.0 \
  --sample-rate 10.0 \
  --csv "$OUT_DIR/01_hold_ready_30s.csv"

echo "[SO101] 2/7 safe-pose repeatability"
rosrun so101_ros1_bridge so101_precision_check.py \
  --sequence ready stow ready reach ready \
  --cycles 3 \
  --duration 2.5 \
  --settle 1.0 \
  --sample-window 3.0 \
  --sample-rate 10.0 \
  --csv "$OUT_DIR/02_repeatability_safe_poses.csv"

run_sine() {
  local idx="$1"
  local joint="$2"
  local amplitude="$3"
  local frequency="$4"
  local duration="$5"
  echo "[SO101] ${idx}/7 sine tracking: ${joint}"
  rosrun so101_ros1_bridge so101_sine_test.py \
    --joint "$joint" \
    --amplitude "$amplitude" \
    --frequency "$frequency" \
    --duration "$duration" \
    --rate 30.0 \
    --csv "$OUT_DIR/${idx}_sine_${joint}.csv"
}

run_sine "3" "shoulder_lift" "0.08" "0.05" "60.0"
run_sine "4" "elbow_flex" "0.08" "0.05" "60.0"
run_sine "5" "wrist_flex" "0.07" "0.05" "60.0"
run_sine "6" "gripper" "0.10" "0.05" "40.0"

echo "[SO101] 7/7 end-effector XZ figure-eight tracking"
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --pattern figure8 \
  --x-amplitude 0.020 \
  --z-amplitude 0.016 \
  --frequency 0.04 \
  --duration 90.0 \
  --rate 10.0 \
  --csv "$OUT_DIR/7_ee_figure8_xz.csv"

rosrun so101_ros1_bridge so101_control_cli.py named ready --duration 2.5
rosrun so101_ros1_bridge so101_control_cli.py status > "$OUT_DIR/status_end.json"

echo "[SO101] Analyzing CSV outputs"
rosrun so101_ros1_bridge so101_analyze_csv.py "$OUT_DIR"/*.csv | tee "$OUT_DIR/analysis.txt"

echo "[SO101] Done:"
echo "  CSV:      $OUT_DIR"
echo "  analysis: $OUT_DIR/analysis.txt"
