#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

OUT_ROOT="${1:-$SO101_ROOT/logs/smoothness}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/${STAMP}"
mkdir -p "$OUT_DIR"

CYCLES="${SO101_REPEAT_CYCLES:-3}"
CENTER_X="${SO101_CENTER_X:-0.380}"
CENTER_Y="${SO101_CENTER_Y:-0.000}"
CENTER_Z="${SO101_CENTER_Z:-0.160}"
X_AMPLITUDE="${SO101_X_AMPLITUDE:-0.060}"
Z_AMPLITUDE="${SO101_Z_AMPLITUDE:-0.030}"
FREQUENCY="${SO101_FREQUENCY:-0.030}"
DURATION="${SO101_DURATION:-90.0}"
RATE="${SO101_TRAJECTORY_RATE:-50.0}"
RAMP_DURATION="${SO101_RAMP_DURATION:-8.0}"
Z_BIAS="${SO101_Z_FEEDFORWARD_BIAS:-0.000}"

so101_source_ros
so101_source_ws

echo "[SO101] EE smoothness/repeatability output:"
echo "  $OUT_DIR"
echo "[SO101] Hardware bridge should already be running, preferably:"
echo "  scripts/run_ros_hardware_bridge_smooth.sh /dev/ttyACM0"
echo "[SO101] Trajectory:"
echo "  center=($CENTER_X, $CENTER_Y, $CENTER_Z) x_amp=$X_AMPLITUDE z_amp=$Z_AMPLITUDE freq=$FREQUENCY rate=$RATE z_bias=$Z_BIAS"

rosrun so101_ros1_bridge so101_control_cli.py status > "$OUT_DIR/status_start.json"

for cycle in $(seq 1 "$CYCLES"); do
  CSV="$OUT_DIR/ee_xz_sine_120w_60h_cycle_${cycle}.csv"
  echo "[SO101] Cycle $cycle/$CYCLES -> $CSV"
  rosrun so101_ros1_bridge so101_ee_sine_test.py \
    --execution-mode joint_trajectory \
    --pattern xz_sine \
    --center "$CENTER_X" "$CENTER_Y" "$CENTER_Z" \
    --x-amplitude "$X_AMPLITUDE" \
    --z-amplitude "$Z_AMPLITUDE" \
    --frequency "$FREQUENCY" \
    --duration "$DURATION" \
    --rate "$RATE" \
    --ramp-duration "$RAMP_DURATION" \
    --move-to-center-duration 12.0 \
    --center-joint-tolerance 0.03 \
    --center-max-start-error 0.04 \
    --z-feedforward-bias "$Z_BIAS" \
    --csv "$CSV"
  sleep 2
done

rosrun so101_ros1_bridge so101_control_cli.py status > "$OUT_DIR/status_end.json"

echo "[SO101] Analyzing repeatability CSV outputs"
rosrun so101_ros1_bridge so101_analyze_csv.py --ignore-start 3.0 "$OUT_DIR"/*.csv | tee "$OUT_DIR/analysis.txt"

echo "[SO101] Done:"
echo "  CSV:      $OUT_DIR"
echo "  analysis: $OUT_DIR/analysis.txt"
