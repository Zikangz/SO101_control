#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

OUT_ROOT="${1:-$SO101_ROOT/logs/mujoco_contact}"
SAMPLES="${2:-3000}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/${STAMP}"
mkdir -p "$OUT_DIR"

PYTHON_BIN="${PYTHON:-python3}"
CSV="$OUT_DIR/contact_scan.csv"

echo "[SO101] MuJoCo self-contact scan:"
echo "  csv:     $CSV"
echo "  samples: $SAMPLES"

"$PYTHON_BIN" "$SO101_ROOT/scripts/scan_so101_mujoco_self_collision.py" \
  --samples "$SAMPLES" \
  --joints shoulder_lift elbow_flex wrist_flex \
  --range shoulder_lift=-1.2:1.2 elbow_flex=-1.4:1.4 wrist_flex=-1.0:1.0 \
  --locked shoulder_pan=0.0 wrist_roll=0.0 gripper=0.5 \
  --csv "$CSV" | tee "$OUT_DIR/summary.txt"

echo "[SO101] summary: $OUT_DIR/summary.txt"
