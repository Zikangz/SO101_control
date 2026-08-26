#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

DIR="${1:-}"
if [ -z "$DIR" ]; then
  DIR="$(find "$SO101_ROOT/logs/precision" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort | tail -n 1)"
fi
if [ -z "$DIR" ] || [ ! -d "$DIR" ]; then
  echo "[SO101][ERROR] Precision directory not found. Pass one explicitly:" >&2
  echo "  scripts/analyze_so101_precision_suite.sh $SO101_ROOT/logs/precision/<stamp>" >&2
  exit 2
fi

so101_source_ros
so101_source_ws

OUT="$DIR/analysis.txt"
rosrun so101_ros1_bridge so101_analyze_csv.py "$DIR"/*.csv | tee "$OUT"
echo "[SO101] analysis: $OUT"
