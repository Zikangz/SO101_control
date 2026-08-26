#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

PORT="${1:-/dev/ttyACM0}"
PYTHON_BIN="$(so101_python)"
so101_require_seeed_repo
cd "$SO101_ROOT/third_party/Seeed_RoboController"
so101_prepare_third_party_python
"$PYTHON_BIN" -m src.gui.factory_calibration_tool --port1 "$PORT"
