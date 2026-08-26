# Source this file to activate the project environment.
# Usage: source scripts/so101_env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/so101_common.sh"

if [ -f "$HOME/miniforge3/etc/profile.d/conda.sh" ] && [ -d "$SO101_ROOT/.conda-so101-noetic" ]; then
  # shellcheck disable=SC1091
  source "$HOME/miniforge3/etc/profile.d/conda.sh"
  conda activate "$SO101_ROOT/.conda-so101-noetic"
elif [ -x "$SO101_ROOT/.conda-so101-noetic/bin/python" ]; then
  export PATH="$SO101_ROOT/.conda-so101-noetic/bin:$PATH"
else
  echo "[SO101] Local conda env not found; using system python3"
fi

export PYTHONNOUSERSITE=1
if [ -d "$SO101_ROOT/.conda-so101-noetic/lib" ]; then
  export LD_LIBRARY_PATH="$SO101_ROOT/.conda-so101-noetic/lib:${LD_LIBRARY_PATH:-}"
fi
if [ -d "$SO101_ROOT/.conda-so101-noetic/lib/python3.8/site-packages/PySide6/Qt/plugins" ]; then
  export QT_PLUGIN_PATH="$SO101_ROOT/.conda-so101-noetic/lib/python3.8/site-packages/PySide6/Qt/plugins"
fi
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_PYTHON_VERSION=3

if [ -f "$(so101_ros_setup_path)" ]; then
  so101_source_ros
fi
if [ -f "$SO101_ROOT/ros1_ws/devel/setup.bash" ]; then
  so101_source_ws
else
  echo "[SO101] Workspace not built yet; run scripts/bootstrap_noetic.sh"
fi

echo "[SO101] Root: $SO101_ROOT"
echo "[SO101] Python: $(command -v python3 || command -v python)"
python3 - <<'PY'
import sys
print("[SO101] Version:", sys.version.split()[0])
PY
