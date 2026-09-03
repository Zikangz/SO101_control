#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
STAGE="/tmp/so101_handoff_${STAMP}"
OUT="/tmp/so101_handoff_${STAMP}.tar.gz"

mkdir -p "$STAGE"/repo "$STAGE"/tmp_artifacts "$STAGE"/metadata

copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [ -e "$src" ]; then
    mkdir -p "$(dirname "$dst")"
    cp -a "$src" "$dst"
  fi
}

copy_if_exists "$ROOT/README.md" "$STAGE/repo/README.md"
copy_if_exists "$ROOT/requirements-noetic.txt" "$STAGE/repo/requirements-noetic.txt"
copy_if_exists "$ROOT/.env.example" "$STAGE/repo/.env.example"
copy_if_exists "$ROOT/指令" "$STAGE/repo/指令"
copy_if_exists "$ROOT/指令.txt" "$STAGE/repo/指令.txt"
copy_if_exists "$ROOT/SO101_PX4_空中操作_RL全身控制项目文档.md" "$STAGE/repo/SO101_PX4_空中操作_RL全身控制项目文档.md"
copy_if_exists "$ROOT/docs" "$STAGE/repo/docs"
copy_if_exists "$ROOT/calibration" "$STAGE/repo/calibration"
copy_if_exists "$ROOT/configs" "$STAGE/repo/configs"
copy_if_exists "$ROOT/scripts" "$STAGE/repo/scripts"
copy_if_exists "$ROOT/so101_plots" "$STAGE/repo/so101_plots"
copy_if_exists "$ROOT/ros1_ws/src" "$STAGE/repo/ros1_ws/src"

for path in \
  /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083.csv \
  /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v1.csv \
  /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2.csv \
  /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_repeat_20260903_203926.csv \
  /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_final_20260903_205926.csv \
  /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083_repeat_20260903_204105.csv \
  /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v3.csv \
  /tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_zbias0083_20260903_220808.csv \
  /tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_phasexz_v1_safe_20260903_222747.csv \
  /tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_phasexz_v1_safe_validate.csv \
  /tmp/so101_ee_xz_zigzag_f100_x040_z018_zbias0083.csv \
  /tmp/so101_ee_xz_sine_f100_x060_z030_zbias0083.csv \
  /tmp/so101_ee_xz_8_fast_f040_zbias0083_current_calib.csv \
  /tmp/so101_ee_xz_8_fast_f050_zbias0083_current_calib.csv \
  /tmp/so101_ee_xz_8_fast_f055_zbias0083_current_calib.csv \
  /tmp/so101_ee_xz_8_fast_f060_zbias0083_current_calib.csv \
  /tmp/so101_ee_sine.csv \
  /tmp/so101_phase_z_edge_vertex8_f100_x050_z020.json \
  /tmp/so101_phase_z_edge_vertex8_f100_x050_z020_v2.json \
  /tmp/so101_phase_z_edge_vertex8_f100_x050_z020_v3.json \
  /tmp/so101_phase_xz_edge_vertex8_f100_x050_z020_v1.json \
  /tmp/so101_phase_xz_edge_vertex8_f100_x050_z020_v1_safe.json \
  /tmp/so101_phase_xz_edge_vertex8_f100_x050_z020_v1_ultrasafe.json \
  /tmp/so101_phase_xz_edge_vertex8_f100_x050_z020_v1_micro.json \
  /tmp/so101_phase_xz_vertex_diamond_f100_x050_z020_v1.json \
  /tmp/so101_phase_xz_vertex_diamond_f100_x050_z020_v1_safe.json \
  /tmp/so101_phase_z_zigzag_f100_x040_z018.json \
  /tmp/so101_phase_z_sine_f100_x060_z030.json \
  /tmp/so101_live_trajectory.png; do
  copy_if_exists "$path" "$STAGE/tmp_artifacts/$(basename "$path")"
done

if [ -d /tmp/so101_plots ]; then
  copy_if_exists /tmp/so101_plots "$STAGE/tmp_artifacts/so101_plots"
fi

{
  echo "timestamp: $STAMP"
  echo "root: $ROOT"
  echo "user: ${USER:-unknown}"
  echo "hostname: $(hostname 2>/dev/null || true)"
  echo "kernel: $(uname -a 2>/dev/null || true)"
  echo "python3: $(python3 --version 2>/dev/null || true)"
  echo "ros_distro: ${ROS_DISTRO:-unknown}"
} > "$STAGE/metadata/system_info.txt"

(
  cd "$ROOT"
  git rev-parse --show-toplevel >/dev/null 2>&1 && git rev-parse HEAD || true
) > "$STAGE/metadata/git_head.txt"

(
  cd "$ROOT"
  git status --short || true
) > "$STAGE/metadata/git_status_short.txt"

(
  cd "$ROOT"
  git diff -- ros1_ws/src/so101_ros1_bridge/scripts/so101_ee_sine_test.py \
    ros1_ws/src/so101_ros1_bridge/scripts/so101_fit_phase_z_compensation.py \
    ros1_ws/src/so101_ros1_bridge/scripts/so101_fit_phase_xz_compensation.py \
    scripts/run_ros_hardware_bridge_no_pid.sh \
    ros1_ws/src/so101_ros1_bridge/config/so101_simplified_4dof.yaml \
    calibration/aerial_so101_follower.json || true
) > "$STAGE/metadata/git_diff_key_files.patch"

tar -czf "$OUT" -C "$STAGE" .

echo "handoff_package: $OUT"
du -h "$OUT"
