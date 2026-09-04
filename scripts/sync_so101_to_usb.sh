#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_USB="/media/${USER:-zzk}/Ventoy/ZZK/SO101"
USB_TARGET="${1:-$DEFAULT_USB}"
USB_DEPLOY_DIR="$(dirname "$USB_TARGET")/SO101_JetsonDeploy_20260904"

if [ ! -f "$ROOT/so101_jetson_deploy_20260904.tar.gz" ]; then
  cat >&2 <<EOF
[SO101][ERROR] Missing deploy package:
  $ROOT/so101_jetson_deploy_20260904.tar.gz
Run the Jetson deploy packaging step first.
EOF
  exit 1
fi

if [ ! -f "$ROOT/so101_jetson_deploy_20260904.tar.gz.sha256" ]; then
  cat >&2 <<EOF
[SO101][ERROR] Missing deploy package checksum:
  $ROOT/so101_jetson_deploy_20260904.tar.gz.sha256
EOF
  exit 1
fi

mkdir -p "$USB_TARGET" "$USB_DEPLOY_DIR"

echo "[SO101] Syncing project tree to: $USB_TARGET"
rsync -a --delete \
  --exclude='.git/' \
  --exclude='.agents/' \
  --exclude='.codex/' \
  --exclude='.conda-*/' \
  --exclude='.venv-*/' \
  --exclude='.vscode/' \
  --exclude='ros1_ws/build/' \
  --exclude='ros1_ws/devel/' \
  --exclude='ros1_ws/install/' \
  --exclude='ros1_ws/log/' \
  --exclude='logs/' \
  --exclude='third_party/' \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='so101_handoff_20260903_194131.tar.gz' \
  --exclude='so101_experiment_package_20260903_final.tar.gz' \
  --exclude='so101_experiment_package_20260903_final.tar.gz.sha256' \
  "$ROOT/" "$USB_TARGET/"

echo "[SO101] Writing clean Jetson deploy copy to: $USB_DEPLOY_DIR"
cp -f "$ROOT/so101_jetson_deploy_20260904.tar.gz" "$USB_DEPLOY_DIR/"
cp -f "$ROOT/so101_jetson_deploy_20260904.tar.gz.sha256" "$USB_DEPLOY_DIR/"
cp -f "$ROOT/README.md" "$USB_DEPLOY_DIR/README.md"
cp -f "$ROOT/指令.txt" "$USB_DEPLOY_DIR/指令.txt"
cp -f "$ROOT/so101_plots/SO101_组会汇报说明_20260904.md" "$USB_DEPLOY_DIR/SO101_组会汇报说明_20260904.md"

sync

echo "[SO101] Verifying USB deploy package checksum"
(
  cd "$USB_DEPLOY_DIR"
  sha256sum -c so101_jetson_deploy_20260904.tar.gz.sha256
)

echo "[SO101] USB sync complete"
echo "  project_tree: $USB_TARGET"
echo "  deploy_copy:  $USB_DEPLOY_DIR"
find "$USB_DEPLOY_DIR" -maxdepth 1 -type f -printf '%TY-%Tm-%Td %TH:%TM:%TS %s %p\n' | sort

