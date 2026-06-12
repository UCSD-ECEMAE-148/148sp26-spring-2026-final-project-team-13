#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/shared/lib/stack_common.sh"

ARCH="$(uname -m)"

echo "=== [1/2] Building camera + lidar images ==="
stack_build_sensor_images "$ROOT"
echo "[OK] ece191/camera:humble and ece191/lidar:humble built."

echo ""
echo "=== [2/2] Building fusion image ==="
if [[ "$ARCH" =~ ^(aarch64|arm64)$ ]]; then
    bash "$ROOT/fusion/docker_rpi5/build.sh"
    FUSION_IMAGE="ros2_camera_lidar_fusion:rpi5"
    LAUNCHER="bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3"
else
    cd "$ROOT/fusion/docker"
    bash build.sh
    FUSION_IMAGE="ros2_camera_lidar_fusion:latest"
    LAUNCHER="bash ~/sensorfusion_ws/shared/start_all.sh 99"
fi
echo "[OK] ${FUSION_IMAGE} built."

echo ""
echo "============================================================"
echo "  All images built. You can now run:"
echo "  ${LAUNCHER}"
echo "============================================================"
