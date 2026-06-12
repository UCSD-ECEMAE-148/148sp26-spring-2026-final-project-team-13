#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FUSION_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

docker build \
  -t ros2_camera_lidar_fusion:rpi5 \
  -f "$SCRIPT_DIR/Dockerfile" \
  "$FUSION_DIR"