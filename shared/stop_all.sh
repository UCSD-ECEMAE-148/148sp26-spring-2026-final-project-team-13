#!/usr/bin/env bash
# stop_all.sh — Stop all sensor fusion containers.
# Usage: bash stop_all.sh [rpi5|jetson]
set -euo pipefail

PROFILE="${1:-all}"

stop_named() {
    local name="$1"
    if docker stop "$name" >/dev/null 2>&1; then
        docker rm "$name" >/dev/null 2>&1 || true
        echo "  Stopped: $name"
    fi
}

echo "Stopping all sensor fusion containers..."

if [[ "$PROFILE" == "all" || "$PROFILE" == "jetson" ]]; then
    stop_named ros2_camera_lidar_fusion
fi

if [[ "$PROFILE" == "all" || "$PROFILE" == "rpi5" ]]; then
    stop_named ros2_camera_lidar_fusion_rpi5
    stop_named sensorfusion-camera
fi

for cname in $(docker ps --format '{{.Names}}' | grep '^lidar_foxy_host_' || true); do
    docker stop "$cname" >/dev/null 2>&1 && docker rm "$cname" >/dev/null 2>&1 && echo "  Stopped: $cname"
done

if [[ "$PROFILE" == "all" || "$PROFILE" == "jetson" ]]; then
    for cname in $(docker ps --format '{{.Names}}' | grep 'camera' || true); do
        docker stop "$cname" >/dev/null 2>&1 && echo "  Stopped: $cname"
    done
fi

echo "Done."
