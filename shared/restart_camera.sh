#!/usr/bin/env bash
# restart_camera.sh — Restart only the OAK-D camera container (e.g. after plugging in USB).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/shared/lib/stack_common.sh"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if ! lsusb | grep -qi '03e7:'; then
    echo "[ERROR] No OAK-D on USB. Plug in the camera first."
    echo "        Expected: lsusb | grep 03e7"
    lsusb
    exit 1
fi

echo "[INFO] OAK-D detected:"
lsusb | grep -i '03e7:'

stack_start_camera "$ROOT" sensorfusion-camera ece191/camera:humble

echo "[INFO] Waiting for /oak/rgb/image_raw..."
if stack_wait_for_topic_in_container sensorfusion-camera /oak/rgb/image_raw 45 "Camera"; then
    echo "[OK] Camera restarted. Refresh Foxglove Image panel."
else
    echo "[ERROR] Camera topic not publishing. Logs:"
    docker logs sensorfusion-camera 2>&1 | tail -20
    exit 1
fi
