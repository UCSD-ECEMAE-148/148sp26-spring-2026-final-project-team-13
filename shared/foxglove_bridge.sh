#!/usr/bin/env bash
# foxglove_bridge.sh — Start Foxglove bridge inside the fusion container.
# Works with both Jetson (ros2_camera_lidar_fusion) and Pi 5 (ros2_camera_lidar_fusion_rpi5).

PORT="${1:-8765}"
CONTAINER="${FOXGLOVE_CONTAINER:-}"

if [[ -z "$CONTAINER" ]]; then
    if docker ps --format '{{.Names}}' | grep -qx ros2_camera_lidar_fusion_rpi5; then
        CONTAINER="ros2_camera_lidar_fusion_rpi5"
    else
        CONTAINER="ros2_camera_lidar_fusion"
    fi
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "ERROR: Fusion container '$CONTAINER' is not running." >&2
    echo "Start the stack first: bash ~/sensorfusion_ws/shared/start_all_rpi5.sh" >&2
    exit 1
fi

HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "[INFO] Starting Foxglove bridge inside ${CONTAINER} on port ${PORT}"
echo "[INFO] Connect at: ws://${HOST_IP:-<host-ip>}:${PORT}"
echo "[INFO] Open https://app.foxglove.dev in your browser"
echo ""

docker exec -it "$CONTAINER" /bin/bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /ros2_ws/install/setup.bash 2>/dev/null || true &&
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
        port:=${PORT} \
        max_update_rate:=7.0 \
        send_buffer_limit:=10000000 \
        num_threads:=4
"
