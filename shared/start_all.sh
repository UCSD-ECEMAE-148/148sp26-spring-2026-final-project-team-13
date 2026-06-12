#!/usr/bin/env bash
# start_all.sh — Launch camera, lidar, and fusion containers.
# Usage: bash start_all.sh [sensor-id]
#   sensor-id: last two digits of Livox serial number (default: 99)
#
# After this completes, enter the fusion container and run:
#   docker exec -it ros2_camera_lidar_fusion /bin/bash
#   bash launch.sh 5   (or 6 for projection + detection)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SENSOR_ID="${1:-99}"
TOPIC_WAIT="${TOPIC_WAIT:-30}"

# ── Colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'
BLUE='\033[0;34m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${BLUE}[INFO]${RESET}  $*"; }
ok()      { echo -e "${GREEN}[OK]${RESET}    $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*"; exit 1; }
section() { echo -e "\n${BOLD}=== $* ===${RESET}"; }

# ── Cleanup on any failure or Ctrl+C ─────────────────────────────────────────
cleanup() {
    echo ""
    info "Cleaning up all containers..."
    bash "$ROOT/shared/stop_all.sh"
}
trap cleanup EXIT

# ── Source host ROS2 ──────────────────────────────────────────────────────────
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    source /opt/ros/foxy/setup.bash
else
    error "No ROS2 installation found on host."
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ── Wait for a ROS2 topic ─────────────────────────────────────────────────────
wait_for_topic() {
    local topic="$1" timeout="$2" label="$3"
    info "Waiting up to ${timeout}s for ${topic}..."
    local deadline=$((SECONDS + timeout))
    while (( SECONDS < deadline )); do
        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            ok "${label} is publishing."
            return 0
        fi
        sleep 2
    done
    echo ""
    echo -e "${RED}[ERROR]${RESET} Timed out waiting for ${topic}."
    case "$topic" in
        /livox/lidar)
            echo "        Possible causes:"
            echo "          - LiDAR is not powered on (check red tape USB is in 12V)"
            echo "          - Ethernet cable is not connected to the Jetson"
            echo "          - Wrong host IP (default: 192.168.1.50 — check LIVOX_HOST_IP)"
            ;;
        /oak/rgb/image_raw)
            echo "        Possible causes:"
            echo "          - OAK-D USB cable is not connected to the Jetson"
            echo "          - Camera is already in use by another process"
            echo "          - Try: docker ps | grep camera  to check for stale containers"
            ;;
        /sensorfusion_out)
            echo "        Possible causes:"
            echo "          - Fusion container failed to start"
            echo "          - Try: docker logs ros2_camera_lidar_fusion"
            ;;
    esac
    exit 1
}

# ── Check all images exist before starting anything ───────────────────────────
docker image inspect ece191/camera:humble >/dev/null 2>&1 \
    || error "ece191/camera:humble missing. Run: bash ~/sensorfusion_ws/shared/build_all.sh"
docker image inspect ece191/lidar:humble >/dev/null 2>&1 \
    || error "ece191/lidar:humble missing. Run: bash ~/sensorfusion_ws/shared/build_all.sh"
docker image inspect ros2_camera_lidar_fusion:latest >/dev/null 2>&1 \
    || error "ros2_camera_lidar_fusion:latest missing. Run: bash ~/sensorfusion_ws/shared/build_all.sh"

# =============================================================================
echo ""
echo -e "${BOLD}sensorfusion_ws — Pipeline Launcher${RESET}"
echo -e "  Sensor ID: ${SENSOR_ID}"
echo ""

# =============================================================================
section "1/3 — Camera"
cd "$ROOT/camera"
bash launch_camera_host.sh &
wait_for_topic "/oak/rgb/image_raw" "$TOPIC_WAIT" "Camera"

# =============================================================================
section "2/3 — LiDAR"
cd "$ROOT/lidar"
bash launch_lidar_foxy_host.sh "$SENSOR_ID" &
wait_for_topic "/livox/lidar" "$TOPIC_WAIT" "LiDAR"

# =============================================================================
section "3/3 — Fusion"
cd "$ROOT/fusion/docker"

# Stop any existing fusion container
docker stop ros2_camera_lidar_fusion 2>/dev/null || true
docker rm   ros2_camera_lidar_fusion 2>/dev/null || true

xhost +local:docker 2>/dev/null || true

docker run -d \
    --name ros2_camera_lidar_fusion \
    --runtime nvidia \
    --device /dev/nvidia0 \
    --device /dev/nvidiactl \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    --env="RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net host \
    --ipc host \
    --pid host \
    --privileged \
    --volume /home/jetson/sensorfusion/ros2_camera_lidar_fusion:/ros2_ws/src/ros2_camera_lidar_fusion \
    --volume "$(pwd)"/start.sh:/ros2_ws/start.sh \
    --volume "$(pwd)"/launch.sh:/ros2_ws/launch.sh \
    --volume /home/jetson/Downloads/p213.pcd:/ros2_ws/p213.pcd \
    -w /ros2_ws \
    ros2_camera_lidar_fusion:latest \
    /bin/bash -c "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/install/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null || true && sleep infinity"	
  ok "Fusion container started."

# ── Disable cleanup trap now that everything is running ───────────────────────
# We only want cleanup on failure, not on normal Ctrl+C from here
trap - EXIT

# =============================================================================
echo ""
echo -e "${GREEN}${BOLD}============================================================${RESET}"
echo -e "${GREEN}${BOLD}  All containers running!${RESET}"
echo -e "${GREEN}${BOLD}============================================================${RESET}"
echo ""
echo -e "  ${BOLD}Next step — enter the fusion container and pick a mode:${RESET}"
echo -e "    docker exec -it ros2_camera_lidar_fusion /bin/bash"
echo -e "    bash launch.sh 5   # lidar projection"
echo -e "    bash launch.sh 6   # lidar projection + detection"
echo ""
echo -e "  ${BOLD}Stop everything:${RESET}"
echo -e "    bash ~/sensorfusion_ws/shared/stop_all.sh"
echo ""

trap 'echo ""; info "Shutting down..."; bash ~/sensorfusion_ws/shared/stop_all.sh; exit 0' INT TERM
info "Press Ctrl+C to stop the entire pipeline."
while true; do sleep 5; done
