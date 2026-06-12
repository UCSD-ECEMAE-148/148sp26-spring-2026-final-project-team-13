#!/usr/bin/env bash
# start_all_rpi5.sh — Unified CPU-only stack launcher for Raspberry Pi 5.
#
# Brings up, in order:
#   1. OAK-D camera driver
#   2. Livox MID-360 LiDAR driver
#   3. Fusion container (ros2_camera_lidar_fusion:rpi5)
#   4. Fusion node + Foxglove bridge
#
# Usage:
#   bash start_all_rpi5.sh [sensor-ip-or-id]
#
# Examples:
#   bash start_all_rpi5.sh 192.168.1.3          # projection only (default)
#   FUSION_MODE=detection bash start_all_rpi5.sh 192.168.1.3   # YOLO + cone detection (launch.sh 6)
#
# Environment:
#   SENSOR_ID / arg1              Livox IP (192.168.1.3) or serial suffix (99)
#   LIVOX_HOST_IP                 Host bind IP (default: auto → 192.168.1.50)
#   FUSION_MODE                   projection | detection (default: projection)
#   FOXGLOVE_PORT                 WebSocket port (default: 8765)
#   TOPIC_WAIT                    Seconds to wait for sensor topics (default: 45)
#   SETUP_LIVOX_NETWORK           If 1, configure eth0 when LiDAR IP is missing
#   LIVOX_IFACE                   Interface for LiDAR network (default: eth0)
#   BUILD_POLICY                  missing | always | never (default: missing)
#   SENSORFUSION_DETECTION_MODEL  YOLO weights (.hef for Hailo, .pt for CPU)
#   SENSORFUSION_DETECTION_BACKEND auto | hailo | cpu (default: auto)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${ROOT}/shared/lib/stack_common.sh"

SENSOR_TARGET="${1:-${SENSOR_ID:-${LIVOX_SENSOR_IP:-192.168.1.3}}}"
FUSION_MODE="${FUSION_MODE:-projection}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
TOPIC_WAIT="${TOPIC_WAIT:-45}"
BUILD_POLICY="${BUILD_POLICY:-missing}"
SETUP_LIVOX_NETWORK="${SETUP_LIVOX_NETWORK:-0}"
LIVOX_IFACE="${LIVOX_IFACE:-eth0}"
DETECTION_MODEL="${SENSORFUSION_DETECTION_MODEL:-}"
DETECTION_BACKEND="${SENSORFUSION_DETECTION_BACKEND:-auto}"
USE_HAILO=0

CAMERA_CONTAINER="sensorfusion-camera"
FUSION_CONTAINER="ros2_camera_lidar_fusion_rpi5"
FUSION_IMAGE="ros2_camera_lidar_fusion:rpi5"
CAMERA_IMAGE="ece191/camera:humble"
LIDAR_IMAGE="ece191/lidar:humble"

RED='\033[0;31m'; GREEN='\033[0;32m'
BLUE='\033[0;34m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${BLUE}[INFO]${RESET}  $*"; }
ok()      { echo -e "${GREEN}[OK]${RESET}    $*"; }
warn()    { echo -e "${RED}[WARN]${RESET}  $*"; }
error()   { echo -e "${RED}[ERROR]${RESET} $*"; exit 1; }
section() { echo -e "\n${BOLD}=== $* ===${RESET}"; }

cleanup() {
    echo ""
    info "Stopping sensor fusion stack..."
    bash "${ROOT}/shared/stop_all.sh" rpi5
}
trap cleanup EXIT INT TERM

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if ! stack_source_ros; then
    warn "No host ROS2 install found; using in-container topic checks."
    HOST_ROS=0
else
    HOST_ROS=1
fi

if ! stack_is_arm64; then
    warn "This launcher targets Raspberry Pi / ARM64. Continuing anyway."
fi

export LIVOX_HOST_IP="${LIVOX_HOST_IP:-$(stack_livox_host_ip "$SENSOR_TARGET")}"

if ! stack_livox_ip_assigned "$LIVOX_HOST_IP"; then
    warn "Livox bind IP ${LIVOX_HOST_IP} is not assigned on the host."
    if [[ "$SETUP_LIVOX_NETWORK" == "1" ]]; then
        info "Running setup_livox_network.sh on ${LIVOX_IFACE}..."
        sudo bash "${ROOT}/shared/setup_livox_network.sh" "$LIVOX_IFACE" "$LIVOX_HOST_IP"
    else
        echo "       Run: sudo bash ${ROOT}/shared/setup_livox_network.sh ${LIVOX_IFACE} ${LIVOX_HOST_IP}"
        echo "       Or:  SETUP_LIVOX_NETWORK=1 bash ${ROOT}/shared/start_all_rpi5.sh ${SENSOR_TARGET}"
    fi
fi

need_image() {
    local image="$1"
    case "$BUILD_POLICY" in
        always) return 0 ;;
        missing) docker image inspect "$image" >/dev/null 2>&1 && return 1 || return 0 ;;
        never) return 1 ;;
    esac
}

if need_image "$CAMERA_IMAGE" || need_image "$LIDAR_IMAGE"; then
    section "Building camera + LiDAR images"
    stack_build_sensor_images "$ROOT"
fi

if need_image "$FUSION_IMAGE"; then
    section "Building fusion image (Pi 5 / CPU-only)"
    bash "${ROOT}/fusion/docker_rpi5/build.sh"
fi

docker image inspect "$CAMERA_IMAGE" >/dev/null 2>&1 \
    || error "${CAMERA_IMAGE} missing. Build with: bash ${ROOT}/fusion/docker_rpi5/build.sh and stack_build_sensor_images, or see shared/docs/PI5_LAUNCH_GUIDE.md"
docker image inspect "$LIDAR_IMAGE" >/dev/null 2>&1 \
    || error "${LIDAR_IMAGE} missing. See shared/docs/PI5_LAUNCH_GUIDE.md"
docker image inspect "$FUSION_IMAGE" >/dev/null 2>&1 \
    || error "${FUSION_IMAGE} missing. Run: bash ${ROOT}/fusion/docker_rpi5/build.sh"

ENABLE_DETECTION=0
FUSION_OUTPUT_TOPIC="/sensorfusion_out"
case "$FUSION_MODE" in
    projection) ;;
    detection)
        ENABLE_DETECTION=1
        FUSION_OUTPUT_TOPIC="/sensorfusion_out2"
        if stack_hailo_device_present && [[ "$DETECTION_BACKEND" == "auto" || "$DETECTION_BACKEND" == "hailo" ]]; then
            USE_HAILO=1
            DETECTION_BACKEND="hailo"
            if [[ -z "$DETECTION_MODEL" ]]; then
                DETECTION_MODEL="$(stack_default_hailo_hef || echo /usr/share/hailo-models/yolov8s_h8.hef)"
            fi
        else
            DETECTION_BACKEND="cpu"
            DETECTION_MODEL="${DETECTION_MODEL:-yolov8n.pt}"
        fi
        ;;
    *)
        error "FUSION_MODE must be 'projection' or 'detection' (got: ${FUSION_MODE})"
        ;;
esac

echo ""
echo -e "${BOLD}sensorfusion_ws — Pi 5 Unified Launcher${RESET}"
echo -e "  LiDAR target:   ${SENSOR_TARGET}"
echo -e "  Livox host IP:  ${LIVOX_HOST_IP}"
echo -e "  Fusion mode:    ${FUSION_MODE} (equiv. launch.sh $([ "$ENABLE_DETECTION" -eq 1 ] && echo 6 || echo 5))"
if [[ "$USE_HAILO" -eq 1 ]]; then
    echo -e "  YOLO backend:   Hailo NPU (${DETECTION_MODEL})"
else
    echo -e "  YOLO backend:   CPU (${DETECTION_MODEL:-n/a})"
fi
echo -e "  Foxglove port:  ${FOXGLOVE_PORT}"
echo ""

# ── 1. Camera ────────────────────────────────────────────────────────────────
section "1/4 — Camera"
if ! stack_check_oak_usb_or_warn; then
    warn "Starting camera container anyway — it will reconnect when OAK-D is plugged in."
    warn "After plugging in: bash ${ROOT}/shared/restart_camera.sh"
else
    ok "OAK-D detected on USB: $(lsusb | grep -i '03e7:' | head -1)"
fi
info "Starting ${CAMERA_CONTAINER}..."
stack_start_camera "$ROOT" "$CAMERA_CONTAINER" "$CAMERA_IMAGE" >/dev/null

if [[ "$HOST_ROS" -eq 1 ]]; then
    stack_wait_for_topic "/oak/rgb/image_raw" "$TOPIC_WAIT" "Camera" \
        || stack_wait_for_topic_in_container "$CAMERA_CONTAINER" "/oak/rgb/image_raw" "$TOPIC_WAIT" "Camera" \
        || { docker logs --tail 40 "$CAMERA_CONTAINER" >&2 || true; exit 1; }
else
    stack_wait_for_topic_in_container "$CAMERA_CONTAINER" "/oak/rgb/image_raw" "$TOPIC_WAIT" "Camera" \
        || warn "Camera topic not yet visible; continuing."
fi

# ── 2. LiDAR ─────────────────────────────────────────────────────────────────
section "2/4 — LiDAR"
LIDAR_CONTAINER="lidar_foxy_host_$(echo "$SENSOR_TARGET" | tr '.' '_')_$(date +%Y%m%d_%H%M%S)"

info "Starting ${LIDAR_CONTAINER}..."
stack_start_lidar "$ROOT" "$LIDAR_CONTAINER" "$LIDAR_IMAGE" "$SENSOR_TARGET" "$LIVOX_HOST_IP" >/dev/null

if [[ "$HOST_ROS" -eq 1 ]]; then
    if ! stack_wait_for_topic "/livox/lidar" "$TOPIC_WAIT" "LiDAR"; then
        stack_wait_for_topic_in_container "$LIDAR_CONTAINER" "/livox/lidar" "$TOPIC_WAIT" "LiDAR" || {
            warn "LiDAR topic not detected. Recent logs:"
            docker logs --tail 50 "$LIDAR_CONTAINER" >&2 || true
            if ! stack_livox_ip_assigned "$LIVOX_HOST_IP"; then
                error "Configure ${LIVOX_HOST_IP} on ${LIVOX_IFACE} and retry."
            fi
            error "LiDAR failed to publish. Check power, Ethernet, and sensor target."
        }
    fi
else
    stack_wait_for_topic_in_container "$LIDAR_CONTAINER" "/livox/lidar" "$TOPIC_WAIT" "LiDAR" \
        || warn "LiDAR topic not yet visible; check: docker logs ${LIDAR_CONTAINER}"
fi

# ── 3. Fusion container ────────────────────────────────────────────────────
section "3/4 — Fusion container"
docker ps -a --format '{{.Names}}' | grep -qx "$FUSION_CONTAINER" \
    && docker rm -f "$FUSION_CONTAINER" >/dev/null 2>&1 || true

FUSION_DIR="${ROOT}/fusion"
info "Starting ${FUSION_CONTAINER}..."
HAILO_DOCKER_ARGS=()
if stack_hailo_device_present; then
    HAILO_DOCKER_ARGS+=(--device /dev/hailo0)
    HAILO_DOCKER_ARGS+=(-v /usr/share/hailo-models:/usr/share/hailo-models:ro)
fi
docker run -d \
    --name "$FUSION_CONTAINER" \
    --net host \
    --ipc host \
    --privileged \
    "${HAILO_DOCKER_ARGS[@]}" \
    --env "DISPLAY=${DISPLAY:-}" \
    --env "QT_X11_NO_MITSHM=1" \
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${FUSION_DIR}:/ros2_ws/src/ros2_camera_lidar_fusion" \
    -w /ros2_ws \
    "$FUSION_IMAGE" \
    /bin/bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && sleep infinity' \
    >/dev/null

ok "Fusion container running (${FUSION_CONTAINER})."

# ── 4. Fusion node + Foxglove ────────────────────────────────────────────────
section "4/4 — Fusion node + Foxglove bridge"
info "Ensuring fusion runtime dependencies..."
stack_ensure_fusion_runtime_deps "$FUSION_CONTAINER" "$ENABLE_DETECTION" "$USE_HAILO"
info "Building fusion package in container (picks up mounted source)..."
stack_rebuild_fusion_package "$FUSION_CONTAINER"
info "Launching fusion + foxglove_bridge (mode=${FUSION_MODE})..."
stack_start_fusion_launch "$FUSION_CONTAINER" "$FUSION_MODE" "$FOXGLOVE_PORT" "$ENABLE_DETECTION" "$DETECTION_MODEL" "$DETECTION_BACKEND"

if ! stack_wait_for_port "$FOXGLOVE_PORT" 30 "Foxglove bridge"; then
    warn "Foxglove did not open port ${FOXGLOVE_PORT}. Recent fusion logs:"
    docker exec "$FUSION_CONTAINER" tail -30 /tmp/fusion_launch.log 2>&1 || true
    error "Foxglove bridge failed to start. See logs above."
fi

stack_wait_for_topic_in_container "$FUSION_CONTAINER" "$FUSION_OUTPUT_TOPIC" 20 "Fusion output" \
    || warn "Fusion output topic not yet visible; sensors may still be syncing."

trap - EXIT
trap 'cleanup; exit 0' INT TERM

WIFI_IP="$(stack_wifi_ip)"
LIDAR_IP="$(stack_lidar_subnet_ip)"
PRIMARY_IP="$(stack_primary_ip)"

echo ""
echo -e "${GREEN}${BOLD}============================================================${RESET}"
echo -e "${GREEN}${BOLD}  Stack running on Raspberry Pi 5$([ "$USE_HAILO" -eq 1 ] && echo ' + Hailo NPU' || echo ' (CPU-only)')${RESET}"
echo -e "${GREEN}${BOLD}============================================================${RESET}"
echo ""
echo -e "  ${BOLD}Foxglove (use WiFi IP from your laptop):${RESET}"
if [[ -n "$WIFI_IP" ]]; then
    echo -e "    ws://${WIFI_IP}:${FOXGLOVE_PORT}"
else
    echo -e "    ws://${PRIMARY_IP}:${FOXGLOVE_PORT}"
fi
if [[ -n "$LIDAR_IP" && "$LIDAR_IP" != "$WIFI_IP" ]]; then
    echo -e "  ${BOLD}LiDAR subnet only:${RESET}  ws://${LIDAR_IP}:${FOXGLOVE_PORT}"
fi
echo ""
echo -e "  ${BOLD}Open:${RESET}  https://app.foxglove.dev"
echo ""
echo -e "  ${BOLD}Topics:${RESET}"
echo -e "    /oak/rgb/image_raw          — camera (Image panel)"
echo -e "    /livox/lidar                — LiDAR (3D panel → add topic)"
echo -e "    ${FUSION_OUTPUT_TOPIC}              — fusion output (Image panel)"
echo ""
echo -e "  ${BOLD}Foxglove panel setup:${RESET}"
echo -e "    Image panel  → /oak/rgb/image_raw"
echo -e "    3D panel     → add /livox/lidar, fixed frame: livox_frame"
if [[ "$ENABLE_DETECTION" -eq 1 ]]; then
    echo -e "    Image panel  → /sensorfusion_out2 (YOLO + cone detection)"
fi
echo ""
echo -e "  ${BOLD}Containers:${RESET}"
echo -e "    Camera:  ${CAMERA_CONTAINER}"
echo -e "    LiDAR:   ${LIDAR_CONTAINER}"
echo -e "    Fusion:  ${FUSION_CONTAINER}"
echo ""
echo -e "  ${BOLD}Stop everything:${RESET}"
echo -e "    bash ${ROOT}/shared/stop_all.sh rpi5"
echo ""
warn "Keep this terminal open. Ctrl+C stops the entire stack."
echo ""

while true; do
    for name in "$CAMERA_CONTAINER" "$LIDAR_CONTAINER" "$FUSION_CONTAINER"; do
        if ! docker ps --format '{{.Names}}' | grep -qx "$name"; then
            warn "Container ${name} exited unexpectedly."
            docker logs --tail 30 "$name" 2>&1 || true
            exit 1
        fi
    done
    sleep 5
done
