#!/usr/bin/env bash
# verify_camera.sh — Check OAK-D USB presence and /oak/rgb/image_raw publishing.
set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RESET='\033[0m'

echo "=== OAK-D USB check (host) ==="
if lsusb | grep -qi '03e7:'; then
    echo -e "${GREEN}[OK]${RESET}    DepthAI / OAK device found on USB:"
    lsusb | grep -i '03e7:'
else
    echo -e "${RED}[FAIL]${RESET}  No OAK-D / DepthAI device on USB."
    echo ""
    echo "  Expected when connected:  ID 03e7:xxxx  (Intel Movidius / Luxonis)"
    echo "  Your lsusb shows only:"
    lsusb | sed 's/^/    /'
    echo ""
    echo "  Action: plug the OAK-D USB cable into a USB 3.0 port on the Pi, then re-run:"
    echo "    lsusb | grep 03e7"
    exit 1
fi

echo ""
echo "=== Docker camera container ==="
if ! docker ps --format '{{.Names}}' | grep -qx sensorfusion-camera; then
    echo -e "${YELLOW}[WARN]${RESET}  sensorfusion-camera is not running."
    echo "  Start the stack:  bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3"
    exit 1
fi

echo -e "${GREEN}[OK]${RESET}    sensorfusion-camera is running."

echo ""
echo "=== USB inside container ==="
if docker exec sensorfusion-camera lsusb 2>/dev/null | grep -qi '03e7:'; then
    docker exec sensorfusion-camera lsusb | grep -i '03e7:'
    echo -e "${GREEN}[OK]${RESET}    Container can see the OAK-D."
else
    echo -e "${RED}[FAIL]${RESET}  Container cannot see OAK-D. Restart camera container:"
    echo "    bash ~/sensorfusion_ws/shared/restart_camera.sh"
    exit 1
fi

echo ""
echo "=== /oak/rgb/image_raw publish rate ==="
rate="$(docker exec sensorfusion-camera bash -lc \
    'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
     timeout 8 ros2 topic hz /oak/rgb/image_raw 2>&1' \
    | awk '/average rate:/ {print $3}' | tail -1)"

if [[ -n "$rate" ]]; then
    echo -e "${GREEN}[OK]${RESET}    Publishing at ${rate} Hz"
else
    echo -e "${RED}[FAIL]${RESET}  No messages on /oak/rgb/image_raw"
    echo "  Recent logs:"
    docker logs sensorfusion-camera 2>&1 | tail -10 | sed 's/^/    /'
    exit 1
fi

echo ""
echo -e "${GREEN}Camera pipeline is healthy.${RESET}"
