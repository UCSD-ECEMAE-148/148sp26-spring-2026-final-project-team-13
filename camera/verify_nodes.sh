#!/bin/bash
# verify_nodes.sh -- Verify that the camera and LiDAR ROS 2 nodes are actively
#                    broadcasting data.
#
# Run this script on the host (or inside any container that shares the same
# ROS_DOMAIN_ID) after the camera and lidar containers are running.
#
# Usage:
#   ./verify_nodes.sh
#
# Environment variables (all optional — shown with their defaults):
#   ROS_DISTRO        ros distribution to source if ros2 is not already on PATH (humble)
#   ROS_DOMAIN_ID     must match the containers (0)
#   RMW_IMPLEMENTATION  must match the containers (rmw_cyclonedds_cpp)
#   CAMERA_TOPIC      topic published by the camera node  (/oak/rgb/image_raw)
#   LIDAR_TOPIC       topic published by the LiDAR node   (/livox/lidar)
#   VERIFY_TIMEOUT    seconds to wait for each topic before declaring failure  (10)
#   VERIFY_MIN_HZ     minimum acceptable publish rate in Hz                    (1.0)

set -euo pipefail

# ── Configurable defaults ─────────────────────────────────────────────────────
CAMERA_TOPIC="${CAMERA_TOPIC:-/oak/rgb/image_raw}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
TIMEOUT="${VERIFY_TIMEOUT:-10}"
MIN_HZ="${VERIFY_MIN_HZ:-1.0}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# ── Source ROS if ros2 is not already on PATH ─────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    SETUP_SCRIPT="/opt/ros/${ROS_DISTRO}/setup.bash"
    if [[ -f "${SETUP_SCRIPT}" ]]; then
        # shellcheck source=/dev/null
        source "${SETUP_SCRIPT}"
    else
        echo "ERROR: ros2 not found and ${SETUP_SCRIPT} does not exist." >&2
        echo "       Install ROS 2 or set ROS_DISTRO to the correct distribution name." >&2
        exit 1
    fi
fi

PASS=0
FAIL=0

# ── check_topic <label> <topic> ───────────────────────────────────────────────
# Confirms the topic exists and is publishing above MIN_HZ.
check_topic() {
    local label="$1"
    local topic="$2"

    printf "  %-44s ... " "${label} (${topic})"

    # 1. Confirm the topic is discoverable.
    if ! ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
        echo "FAIL  (topic not found)"
        FAIL=$((FAIL + 1))
        return
    fi

    # 2. Sample the publish rate for up to TIMEOUT seconds.
    #    ros2 topic hz streams output until killed; timeout terminates it.
    local hz_output
    hz_output=$(timeout "${TIMEOUT}" ros2 topic hz "${topic}" 2>/dev/null || true)

    # Parse the most recent "average rate:" line (value in Hz).
    local avg_hz
    avg_hz=$(echo "${hz_output}" | awk '/average rate:/{rate=$NF} END{print rate}')

    if [[ -z "${avg_hz}" ]]; then
        echo "FAIL  (no messages received within ${TIMEOUT}s)"
        FAIL=$((FAIL + 1))
        return
    fi

    # Floating-point comparison via awk.
    if awk -v rate="${avg_hz}" -v min="${MIN_HZ}" 'BEGIN { exit !(rate + 0 >= min + 0) }'; then
        echo "PASS  (${avg_hz} Hz)"
        PASS=$((PASS + 1))
    else
        echo "FAIL  (${avg_hz} Hz is below minimum ${MIN_HZ} Hz)"
        FAIL=$((FAIL + 1))
    fi
}

# ── Header ────────────────────────────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════════"
echo " ECE 191 ROS 2 Node Verification"
echo " ROS_DOMAIN_ID    : ${ROS_DOMAIN_ID:-0}"
echo " RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo " Timeout per topic : ${TIMEOUT}s  |  Min rate: ${MIN_HZ} Hz"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Checking topics..."
echo ""

# ── Run checks ────────────────────────────────────────────────────────────────
check_topic "Camera (DepthAI/OAK)" "${CAMERA_TOPIC}"
check_topic "LiDAR  (Livox)"       "${LIDAR_TOPIC}"

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "Results: ${PASS} passed, ${FAIL} failed"
echo ""

if [[ ${FAIL} -gt 0 ]]; then
    echo "One or more nodes are not broadcasting. Make sure the containers are"
    echo "running and that the host environment matches the containers:"
    echo ""
    echo "  export ROS_DOMAIN_ID=0"
    echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    echo "  source /opt/ros/\$ROS_DISTRO/setup.bash"
    echo ""
    echo "Start the containers (if not already running):"
    echo "  docker compose up camera"
    echo "  docker compose run --rm lidar \\"
    echo "    /bin/bash -c \"/home/devuser/livox_ws/src/run.sh <id> ros2 launch livox_ros_driver2 rviz_MID360_launch.py\""
    echo ""
    exit 1
fi

echo "All nodes are broadcasting. ✓"
exit 0
