#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${SCRIPT_DIR}"

HOST_ROS_DISTRO="${HOST_ROS_DISTRO:-foxy}"
LIVOX_HOST_IP="${LIVOX_HOST_IP:-}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-30}"
ECHO_TIMEOUT="${ECHO_TIMEOUT:-15}"
BUILD_POLICY="${BUILD_POLICY:-missing}"
USE_ARM64_OVERRIDE="${USE_ARM64_OVERRIDE:-auto}"
IMAGE_NAME="ece191/lidar:humble"

usage() {
    cat <<'EOF'
Usage:
  ./launch_lidar_foxy_host.sh <sensor-id> [topic]

Arguments:
  <sensor-id>   Last two digits of the Livox MID360 serial number.
  [topic]       Host topic to verify after startup. Default: /livox/lidar

Environment overrides:
  HOST_ROS_DISTRO     Host ROS distro to source before running ros2 commands (default: foxy)
  LIVOX_HOST_IP       Host IP used by the Livox driver config (default: 192.168.1.50)
  LIDAR_TOPIC         Topic to verify from the host (default: /livox/lidar)
  STARTUP_TIMEOUT     Seconds to wait for the topic to appear on the host (default: 30)
  ECHO_TIMEOUT        Seconds to allow ros2 topic echo to produce one full message (default: 15)
  BUILD_POLICY        missing | always | never (default: missing)
  USE_ARM64_OVERRIDE  auto | yes | no (default: auto)
EOF
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
    usage >&2
    exit 1
fi

SENSOR_ID="$1"
if [[ ! "${SENSOR_ID}" =~ ^[0-9]+$ ]]; then
    echo "ERROR: <sensor-id> must be numeric." >&2
    exit 1
fi

if [[ $# -eq 2 ]]; then
    LIDAR_TOPIC="$2"
fi

if [[ -z "${LIVOX_HOST_IP}" ]]; then
    LIVOX_HOST_IP="$(bash "${ROOT}/shared/detect_livox_host_ip.sh" "${SENSOR_ID}")"
fi
export LIVOX_HOST_IP

if ! ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | grep -qx "${LIVOX_HOST_IP}"; then
    echo "WARNING: ${LIVOX_HOST_IP} is not assigned to any host interface." >&2
    echo "         The Livox driver will fail to bind unless this address is configured." >&2
    echo "         Run: sudo bash ${ROOT}/shared/setup_livox_network.sh eth0 ${LIVOX_HOST_IP}" >&2
fi

case "${BUILD_POLICY}" in
    missing|always|never) ;;
    *)
        echo "ERROR: BUILD_POLICY must be one of: missing, always, never." >&2
        exit 1
        ;;
esac

case "${USE_ARM64_OVERRIDE}" in
    auto|yes|no) ;;
    *)
        echo "ERROR: USE_ARM64_OVERRIDE must be one of: auto, yes, no." >&2
        exit 1
        ;;
esac

for numeric_var in STARTUP_TIMEOUT ECHO_TIMEOUT; do
    if [[ ! "${!numeric_var}" =~ ^[1-9][0-9]*$ ]]; then
        echo "ERROR: ${numeric_var} must be a positive integer." >&2
        exit 1
    fi
done

for required_cmd in docker timeout grep awk uname sed mktemp cat; do
    if ! command -v "${required_cmd}" >/dev/null 2>&1; then
        echo "ERROR: Required command not found: ${required_cmd}" >&2
        exit 1
    fi
done

ROS_SETUP="/opt/ros/${HOST_ROS_DISTRO}/setup.bash"
HOST_ROS_AVAILABLE=1
if [[ ! -f "${ROS_SETUP}" ]]; then
    for fallback_distro in humble foxy; do
        if [[ -f "/opt/ros/${fallback_distro}/setup.bash" ]]; then
            HOST_ROS_DISTRO="${fallback_distro}"
            ROS_SETUP="/opt/ros/${HOST_ROS_DISTRO}/setup.bash"
            break
        fi
    done
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
    HOST_ROS_AVAILABLE=0
    echo "[INFO] No host ROS installation found; starting LiDAR container without ROS topic verification."
fi

if [[ ${HOST_ROS_AVAILABLE} -eq 1 ]]; then
    # shellcheck source=/dev/null
    set +u
    source "${ROS_SETUP}"
    set -u

    if ! command -v ros2 >/dev/null 2>&1; then
        echo "ERROR: ros2 is still unavailable after sourcing ${ROS_SETUP}" >&2
        exit 1
    fi
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

COMPOSE_FILES=(-f docker-compose.yml)
ARCH="$(uname -m)"
if [[ "${USE_ARM64_OVERRIDE}" == "yes" ]] || {
    [[ "${USE_ARM64_OVERRIDE}" == "auto" ]] &&
    [[ -f docker-compose.arm64.yml ]] &&
    [[ "${ARCH}" =~ ^(aarch64|arm64)$ ]]
}; then
    COMPOSE_FILES+=(-f docker-compose.arm64.yml)
fi

compose_cmd() {
    if docker compose version >/dev/null 2>&1; then
        docker compose "${COMPOSE_FILES[@]}" "$@"
    elif command -v docker-compose >/dev/null 2>&1; then
        docker-compose "${COMPOSE_FILES[@]}" "$@"
    else
        echo "ERROR: neither 'docker compose' nor 'docker-compose' is available." >&2
        exit 1
    fi
}

docker_build() {
    docker build \
        --build-arg ROS_BASE_IMAGE=arm64v8/ros:humble-perception-jammy \
        -t "${IMAGE_NAME}" \
    -f ../camera/Dockerfile.lidar \
        .
}

docker_run() {
    docker run -d \
        --name "${container_name}" \
        --network host \
        --privileged \
        -e DISPLAY="${DISPLAY:-}" \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
        -e LIVOX_HOST_IP="${LIVOX_HOST_IP}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        -v /sys:/sys:ro \
        -v /run/udev:/run/udev:ro \
        -v "${ROOT}/lidar/ece191-ros2-livox-lidar/launch/run.sh:/home/devuser/livox_ws/src/run.sh:ro" \
        -v "${ROOT}/lidar/ece191-ros2-livox-lidar/launch/pointcloud_MID360_launch.py:/home/devuser/livox_ws/src/pointcloud_MID360_launch.py:ro" \
        "${IMAGE_NAME}" \
        /bin/bash -lc "bash /home/devuser/livox_ws/src/run.sh ${SENSOR_ID} ros2 launch /home/devuser/livox_ws/src/pointcloud_MID360_launch.py"
}

container_name="lidar_foxy_host_${SENSOR_ID}_$(date +%Y%m%d_%H%M%S)"
compose_description="docker-compose.yml"
container_started=0
if [[ " ${COMPOSE_FILES[*]} " == *" docker-compose.arm64.yml "* ]]; then
    compose_description+=" + docker-compose.arm64.yml"
fi

cleanup() {
    if [[ ${container_started} -eq 1 && -n "${container_name:-}" ]]; then
        docker rm -f "${container_name}" >/dev/null 2>&1 || true
    fi
}

handle_signal() {
    echo "Interrupted. Cleaning up ${container_name}..."
    exit 130
}

trap cleanup EXIT
trap handle_signal HUP INT TERM

echo "Using host ROS distro: ${HOST_ROS_DISTRO}"
echo "Using compose files: ${compose_description}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "LIVOX_HOST_IP=${LIVOX_HOST_IP}"
echo "Host verify topic: ${LIDAR_TOPIC}"

need_build=0
case "${BUILD_POLICY}" in
    always)
        need_build=1
        ;;
    missing)
        if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
            need_build=1
        fi
        ;;
    never)
        need_build=0
        ;;
esac

if [[ ${need_build} -eq 1 ]]; then
    echo "Building ${IMAGE_NAME}..."
    if docker compose version >/dev/null 2>&1 || command -v docker-compose >/dev/null 2>&1; then
        compose_cmd build lidar
    else
        docker_build
    fi
else
    echo "Skipping image build (${BUILD_POLICY})."
fi

echo "Starting LiDAR publisher container: ${container_name}"
if docker compose version >/dev/null 2>&1 || command -v docker-compose >/dev/null 2>&1; then
    compose_cmd run --rm -d --name "${container_name}" \
        -e LIVOX_HOST_IP="${LIVOX_HOST_IP}" \
        -v "${ROOT}/lidar/ece191-ros2-livox-lidar/launch/run.sh:/home/devuser/livox_ws/src/run.sh:ro" \
        -v "${ROOT}/lidar/ece191-ros2-livox-lidar/launch/pointcloud_MID360_launch.py:/home/devuser/livox_ws/src/pointcloud_MID360_launch.py:ro" \
        lidar /bin/bash -lc \
        "bash /home/devuser/livox_ws/src/run.sh ${SENSOR_ID} ros2 launch /home/devuser/livox_ws/src/pointcloud_MID360_launch.py" >/dev/null
else
    docker ps -a --format '{{.Names}}' | grep -qx "${container_name}" && docker rm -f "${container_name}" >/dev/null 2>&1 || true
    docker_run >/dev/null
fi
container_started=1

if [[ ${HOST_ROS_AVAILABLE} -eq 1 ]]; then
    echo "Waiting up to ${STARTUP_TIMEOUT}s for ${LIDAR_TOPIC} to appear on the Foxy host..."
    deadline=$((SECONDS + STARTUP_TIMEOUT))
    while true; do
        if ros2 topic list 2>/dev/null | grep -qx "${LIDAR_TOPIC}"; then
            break
        fi

        if ! docker ps --format '{{.Names}}' | grep -qx "${container_name}"; then
            echo "ERROR: LiDAR container exited before ${LIDAR_TOPIC} appeared." >&2
            echo "Recent container logs:" >&2
            docker logs --tail 50 "${container_name}" >&2 || true
            exit 1
        fi

        if (( SECONDS >= deadline )); then
            echo "ERROR: Timed out waiting for ${LIDAR_TOPIC} on the host." >&2
            echo "Recent container logs:" >&2
            docker logs --tail 50 "${container_name}" >&2 || true
            exit 1
        fi

        sleep 2
    done

    echo "Topic discovered on the host. Capturing one full message with ros2 topic echo..."
    echo_capture_file="$(mktemp)"
    echo_pid=""

    ros2 topic echo "${LIDAR_TOPIC}" >"${echo_capture_file}" 2>&1 &
    echo_pid=$!

    deadline=$((SECONDS + ECHO_TIMEOUT))
    while (( SECONDS < deadline )); do
        if grep -q '^---$' "${echo_capture_file}"; then
            break
        fi

        if ! kill -0 "${echo_pid}" >/dev/null 2>&1; then
            break
        fi

        sleep 1
    done

    if kill -0 "${echo_pid}" >/dev/null 2>&1; then
        kill -INT "${echo_pid}" >/dev/null 2>&1 || true
        wait "${echo_pid}" 2>/dev/null || true
    else
        wait "${echo_pid}" 2>/dev/null || true
    fi
    echo_pid=""

    if grep -q '^---$' "${echo_capture_file}"; then
        echo_output="$(sed '/^---$/q' "${echo_capture_file}")"
    else
        echo_output="$(cat "${echo_capture_file}")"
    fi
    rm -f "${echo_capture_file}"

    if [[ -z "${echo_output}" ]]; then
        echo "ERROR: ros2 topic echo returned no output for ${LIDAR_TOPIC}." >&2
        echo "Recent container logs:" >&2
        docker logs --tail 50 "${container_name}" >&2 || true
        exit 1
    fi

    if ! printf '%s\n' "${echo_output}" | grep -q '^---$'; then
        echo "ERROR: ros2 topic echo did not produce a complete message within ${ECHO_TIMEOUT}s." >&2
        echo "Partial host output:" >&2
        printf '%s\n' "${echo_output}" >&2
        echo "Recent container logs:" >&2
        docker logs --tail 50 "${container_name}" >&2 || true
        exit 1
    fi

    printf '%s\n' "${echo_output}"
else
    echo "LiDAR container started. Verify /livox/lidar from Foxglove or from a ROS-enabled machine if needed."
fi
echo ""
echo "LiDAR container is running as ${container_name}."
echo "Press Ctrl-C or close this terminal to stop it and clean it up."

while docker ps --format '{{.Names}}' | grep -qx "${container_name}"; do
    sleep 1
done

echo "LiDAR container exited."
