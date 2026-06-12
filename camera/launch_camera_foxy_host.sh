#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

HOST_ROS_DISTRO="${HOST_ROS_DISTRO:-humble}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/oak/rgb/image_raw}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-30}"
HZ_TIMEOUT="${HZ_TIMEOUT:-10}"
BUILD_POLICY="${BUILD_POLICY:-always}"
USE_ARM64_OVERRIDE="${USE_ARM64_OVERRIDE:-auto}"
CAMERA_CONTAINER_NAME="${CAMERA_CONTAINER_NAME:-last-try-camera}"
HOST_RMW_IMPLEMENTATION="${HOST_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
IMAGE_NAME="ece191/camera:humble"

usage() {
    cat <<'EOF'
Usage:
  ./launch_camera_foxy_host.sh [topic]

Arguments:
  [topic]          Host topic to verify after startup. Default: /oak/rgb/image_raw

Environment overrides:
  HOST_ROS_DISTRO     Host ROS distro to source before running ros2 commands (default: humble)
  CAMERA_TOPIC        Topic to verify from the host (default: /oak/rgb/image_raw)
  STARTUP_TIMEOUT     Seconds to wait for the topic to appear on the host (default: 30)
  HZ_TIMEOUT          Seconds to sample ros2 topic hz on the host (default: 10)
  BUILD_POLICY        missing | always | never (default: always)
  USE_ARM64_OVERRIDE  auto | yes | no (default: auto)
  CAMERA_CONTAINER_NAME  Container name to use for the detached camera service
                         (default: last-try-camera)
  HOST_RMW_IMPLEMENTATION  Host middleware used for verification commands
                           (default: rmw_cyclonedds_cpp)
EOF
}

if [[ $# -gt 1 ]]; then
    usage >&2
    exit 1
fi

if [[ $# -eq 1 ]]; then
    CAMERA_TOPIC="$1"
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

for numeric_var in STARTUP_TIMEOUT HZ_TIMEOUT; do
    if [[ ! "${!numeric_var}" =~ ^[1-9][0-9]*$ ]]; then
        echo "ERROR: ${numeric_var} must be a positive integer." >&2
        exit 1
    fi
done

for required_cmd in docker timeout grep awk uname; do
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
    echo "[INFO] No host ROS installation found; starting camera container without ROS topic verification."
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
export RMW_IMPLEMENTATION="${HOST_RMW_IMPLEMENTATION}"

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
        --build-arg ROS_DISTRO=humble \
        --build-arg ROS_BASE_IMAGE=arm64v8/ros:humble-ros-base-jammy \
        -t "${IMAGE_NAME}" \
    -f Dockerfile.camera \
        .
}

docker_run() {
    docker run -d \
        --name "${CAMERA_CONTAINER_NAME}" \
        --network host \
        --privileged \
        -e DISPLAY="${DISPLAY:-}" \
        -e ROS_DISTRO=humble \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        -e RMW_IMPLEMENTATION="${HOST_RMW_IMPLEMENTATION}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        -v /sys:/sys:ro \
        -v /run/udev:/run/udev:ro \
        "${IMAGE_NAME}" \
        /bin/bash -lc 'bash /ws/run.sh'
}

container_name="${CAMERA_CONTAINER_NAME}"
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
echo "Host verify topic: ${CAMERA_TOPIC}"
echo "Container name: ${container_name}"

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
        compose_cmd build camera
    else
        docker_build
    fi
else
    echo "Skipping image build (${BUILD_POLICY})."
fi

if docker ps -a --format '{{.Names}}' | grep -qx "${container_name}"; then
    echo "Removing existing container with name ${container_name}..."
    docker rm -f "${container_name}" >/dev/null 2>&1 || true
fi

echo "Starting camera publisher container: ${container_name}"
if docker compose version >/dev/null 2>&1 || command -v docker-compose >/dev/null 2>&1; then
    compose_cmd run --rm -d --name "${container_name}" \
        camera /bin/bash -lc "/ws/run.sh" >/dev/null
else
    if docker ps -a --format '{{.Names}}' | grep -qx "${container_name}"; then
        docker rm -f "${container_name}" >/dev/null 2>&1 || true
    fi
    docker_run >/dev/null
fi
container_started=1

if [[ ${HOST_ROS_AVAILABLE} -eq 1 ]]; then
    echo "Waiting up to ${STARTUP_TIMEOUT}s for ${CAMERA_TOPIC} to appear on the host..."
    deadline=$((SECONDS + STARTUP_TIMEOUT))
    while true; do
        if ros2 topic list 2>/dev/null | grep -qx "${CAMERA_TOPIC}"; then
            break
        fi

        if ! docker ps --format '{{.Names}}' | grep -qx "${container_name}"; then
            echo "ERROR: Camera container exited before ${CAMERA_TOPIC} appeared." >&2
            echo "Recent container logs:" >&2
            docker logs --tail 50 "${container_name}" >&2 || true
            exit 1
        fi

        if (( SECONDS >= deadline )); then
            echo "ERROR: Timed out waiting for ${CAMERA_TOPIC} on the host." >&2
            echo "Recent container logs:" >&2
            docker logs --tail 50 "${container_name}" >&2 || true
            exit 1
        fi

        sleep 2
    done

    echo "Topic discovered on the host. Sampling publish rate with ros2 topic hz..."
    hz_output="$(timeout "${HZ_TIMEOUT}" ros2 topic hz "${CAMERA_TOPIC}" 2>/dev/null || true)"
    avg_hz="$(printf '%s\n' "${hz_output}" | awk '/average rate:/{rate=$NF} END{print rate}')"

    if [[ -z "${avg_hz}" ]]; then
        echo "WARNING: ros2 topic hz did not observe any camera messages within ${HZ_TIMEOUT}s." >&2
        echo "Continuing because the topic is visible and the container is still publishing." >&2
        echo "Recent container logs:" >&2
        docker logs --tail 20 "${container_name}" >&2 || true
    else
        printf 'Camera publish rate: %s Hz\n' "${avg_hz}"
    fi
else
    echo "Camera container started. Verify /oak/rgb/image_raw from Foxglove or from a ROS-enabled machine if needed."
fi
echo ""
echo "Camera container is running as ${container_name}."
echo "Press Ctrl-C or close this terminal to stop it and clean it up."

while docker ps --format '{{.Names}}' | grep -qx "${container_name}"; do
    sleep 1
done

echo "Camera container exited."
