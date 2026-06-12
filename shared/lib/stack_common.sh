#!/usr/bin/env bash
# Shared helpers for sensorfusion_ws launch scripts.

stack_root() {
    local lib_dir
    lib_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    cd "${lib_dir}/.." && pwd
}

stack_source_ros() {
    set +u
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        # shellcheck source=/dev/null
        source /opt/ros/humble/setup.bash
    elif [[ -f /opt/ros/foxy/setup.bash ]]; then
        # shellcheck source=/dev/null
        source /opt/ros/foxy/setup.bash
    else
        return 1
    fi
    set -u
    return 0
}

stack_is_arm64() {
    local arch
    arch="$(uname -m)"
    [[ "$arch" =~ ^(aarch64|arm64)$ ]]
}

stack_compose_files() {
    local root="$1"
    local component="$2"
    local files=(-f "${root}/${component}/docker-compose.yml")
    if stack_is_arm64 && [[ -f "${root}/${component}/docker-compose.arm64.yml" ]]; then
        files+=(-f "${root}/${component}/docker-compose.arm64.yml")
    fi
    printf '%s\n' "${files[@]}"
}

stack_compose_cmd() {
    local -a files=()
    while [[ $# -gt 1 ]]; do
        files+=("$1")
        shift
    done
    local subcommand="$1"

    if docker compose version >/dev/null 2>&1; then
        docker compose "${files[@]}" "$subcommand" "${@:2}"
    elif command -v docker-compose >/dev/null 2>&1; then
        docker-compose "${files[@]}" "$subcommand" "${@:2}"
    else
        echo "ERROR: docker compose is not available." >&2
        return 1
    fi
}

stack_wait_for_topic() {
    local topic="$1"
    local timeout="$2"
    local label="$3"
    local deadline=$((SECONDS + timeout))

    while (( SECONDS < deadline )); do
        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            echo "[OK]    ${label} is publishing on ${topic}."
            return 0
        fi
        sleep 2
    done

    echo "[ERROR] Timed out waiting for ${topic} (${label})." >&2
    return 1
}

stack_livox_host_ip() {
    local sensor_id="${1:-}"
    local root
    root="$(stack_root)"
    bash "${root}/detect_livox_host_ip.sh" "$sensor_id"
}

stack_livox_ip_assigned() {
    local host_ip="$1"
    ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | grep -qx "$host_ip"
}

stack_primary_ip() {
    hostname -I 2>/dev/null | awk '{print $1}'
}

stack_wifi_ip() {
    ip -4 -o addr show scope global 2>/dev/null \
        | awk '{print $4}' \
        | cut -d/ -f1 \
        | grep -Ev '^192\.168\.1\.' \
        | head -n1
}

stack_lidar_subnet_ip() {
    ip -4 -o addr show scope global 2>/dev/null \
        | awk '{print $4}' \
        | cut -d/ -f1 \
        | grep -E '^192\.168\.1\.' \
        | head -n1
}

stack_wait_for_port() {
    local port="$1"
    local timeout="$2"
    local label="$3"
    local deadline=$((SECONDS + timeout))

    while (( SECONDS < deadline )); do
        if ss -tln 2>/dev/null | awk '{print $4}' | grep -q ":${port}\$"; then
            echo "[OK]    ${label} is listening on port ${port}."
            return 0
        fi
        sleep 1
    done

    echo "[ERROR] Timed out waiting for port ${port} (${label})." >&2
    return 1
}

stack_build_camera_image() {
    local root="$1"
    docker build \
        --build-arg ROS_DISTRO=humble \
        --build-arg ROS_BASE_IMAGE=arm64v8/ros:humble-ros-base-jammy \
        -t ece191/camera:humble \
        -f "${root}/camera/Dockerfile.camera" \
        "${root}/camera"
}

stack_build_lidar_image() {
    local root="$1"
    docker build \
        --build-arg ROS_BASE_IMAGE=arm64v8/ros:humble-perception-jammy \
        -t ece191/lidar:humble \
        -f "${root}/camera/Dockerfile.lidar" \
        "${root}/lidar"
}

stack_build_sensor_images() {
    local root="$1"
    if stack_has_compose; then
        mapfile -t compose_files < <(stack_compose_files "$root" "camera")
        stack_compose_cmd "${compose_files[@]}" build
        return
    fi
    stack_build_camera_image "$root"
    stack_build_lidar_image "$root"
}

stack_ensure_fusion_runtime_deps() {
    local container="$1"
    local install_detection="${2:-0}"
    local use_hailo="${3:-0}"
    docker exec "$container" bash -lc '
        set -e
        missing=0
        dpkg -s ros-humble-rmw-cyclonedds-cpp >/dev/null 2>&1 || missing=1
        dpkg -s ros-humble-foxglove-bridge >/dev/null 2>&1 || missing=1
        if [[ "$missing" -eq 1 ]]; then
            apt-get update -qq
            apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp ros-humble-foxglove-bridge
        fi
    '
    if [[ "$install_detection" == "1" && "$use_hailo" == "1" ]]; then
        stack_ensure_hailo_runtime_deps "$container"
    fi
    if [[ "$install_detection" == "1" ]]; then
        docker exec "$container" bash -lc '
            set -e
            source /opt/ros/humble/setup.bash
            deps_ok=0
            if python3 -c "import numpy; assert tuple(int(x) for x in numpy.__version__.split(\".\")[:2]) < (2, 0)" 2>/dev/null \
                && python3 -c "from cv_bridge import CvBridge" 2>/dev/null \
                && python3 -c "import ultralytics" 2>/dev/null; then
                deps_ok=1
            fi
            if [[ "$deps_ok" -eq 1 ]]; then
                exit 0
            fi
            echo "[INFO] Installing CPU-only PyTorch + ultralytics for YOLO (numpy<2 for cv_bridge)..."
            pip3 install --no-cache-dir "numpy<2"
            pip3 install --no-cache-dir torch torchvision \
                --index-url https://download.pytorch.org/whl/cpu
            pip3 install --no-cache-dir "numpy<2" "opencv-python<4.10" ultralytics
            python3 -c "from cv_bridge import CvBridge; import ultralytics; print(\"[OK] YOLO deps ready\")"
        '
    fi
}

stack_rebuild_fusion_package() {
    local container="$1"
    docker exec "$container" bash -lc '
        source /opt/ros/humble/setup.bash
        cd /ros2_ws
        colcon build --packages-select ros2_camera_lidar_fusion
    ' >/dev/null
}

stack_start_fusion_launch() {
    local container="$1"
    local fusion_mode="$2"
    local foxglove_port="$3"
    local enable_detection="${4:-0}"
    local detection_model="${5:-yolov8n.pt}"
    local detection_backend="${6:-auto}"

    docker exec "$container" bash -lc "
        pkill -9 -f 'sensor_fusion.launch' 2>/dev/null || true
        pkill -9 -f 'ros2 launch ros2_camera_lidar_fusion' 2>/dev/null || true
        pkill -9 -f foxglove_bridge 2>/dev/null || true
        pkill -9 -f lidar_camera_projection 2>/dev/null || true
        pkill -9 -f static_transform_publisher 2>/dev/null || true
        pkill -9 -f hailo_detect_bridge 2>/dev/null || true
        sleep 2
    " >/dev/null 2>&1 || true

    local -a env_args=(
        -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
        -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    )
    if [[ "$enable_detection" == "1" ]]; then
        env_args+=(
            -e "SENSORFUSION_ENABLE_DETECTION=1"
            -e "SENSORFUSION_DETECTION_MODEL=${detection_model}"
            -e "SENSORFUSION_DETECTION_BACKEND=${detection_backend}"
        )
    fi

    docker exec "${env_args[@]}" -d "$container" /bin/bash -lc "
        source /opt/ros/humble/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
        ros2 launch ros2_camera_lidar_fusion sensor_fusion.launch.py \
            fusion_mode:=${fusion_mode} \
            foxglove_port:=${foxglove_port} \
            > /tmp/fusion_launch.log 2>&1
    "
}

stack_has_compose() {
    docker compose version >/dev/null 2>&1 || command -v docker-compose >/dev/null 2>&1
}

stack_oak_usb_present() {
    lsusb 2>/dev/null | grep -qi '03e7:'
}

stack_hailo_device_present() {
    [[ -e /dev/hailo0 ]]
}

stack_hailo_arch() {
    if ! stack_hailo_device_present; then
        return 1
    fi
    local out
    out="$(hailortcli fw-control identify 2>/dev/null | tr -d '\0' || true)"
    if grep -qi 'hailo-8l\|hailo8l' <<<"$out"; then
        echo "hailo8l"
        return 0
    fi
    if grep -qi 'hailo-8\|hailo8' <<<"$out"; then
        echo "hailo8"
        return 0
    fi
    return 1
}

stack_default_hailo_hef() {
    local arch="${1:-$(stack_hailo_arch || echo hailo8)}"
    local model_dir="/usr/share/hailo-models"
    if [[ "$arch" == "hailo8l" ]]; then
        [[ -f "${model_dir}/yolov8s_h8l.hef" ]] && echo "${model_dir}/yolov8s_h8l.hef" && return 0
    else
        [[ -f "${model_dir}/yolov8s_h8.hef" ]] && echo "${model_dir}/yolov8s_h8.hef" && return 0
        [[ -f "${model_dir}/yolov6n_h8.hef" ]] && echo "${model_dir}/yolov6n_h8.hef" && return 0
    fi
    return 1
}

stack_ensure_hailo_runtime_deps() {
    local container="$1"
    docker exec "$container" bash -lc '
        set -e
        if python3.11 -c "from hailo_platform import VDevice" 2>/dev/null; then
            exit 0
        fi
        echo "[INFO] Installing Hailo NPU runtime (Python 3.11 + hailort)..."
        apt-get update -qq
        apt-get install -y -qq python3.11 python3.11-venv python3-netaddr python3-future wget ca-certificates

        cd /tmp
        if ! dpkg -s hailort >/dev/null 2>&1; then
            wget -q -O hailort.deb \
                "http://archive.raspberrypi.com/debian/pool/main/h/hailort/hailort_4.20.0-1_arm64.deb"
            dpkg -i hailort.deb || apt-get install -y -f -qq
        fi
        if ! dpkg -s python3-hailort >/dev/null 2>&1; then
            wget -q -O python3-hailort.deb \
                "http://archive.raspberrypi.com/debian/pool/main/p/pyhailort/python3-hailort_4.20.0-1_arm64.deb"
            apt-get install -y -qq python3-contextlib2 || true
            dpkg --force-depends -i python3-hailort.deb || true
        fi
        # Do not run apt-get -f here; it removes python3-hailort on Ubuntu 22.04 (python3=3.10).
        if [[ ! -d /usr/lib/python3/dist-packages/hailo_platform/pyhailort ]]; then
            dpkg-deb -x python3-hailort.deb /tmp/hailo_extract
            cp -a /tmp/hailo_extract/usr/lib/python3/dist-packages/hailo_platform \
                /usr/lib/python3/dist-packages/
        fi
        python3.11 -m pip install -q --force-reinstall "numpy<2"
        python3.11 -m pip install -q "opencv-python-headless<4.10" "numpy<2"
        python3.11 -c "from hailo_platform import VDevice; import cv2; print(\"[OK] Hailo NPU ready\")"
    '
}

stack_check_oak_usb_or_warn() {
    if stack_oak_usb_present; then
        return 0
    fi
    echo "[WARN]  No OAK-D / DepthAI device on USB (expected: ID 03e7:xxxx)." >&2
    echo "        Plug the OAK-D into a USB 3.0 port, then run: lsusb | grep 03e7" >&2
    echo "        Current USB devices:" >&2
    lsusb 2>/dev/null | sed 's/^/          /' >&2
    return 1
}

stack_start_camera() {
    local root="$1"
    local container="$2"
    local image="$3"

    docker ps -a --format '{{.Names}}' | grep -qx "$container" \
        && docker rm -f "$container" >/dev/null 2>&1 || true

    if stack_has_compose; then
        mapfile -t compose_files < <(stack_compose_files "$root" "camera")
        stack_compose_cmd "${compose_files[@]}" run --rm -d --name "$container" \
            camera /bin/bash -lc '/ws/run.sh'
        return
    fi

    docker run -d \
        --name "$container" \
        --network host \
        --privileged \
        -e DISPLAY="${DISPLAY:-}" \
        -e ROS_DISTRO=humble \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        -v /sys:/sys:ro \
        -v /run/udev:/run/udev:ro \
        -v "${root}/camera/ece191-ros2-depthai-camera/ros2_depthai_package/config/camera.yaml:/ws/src/ece191-ros2-depthai-camera/ros2_depthai_package/config/camera.yaml:ro" \
        "$image" \
        /bin/bash -lc '/ws/run.sh'
}

stack_start_lidar() {
    local root="$1"
    local container="$2"
    local image="$3"
    local sensor_target="$4"
    local host_ip="$5"

    docker ps -a --format '{{.Names}}' | grep -qx "$container" \
        && docker rm -f "$container" >/dev/null 2>&1 || true

    local -a mount_args=(
        -v "${root}/lidar/ece191-ros2-livox-lidar/launch/run.sh:/home/devuser/livox_ws/src/run.sh:ro"
        -v "${root}/lidar/ece191-ros2-livox-lidar/launch/pointcloud_MID360_launch.py:/home/devuser/livox_ws/src/pointcloud_MID360_launch.py:ro"
    )

    local lidar_cmd="bash /home/devuser/livox_ws/src/run.sh ${sensor_target} ros2 launch /home/devuser/livox_ws/src/pointcloud_MID360_launch.py"

    if stack_has_compose; then
        mapfile -t compose_files < <(stack_compose_files "$root" "lidar")
        stack_compose_cmd "${compose_files[@]}" run --rm -d --name "$container" \
            -e "LIVOX_HOST_IP=${host_ip}" \
            "${mount_args[@]}" \
            lidar /bin/bash -lc "$lidar_cmd"
        return
    fi

    docker run -d \
        --name "$container" \
        --network host \
        --privileged \
        -e DISPLAY="${DISPLAY:-}" \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
        -e "LIVOX_HOST_IP=${host_ip}" \
        "${mount_args[@]}" \
        "$image" \
        /bin/bash -lc "$lidar_cmd"
}

stack_wait_for_topic_in_container() {
    local container="$1"
    local topic="$2"
    local timeout="$3"
    local label="$4"
    local deadline=$((SECONDS + timeout))

    while (( SECONDS < deadline )); do
        local pub_count
        pub_count="$(docker exec "$container" bash -lc \
            'source /opt/ros/humble/setup.bash 2>/dev/null
             source /ws/install/setup.bash 2>/dev/null || true
             source /home/devuser/livox_ws/install/setup.bash 2>/dev/null || true
             source /ros2_ws/install/setup.bash 2>/dev/null || true
             export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
             ros2 topic info "'"$topic"'" 2>/dev/null | awk "/Publisher count:/ {print \$3}"' \
            || true)"
        if [[ "${pub_count:-0}" =~ ^[1-9][0-9]*$ ]]; then
            echo "[OK]    ${label} is publishing on ${topic}."
            return 0
        fi
        sleep 2
    done

    echo "[ERROR] Timed out waiting for ${topic} (${label})." >&2
    return 1
}
