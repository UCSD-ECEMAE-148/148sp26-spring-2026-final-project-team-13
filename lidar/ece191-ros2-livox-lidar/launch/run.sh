#!/bin/bash
set -e

livox_host_ip_from_subnet() {
    ip -4 -o addr show scope global 2>/dev/null \
        | awk '{print $4}' \
        | cut -d/ -f1 \
        | grep -E '^192\.168\.1\.[0-9]+$' \
        | head -n1
}

livox_host_ip_is_local() {
    local candidate="$1"
    ip -4 -o addr show scope global 2>/dev/null \
        | awk '{print $4}' \
        | cut -d/ -f1 \
        | grep -qx "$candidate"
}

# Change MID360 sensor IP address if provided.
# Accepts: serial suffix (e.g. 99 -> 192.168.1.199), full IP (192.168.1.3),
# or LIVOX_SENSOR_IP env var.
if [[ -n "${LIVOX_SENSOR_IP:-}" ]]; then
    sensor_ip="${LIVOX_SENSOR_IP}"
elif [[ $1 =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    sensor_ip="$1"
elif [[ $1 =~ ^[0-9]+$ ]]; then
    sensor_ip="192.168.1.1$1"
fi

if [[ -n "${sensor_ip:-}" ]]; then
    host_ip="${LIVOX_HOST_IP:-}"
    if [[ -z "$host_ip" ]]; then
        host_ip="$(livox_host_ip_from_subnet)"
    fi
    if [[ -z "$host_ip" ]]; then
        route_ip="$(ip route get "$sensor_ip" 2> /dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") { print $(i+1); exit }}')"
        if [[ -n "$route_ip" && "$route_ip" =~ ^192\.168\.1\. ]]; then
            host_ip="$route_ip"
        fi
    fi
    if [[ -z "$host_ip" ]]; then
        host_ip="192.168.1.50"
    fi

    if ! livox_host_ip_is_local "$host_ip"; then
        echo "ERROR: Livox host bind IP ${host_ip} is not assigned to any local interface." >&2
        echo "       The Livox SDK cannot bind UDP sockets without this address." >&2
        echo "       Fix: sudo bash setup_livox_network.sh eth0 ${host_ip}" >&2
        echo "       (setup script: shared/setup_livox_network.sh in sensorfusion_ws)" >&2
        echo "       Or export LIVOX_HOST_IP=<your-lidar-facing-ip> before launching." >&2
        echo "       Sensor target IP: ${sensor_ip}" >&2
        ip -br -4 addr show >&2 || true
        exit 1
    fi

    if [[ ! "$host_ip" =~ ^192\.168\.1\. ]]; then
        echo "ERROR: Livox MID-360 requires a host bind IP on 192.168.1.x (got ${host_ip})." >&2
        echo "       Configure the Ethernet port wired to the LiDAR, not WiFi." >&2
        echo "       Fix: sudo bash setup_livox_network.sh eth0 192.168.1.50" >&2
        exit 1
    fi

    echo "[livox] sensor_ip=${sensor_ip} host_bind_ip=${host_ip}"

    config_files=(
        /home/devuser/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
        /home/devuser/livox_ws/src/livox_ros_driver2/config/MID360_config.json
    )

    for config_file in "${config_files[@]}"; do
        if [[ -f "$config_file" ]]; then
            sed -i -E "s/(\"ip\"[[:space:]]*:[[:space:]]*\")([0-9]{1,3}\\.){3}[0-9]{1,3}(\")/\1$sensor_ip\3/" "$config_file"

            if [[ -n "$host_ip" ]]; then
                sed -i -E "s/(\"cmd_data_ip\"[[:space:]]*:[[:space:]]*\")([0-9]{1,3}\\.){3}[0-9]{1,3}(\")/\1$host_ip\3/" "$config_file"
                sed -i -E "s/(\"push_msg_ip\"[[:space:]]*:[[:space:]]*\")([0-9]{1,3}\\.){3}[0-9]{1,3}(\")/\1$host_ip\3/" "$config_file"
                sed -i -E "s/(\"point_data_ip\"[[:space:]]*:[[:space:]]*\")([0-9]{1,3}\\.){3}[0-9]{1,3}(\")/\1$host_ip\3/" "$config_file"
                sed -i -E "s/(\"imu_data_ip\"[[:space:]]*:[[:space:]]*\")([0-9]{1,3}\\.){3}[0-9]{1,3}(\")/\1$host_ip\3/" "$config_file"
            fi
        fi
    done
    if [[ $1 =~ ^[0-9]+$ || $1 =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        shift
    fi
fi

# Source the necessary ROS2 and workspaces
source /opt/ros/humble/setup.bash
source /home/devuser/livox_ws/install/setup.bash
# This will fail in the base livox container since it doesn't use that workspace. Hence I send the error to /dev/null.
source /home/devuser/ros2_ws/install/setup.bash 2> /dev/null || true
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
# Run the provided command
"$@"
