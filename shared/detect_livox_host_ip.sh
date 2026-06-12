#!/usr/bin/env bash
# Detect the host IP the Livox MID-360 driver should bind to.
# Prints the chosen IP to stdout. Exit 0 on success.
#
# Usage: detect_livox_host_ip.sh [sensor-id]
#   sensor-id  Optional last-two-digit Livox serial suffix for route probing.
#
# Override with LIVOX_HOST_IP env var.

set -euo pipefail

SENSOR_SUFFIX="${1:-}"

if [[ -n "${LIVOX_HOST_IP:-}" ]]; then
    printf '%s\n' "$LIVOX_HOST_IP"
    exit 0
fi

# Prefer an address already assigned on the 192.168.1.x LiDAR subnet.
while IFS= read -r addr; do
    if [[ -n "$addr" ]]; then
        printf '%s\n' "$addr"
        exit 0
    fi
done < <(ip -4 -o addr show scope global 2>/dev/null \
    | awk '{print $4}' \
    | cut -d/ -f1 \
    | grep -E '^192\.168\.1\.[0-9]+$' || true)

# Probe routing toward the configured sensor IP.
if [[ "$SENSOR_SUFFIX" =~ ^[0-9]+$ ]]; then
    sensor_ip="192.168.1.1${SENSOR_SUFFIX}"
    route_ip="$(ip route get "$sensor_ip" 2>/dev/null \
        | awk '{for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit }}')"
    if [[ -n "$route_ip" && "$route_ip" =~ ^192\.168\.1\. ]]; then
        printf '%s\n' "$route_ip"
        exit 0
    fi
fi

# Livox MID-360 default host-side address on the dedicated Ethernet link.
printf '%s\n' "192.168.1.50"
