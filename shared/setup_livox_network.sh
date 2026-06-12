#!/usr/bin/env bash
# Configure the Pi's LiDAR Ethernet port for Livox MID-360 communication.
#
# Usage: sudo bash setup_livox_network.sh [interface] [host-ip]
#   interface  Network interface wired to the LiDAR (default: eth0)
#   host-ip    Host bind IP for the Livox driver (default: 192.168.1.50)
#
# Makes the address persistent via a NetworkManager connection when nmcli is
# available; otherwise applies a temporary ip(8) address for the current boot.

set -euo pipefail

IFACE="${1:-eth0}"
HOST_IP="${2:-192.168.1.50}"
PREFIX="${LIVOX_NETWORK_PREFIX:-24}"
CON_NAME="livox-mid360"

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
    echo "ERROR: run as root: sudo bash $0 [interface] [host-ip]" >&2
    exit 1
fi

if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "ERROR: interface '$IFACE' not found." >&2
    ip -br link show >&2 || true
    exit 1
fi

carrier="$(cat "/sys/class/net/${IFACE}/carrier" 2>/dev/null || echo 0)"
if [[ "$carrier" != "1" ]]; then
    echo "WARNING: ${IFACE} reports no link (cable unplugged or LiDAR powered off)."
    echo "         Continuing anyway so the bind IP is ready when the link comes up."
fi

if command -v nmcli >/dev/null 2>&1; then
    nmcli dev set "$IFACE" managed yes 2>/dev/null || true
    nmcli con delete "$CON_NAME" >/dev/null 2>&1 || true
    nmcli con add type ethernet ifname "$IFACE" con-name "$CON_NAME" \
        ipv4.method manual \
        ipv4.addresses "${HOST_IP}/${PREFIX}" \
        ipv4.never-default yes \
        ipv6.method ignore \
        connection.autoconnect yes
    nmcli con up "$CON_NAME"
    echo "Configured ${IFACE} with ${HOST_IP}/${PREFIX} via NetworkManager (${CON_NAME})."
else
    ip link set "$IFACE" up
    ip addr flush dev "$IFACE" 2>/dev/null || true
    ip addr add "${HOST_IP}/${PREFIX}" dev "$IFACE"
    echo "Configured ${IFACE} with ${HOST_IP}/${PREFIX} (temporary until reboot)."
    echo "Install NetworkManager or add a systemd-networkd unit for persistence."
fi

echo ""
echo "Verify:"
echo "  ip -4 addr show dev ${IFACE}"
echo "  ping -c 2 192.168.1.1<SENSOR_ID>   # last two digits of Livox serial"
