#!/usr/bin/env bash
cd ~/sensorfusion_ws/lidar
bash launch_lidar_foxy_host.sh "${1:-99}"
