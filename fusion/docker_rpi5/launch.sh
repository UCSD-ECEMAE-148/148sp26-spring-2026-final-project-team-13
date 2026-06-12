#!/bin/bash
# Pi 5 fusion node launcher — mirrors fusion/docker/launch.sh on Jetson.
# Run inside the fusion container:
#   docker exec -it ros2_camera_lidar_fusion_rpi5 bash
#   bash /ros2_ws/src/ros2_camera_lidar_fusion/docker_rpi5/launch.sh <number>
#
# Or from the host after the fusion container is running:
#   docker exec -it ros2_camera_lidar_fusion_rpi5 bash -lc \
#     'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
#      bash /ros2_ws/src/ros2_camera_lidar_fusion/docker_rpi5/launch.sh 6'

set -e

PACKAGE="ros2_camera_lidar_fusion"

if [ -z "${1:-}" ]; then
  echo "Usage: $0 <number>"
  echo ""
  echo "Available nodes:"
  echo "  1. get_intrinsic_camera_calibration  - Computes intrinsic camera calibration"
  echo "  2. save_sensor_data.py               - Records synchronized camera and LiDAR data"
  echo "  3. extract_points.py                 - Manual selection of camera/LiDAR point pairs"
  echo "  4. get_extrinsic_camera_calibration  - Computes extrinsic camera/LiDAR calibration"
  echo "  5. lidar_camera_projection           - Projects LiDAR points onto camera image"
  echo "  6. lidar_camera_projection_detection - Projection + YOLO + HSV cone detection"
  exit 1
fi

case "$1" in
  1) NODE="get_intrinsic_camera_calibration" ;;
  2) NODE="save_data" ;;
  3) NODE="extract_points" ;;
  4) NODE="get_extrinsic_camera_calibration" ;;
  5) NODE="lidar_camera_projection" ;;
  6)
    NODE="lidar_camera_projection_detection"
    export SENSORFUSION_ENABLE_DETECTION="${SENSORFUSION_ENABLE_DETECTION:-1}"
    export SENSORFUSION_DETECTION_BACKEND="${SENSORFUSION_DETECTION_BACKEND:-auto}"
    export SENSORFUSION_DETECTION_MODEL="${SENSORFUSION_DETECTION_MODEL:-/usr/share/hailo-models/yolov8s_h8.hef}"
    ;;
  *)
    echo "Error: invalid selection '$1'. Choose a number 1-6."
    exit 1
    ;;
esac

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

echo "Running: ros2 run $PACKAGE $NODE"
ros2 run "$PACKAGE" "$NODE"
