#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FUSION_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE_TAG="ros2_camera_lidar_fusion:rpi5"
CONTAINER_NAME="ros2_camera_lidar_fusion_rpi5"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  docker rm -f "$CONTAINER_NAME" >/dev/null
fi

docker run \
  --name "$CONTAINER_NAME" \
  -it \
  --net host \
  --ipc host \
  --privileged \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$FUSION_DIR:/ros2_ws/src/ros2_camera_lidar_fusion" \
  -w /ros2_ws \
  "$IMAGE_TAG"