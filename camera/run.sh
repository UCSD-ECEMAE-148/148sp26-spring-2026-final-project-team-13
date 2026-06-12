#!/usr/bin/env bash
set -euo pipefail

source_without_nounset() {
  # ROS setup scripts reference optional vars before defining them.
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

source_without_nounset /opt/ros/"$ROS_DISTRO"/setup.bash
if [[ ! -f /ws/install/setup.bash ]]; then
  echo "Expected built workspace at /ws/install/setup.bash, but it was not found." >&2
  exit 1
fi
source_without_nounset /ws/install/setup.bash
exec ros2 run ros2_depthai_package camera_publisher
