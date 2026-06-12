# ece191-ros2-depthai-camera

ROS 2 workspace for publishing RGB images from a Luxonis DepthAI/OAK camera using the `ros2_depthai_package` package.

## Overview

This repository is organized as a ROS 2 colcon workspace root. The main package is:

- `ros2_depthai_package`: Python ROS 2 node that publishes `sensor_msgs/msg/Image` frames from a connected DepthAI camera.

For package-level details (parameters, troubleshooting, examples), see:

- `ros2_depthai_package/README.md`

## Repository Layout

```text
.
├── ros2_depthai_package/     # ROS 2 Python package
├── build/                    # Colcon build artifacts (generated)
├── install/                  # Colcon install artifacts (generated)
└── log/                      # Colcon logs (generated)
```

## Prerequisites

- Ubuntu with ROS 2 installed and sourced (for example, Humble/Iron/Jazzy).
- `colcon` build tools.
- A supported Luxonis DepthAI/OAK USB camera.
- Python package `depthai` available in the same environment used to run ROS 2.

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  ros-$ROS_DISTRO-rclpy \
  ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-cv-bridge \
  python3-yaml
python3 -m pip install --user depthai
```

## Quick Start

From the workspace root (this directory):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ros2_depthai_package
source install/setup.bash
ros2 run ros2_depthai_package camera_publisher
```

## Runtime Verification

In another terminal:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic list | grep oak
ros2 topic hz /oak/rgb/image_raw
```

## Configuration

Default parameters are in:

- `ros2_depthai_package/config/camera.yaml`

You can override parameters at runtime:

```bash
ros2 run ros2_depthai_package camera_publisher --ros-args \
  -p fps:=15.0 \
  -p width:=1280 \
  -p height:=720 \
  -p qos_reliability:=best_effort
```

## Development

Run lint/tests for the package:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon test --packages-select ros2_depthai_package
colcon test-result --verbose
```

## Notes

- `build/`, `install/`, and `log/` are generated workspace directories.
- Package metadata (`description`, `license`) in `ros2_depthai_package/package.xml` is currently placeholder text and should be updated before release.
