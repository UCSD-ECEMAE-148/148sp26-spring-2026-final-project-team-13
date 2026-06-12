# ros2_depthai_package

ROS 2 package that publishes RGB frames from a Luxonis DepthAI/OAK camera as `sensor_msgs/msg/Image`.

## What This Package Provides

- Node: `camera_publisher`
- Executable: `ros2 run ros2_depthai_package camera_publisher`
- Default config: `config/camera.yaml`
- Output: RGB images (`bgr8`) on a configurable topic
- Auto-reconnect: if the camera disconnects, the node keeps retrying

## Prerequisites

- ROS 2 installed and sourced (`$ROS_DISTRO`)
- Colcon workspace
- A supported DepthAI/OAK camera connected over USB
- Python package `depthai` available in your ROS environment

Install missing ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-rclpy \
  ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-cv-bridge \
  python3-yaml
```

Install DepthAI Python package (if not already installed):

```bash
python3 -m pip install depthai
```

## Build

From your workspace root:

```bash
colcon build --packages-select ros2_depthai_package
source install/setup.bash
```

## Run

Run with defaults:

```bash
ros2 run ros2_depthai_package camera_publisher
```

## Parameters

Default values come from `config/camera.yaml` (or in-code fallbacks if config is not found).

| Name | Type | Default | Notes |
|---|---|---|---|
| `topic_name` | string | `/oak/rgb/image_raw` | Output topic |
| `fps` | double | `30.0` | Must be `> 0` |
| `width` | int | `640` | Preview width, must be `> 0` |
| `height` | int | `400` | Preview height, must be `> 0` |
| `frame_id` | string | `oak_rgb_camera_frame` | Message header frame ID |
| `anti_banding_mode` | string | `mains_60_hz` | One of: `off`, `auto`, `mains_50_hz`, `mains_60_hz` |
| `qos_reliability` | string | `reliable` | `reliable` or `best_effort` |
| `qos_depth` | int | `10` | QoS queue depth, must be `> 0` |

Override parameters from CLI:

```bash
ros2 run ros2_depthai_package camera_publisher --ros-args \
  -p fps:=15.0 \
  -p width:=1280 \
  -p height:=720 \
  -p anti_banding_mode:=auto \
  -p qos_reliability:=best_effort
```

Use a parameter file:

```bash
ros2 run ros2_depthai_package camera_publisher --ros-args \
  --params-file $(ros2 pkg prefix ros2_depthai_package)/share/ros2_depthai_package/config/camera.yaml
```

## Published Topic

- `topic_name` (default: `/oak/rgb/image_raw`)
  - Type: `sensor_msgs/msg/Image`
  - Encoding: `bgr8`
  - `header.frame_id`: `frame_id` parameter

## Quick Verification

In another terminal (with ROS sourced):

```bash
ros2 topic list | grep oak
ros2 topic hz /oak/rgb/image_raw
ros2 topic echo /oak/rgb/image_raw --once
```

## Troubleshooting

- Camera not detected:
  - Check USB cable/power and reconnect the device.
  - The node retries connection automatically every ~2 seconds.
- Topic exists but no subscribers receive data:
  - Match subscriber QoS to `qos_reliability` and depth.
- Parameter errors on startup:
  - Verify value ranges and allowed enums listed above.
