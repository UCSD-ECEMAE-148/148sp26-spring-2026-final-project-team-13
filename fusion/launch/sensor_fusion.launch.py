"""Launch fusion processing nodes together with the Foxglove bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("fusion_mode").perform(context)
    foxglove_port = LaunchConfiguration("foxglove_port").perform(context)
    max_update_rate = LaunchConfiguration("foxglove_max_update_rate").perform(context)

    if mode == "detection":
        fusion_executable = "lidar_camera_projection_detection"
        fusion_name = "lidar_camera_projection_detection"
    else:
        fusion_executable = "lidar_camera_projection"
        fusion_name = "lidar_camera_projection"

    foxglove_share = get_package_share_directory("foxglove_bridge")
    foxglove_launch = os.path.join(foxglove_share, "launch", "foxglove_bridge_launch.xml")

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="livox_to_camera_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "livox_frame",
                "--child-frame-id", "oak_rgb_camera_frame",
            ],
        ),
        Node(
            package="ros2_camera_lidar_fusion",
            executable=fusion_executable,
            name=fusion_name,
            output="screen",
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(foxglove_launch),
            launch_arguments={
                "port": foxglove_port,
                "max_update_rate": max_update_rate,
                "send_buffer_limit": "10000000",
                "num_threads": "4",
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fusion_mode",
                default_value="projection",
                description="Fusion mode: projection or detection",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="Foxglove WebSocket port",
            ),
            DeclareLaunchArgument(
                "foxglove_max_update_rate",
                default_value="7.0",
                description="Foxglove max update rate (Hz)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
