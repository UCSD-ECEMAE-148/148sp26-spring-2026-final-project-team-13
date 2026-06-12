import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("livox_ros_driver2").find("livox_ros_driver2")
    user_config_path = os.path.join(package_share, "config", "MID360_config.json")

    livox_ros2_params = [
        {"xfer_format": 0},  # 0 = PointCloud2, 1 = Livox CustomMsg
        {"multi_topic": 0},
        {"data_src": 0},
        {"publish_freq": 10.0},
        {"output_data_type": 0},
        {"frame_id": "livox_frame"},
        {"lvx_file_path": "/home/livox/livox_test.lvx"},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": "livox0000000001"},
    ]

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=livox_ros2_params,
    )

    return LaunchDescription([livox_driver])
