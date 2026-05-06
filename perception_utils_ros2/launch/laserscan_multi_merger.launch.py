"""!
@file laserscan_multi_merger.launch.py
@author Valerio Passamano
@brief Launches a multi-laserscan merger configuration for two lidar scan topics.

This launch file starts the `laserscan_multi_merger` node, 
configures the front and back lidar scan topics as inputs,
and publishes a fused scan aligned to the `VLP16_lidar_back` frame.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    destination_frame_arg = DeclareLaunchArgument(
        "destination_frame",
        default_value="VLP16_lidar_back",
        description="Target frame used to merge all incoming laser scans",
    )
    cloud_destination_topic_arg = DeclareLaunchArgument(
        "cloud_destination_topic",
        default_value="/merged_cloud",
        description="Output topic for the merged point cloud",
    )
    scan_destination_topic_arg = DeclareLaunchArgument(
        "scan_destination_topic",
        default_value="/scan",
        description="Output topic for the merged laser scan",
    )
    laserscan_topics_arg = DeclareLaunchArgument(
        "laserscan_topics",
        default_value="/VLP16_lidar_back/scan /VLP16_lidar_front/scan",
        description="Whitespace-separated list of input laser scan topics",
    )

    laserscan_multi_merger = Node(
        package="perception_utils_ros2",
        executable="laserscan_multi_merger",
        name="laserscan_multi_merger",
        output="screen",
        parameters=[
            {"destination_frame": LaunchConfiguration("destination_frame")},
            {"cloud_destination_topic": LaunchConfiguration("cloud_destination_topic")},
            {"scan_destination_topic": LaunchConfiguration("scan_destination_topic")},
            {"laserscan_topics": LaunchConfiguration("laserscan_topics")},
            {"angle_min": -3.1416},
            {"angle_max": 3.1416},
            {"angle_increment": 0.007},
            {"scan_time": 0.2},
            {"range_min": 0.9},
            {"range_max": 130.0}
        ]
    )

    return LaunchDescription([
        destination_frame_arg,
        cloud_destination_topic_arg,
        scan_destination_topic_arg,
        laserscan_topics_arg,
        laserscan_multi_merger
    ])
