"""!
@file laserscan_multi_merger.launch.py
@author Valerio Passamano
@brief Launches a multi-laserscan merger configuration for two lidar scan topics.

This launch file starts the `laserscan_multi_merger` node from
`ira_laser_tools`, configures the front and back lidar scan topics as inputs,
and publishes a fused scan aligned to the `VLP16_lidar_back` frame.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    laserscan_multi_merger = Node(
        package="perception_utils_ros2",
        executable="laserscan_multi_merger",
        name="laserscan_multi_merger",
        output="screen",
        parameters=[
            {"destination_frame": "VLP16_lidar_back"},
            {"cloud_destination_topic": "/merged_cloud"},
            {"scan_destination_topic": "/scan"},
            {"laserscan_topics": "/VLP16_lidar_back/scan /VLP16_lidar_front/scan"},
            {"angle_min": -3.1416},
            {"angle_max": 3.1416},
            {"angle_increment": 0.007},
            {"scan_time": 0.2},
            {"range_min": 0.9},
            {"range_max": 130.0}
        ]
    )

    return LaunchDescription([
        laserscan_multi_merger
    ])
