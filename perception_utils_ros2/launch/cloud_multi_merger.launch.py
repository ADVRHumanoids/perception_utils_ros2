"""!
@file cloud_multi_merger.launch.py
@author Valerio Passamano
@brief Launches the point cloud merger node for two lidar point cloud topics.

This launch file starts the `pointcloud_merger` executable from
`perception_utils_ros2` and configures it to transform and merge the
`/VLP16_lidar_back/points` and `/VLP16_lidar_front/points` topics into a
single `/merged_cloud` output in the `VLP16_lidar_back` frame.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    laserscan_multi_merger = Node(
        package="perception_utils_ros2",
        executable="pointcloud_merger",
        name="pointcloud_merger",
        output="screen",
        parameters=[
            {"destination_frame": "base_link"},
            {"cloud_destination_topic": "/merged_cloud"},
            {"pointcloud_topics": "/VLP16_lidar_back/velodyne_points /VLP16_lidar_front/velodyne_points"},
        ]
    )

    return LaunchDescription([
        laserscan_multi_merger
    ])
