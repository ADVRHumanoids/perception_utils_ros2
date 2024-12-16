from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    laserscan_multi_merger = Node(
        package="perception_utils_ros2",
        executable="pointcloud_merger",
        name="pointcloud_merger",
        output="screen",
        parameters=[
            {"destination_frame": "VLP16_lidar_back"},
            {"cloud_destination_topic": "/merged_cloud"},
            {"pointcloud_topics": "/VLP16_lidar_back/points /VLP16_lidar_front/points"},
        ]
    )

    return LaunchDescription([
        laserscan_multi_merger
    ])