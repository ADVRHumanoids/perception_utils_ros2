from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    laserscan_multi_merger = Node(
        package="ira_laser_tools",
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