from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='front_scanner', default_value='VLP16_lidar_front',
            description='Namespace for the lidar topics'
        ),
        DeclareLaunchArgument(
            name='back_scanner', default_value='VLP16_lidar_back',
            description='Namespace for the back lidar topics'
        ),
        # Node for the front LiDAR
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', [LaunchConfiguration(variable_name='front_scanner'), '/points']),
                ('scan', [LaunchConfiguration(variable_name='front_scanner'), '/scan'])
            ],
            parameters=[{
                'target_frame': 'VLP16_lidar_front',
                'transform_tolerance': 0.01,
                'min_height': -3.0,  # Expanding to cover a larger vertical range
                'max_height': 0.5,   # Adjust as necessary based on sensor position
                'angle_min': -3.1416,  # Full 360-degree scan
                'angle_max': 3.1416,
                'angle_increment': 0.007,  # Higher resolution
                'scan_time': 0.2,        # Adjusted for 10 Hz rate
                'range_min': 0.7,  # Reduce to capture closer points if necessary
                'range_max': 10.0, # Extend if you need more distant points
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_front'
        ),

        # Node for the back LiDAR
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', [LaunchConfiguration(variable_name='back_scanner'), '/points']),
                ('scan', [LaunchConfiguration(variable_name='back_scanner'), '/scan'])
            ],
            parameters=[{
                'target_frame': 'VLP16_lidar_back',
                'transform_tolerance': 0.01,
                'min_height': -3.0,      # Adjusted height range
                'max_height': 0.5,       # Adjusted height range
                'angle_min': -3.1416,  # Full 360-degree scan
                'angle_max': 3.1416,
                'angle_increment': 0.007,  # Higher resolution
                'scan_time': 0.2,        # Adjusted for 10 Hz rate
                'range_min': 0.7,        # Reduced range
                'range_max': 10.0,       # Extended range
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_back'
        )
    ])
