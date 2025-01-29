import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Resolve the package's share directory
    package_share_dir = get_package_share_directory("concert_mapping")
    # Full path to the PCD file
    pcd_file_path = os.path.join(package_share_dir,
                                "pointclouds",
                                "pointclouds_")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pcd_file",
            default_value=pcd_file_path  # Use the constructed path
        ),
        Node(
            package='perception_utils_ros2',  # Replace with the actual package name containing pointcloud_to_pcd
            executable='combined_pointcloud_to_pcd_node',  # Name of the executable
            name='pointcloud_to_pcd',
            parameters=[{
                'prefix': LaunchConfiguration("pcd_file"),        # Set the PCD file name prefix
                'binary': False,           # Save the PCD file in ASCII format
                'compressed': False,       # Disable compression
                'rgb': False,              # Set RGB support to false if the point cloud doesn't contain color
                'save_timer_sec': 0.0,
                'save_on_shutdown': True
            }],
            remappings=[
                ('input', '/cloud_map')       # Remap the input point cloud topic to '/cloud_in'
            ],
            output='screen'
        )
    ])
