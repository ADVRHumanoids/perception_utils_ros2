"""
@file pointcloud_to_pcd.launch.py
@author Valerio Passamano
@brief Launches the point cloud accumulation node that exports a merged PCD file.

This launch file starts `pointcloud_to_pcd_node`, provides its PCD
output prefix and save options, and remaps the node's `input` subscription to
the `/cloud_map` topic so an accumulated cloud can be written on shutdown.
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "pcd_file",
            default_value="maps/pointclouds_"
        ),
        Node(
            package='perception_utils_ros2',  # Replace with the actual package name containing pointcloud_to_pcd
            executable='pointcloud_to_pcd_node',  # Name of the executable
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
