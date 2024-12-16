from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments for the PCD file and topic
    return LaunchDescription([
        DeclareLaunchArgument(
            "base_link_frame",
            default_value="base_link",
            description="base_link frame needed"
        ),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="cloud_pcd",
            description="Topic to publish the PointCloud2 messages"
        ),
        DeclareLaunchArgument(
            "publishing_period_ms",
            default_value="3000",
            description="Publishing period in milliseconds"
        ),

        DeclareLaunchArgument(
            "pcd_file",
            default_value=os.path.join(
                get_package_share_directory("concert_mapping"),
                "pointclouds",
                "pointclouds_combined.pcd"
            )
        ),
        
        # Node configuration
        Node(
            package="pcl_ros",  # Replace with your package name
            executable="pcd_to_pointcloud",  # Replace with your node executable name
            name="pcd_publisher",
            parameters=[
                {
                    "file_name": LaunchConfiguration("pcd_file"),
                    "tf_frame": LaunchConfiguration("base_link_frame"),
                    "cloud_topic": LaunchConfiguration("cloud_topic"),
                    "publishing_period_ms": LaunchConfiguration("publishing_period_ms")
                }
            ]
        )
    ])
