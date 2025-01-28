import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for 'perception_utils_ros2'
    perception_share_dir = get_package_share_directory('perception_utils_ros2')


    # Include the pointcloud_to_laserscan launch file
    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(perception_share_dir, 'launch','pointcloud_to_laserscan.launch.py'))
    )
    # Include the mux launch file
    mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(perception_share_dir, 'launch','laserscan_multi_merger.launch.py'))
    )

    return LaunchDescription([
        pointcloud_to_laserscan_launch,
        mux_launch
    ])
