import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    uav_pkg_dir = get_package_share_directory('uav_simulation')
    tracking_pkg_dir = get_package_share_directory('object_tracking')
    
    # Include the main UAV simulation launch file
    uav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uav_pkg_dir, 'launch', 'main_launch.py')
        )
    )
    
    # Include the object tracking launch file
    tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tracking_pkg_dir, 'launch', 'tracking_launch.py')
        )
    )
    
    # Create and return the launch description
    return LaunchDescription([
        uav_launch,
        tracking_launch
    ]) 