import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    tracking_pkg_dir = get_package_share_directory('object_tracking')
    
    # Launch the object tracking node
    tracking_node = Node(
        package='object_tracking',
        executable='tracking_node',
        name='object_tracking_node',
        output='screen'
    )
    
    # Create and return the launch description
    return LaunchDescription([
        tracking_node
    ])
