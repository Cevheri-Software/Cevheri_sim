import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('uav_simulation')
    
    # Path to the world file
    world_file = os.path.join(pkg_dir, 'worlds', 'uav_world.sdf')
    
    # Launch Gazebo
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Launch PX4 SITL - fixed command for Gazebo Garden
    px4_sitl = ExecuteProcess(
        cmd=['cd', os.path.expanduser('~/PX4-Autopilot'), '&&', 
             'PX4_SYS_AUTOSTART=4002', 'PX4_GZ_MODEL=x500_depth', './build/px4_sitl_default/bin/px4'],
        shell=True,
        output='screen'
    )
    
    # Launch Micro XRCE-DDS Agent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )
    
    # Launch ROS2-Gazebo bridge for camera images
    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera'],
        output='screen'
    )
    
    depth_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/depth_camera'],
        output='screen'
    )
    
    # Launch our UAV control node
    uav_control = Node(
        package='uav_simulation',
        executable='uav_control',
        name='uav_control',
        output='screen'
    )
    
    return LaunchDescription([
        gz_server,
        px4_sitl,
        micro_xrce_agent,
        camera_bridge,
        depth_camera_bridge,
        uav_control
    ])
