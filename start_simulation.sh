#!/bin/bash

# Function to start a new terminal and run a command
function start_term() {
    gnome-terminal --tab -- bash -c "$1; exec bash"
}

echo "Starting UAV Object Detection and Tracking Simulation..."

# Start MicroXRCEAgent
start_term "MicroXRCEAgent udp4 -p 8888"
sleep 2

# Start PX4 SITL
start_term "export GZ_SIM_RESOURCE_PATH=~/.gz/models:/usr/share/gz/gz-garden && cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4"
sleep 5

# Start Gazebo
start_term "source /opt/ros/humble/setup.bash && source ~/uav_ws/install/setup.bash && gz sim -r ~/uav_ws/install/uav_simulation/share/uav_simulation/worlds/uav_world.sdf"
sleep 5

# Start Camera Bridge
start_term "source /opt/ros/humble/setup.bash && source ~/uav_ws/install/setup.bash && ros2 run ros_gz_image image_bridge /camera"
sleep 2

# Start Object Tracking
start_term "source /opt/ros/humble/setup.bash && source ~/uav_ws/install/setup.bash && ros2 launch object_tracking tracking_launch.py"

echo "All components started. Check the individual terminals for any errors."
echo "To visualize the camera feed, select '/camera' or '/object_tracking/detection_image' in rqt_image_view." 