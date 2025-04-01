# UAV Object Detection and Tracking

A ROS2 Humble-based project for UAV simulation with object detection and tracking capabilities using YOLOv8 and DeepSORT.

## Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Garden
- PX4 Autopilot
- Python 3.10+

## Installation

### 1. Install ROS 2 Humble

Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).

### 2. Install Gazebo Garden

```bash
sudo apt-get update && sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
sudo apt install ros-humble-ros-gz-image ros-humble-ros-gz-bridge
```

### 3. Install PX4 Autopilot

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl_default
```

### 4. Setup Micro XRCE-DDS Agent & Client

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### 5. Install Python Dependencies

```bash
pip install opencv-python numpy ultralytics
```

### 6. Setup Environment

Create Gazebo model directory and update environment variables:

```bash
mkdir -p ~/.gz/models
echo 'export GZ_SIM_RESOURCE_PATH=~/.gz/models:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

### 7. Modify PX4 camera angle (for better visuals)

Update the camera pose for the x500_depth model:

```bash
sudo sed -i 's/<pose>.12 .03 .242 0 0 0<\/pose>/<pose>.15 .029 .21 0 0.7854 0<\/pose>/g' ~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf
```

### 8. Build this Repository

```bash
cd ~/uav_ws
colcon build
source install/setup.bash
```

## Project Structure

```
uav_ws/
├── src/
│   ├── object_detection/       # YOLOv8 object detection package
│   ├── object_tracking/        # Object tracking package (DeepSORT)
│   ├── px4_msgs/               # PX4 message definitions
│   ├── px4_ros_com/            # PX4-ROS2 communication bridge
│   └── uav_simulation/         # UAV simulation package
│       ├── launch/             # Launch files
│       ├── models/             # Gazebo models
│       ├── uav_simulation/     # Python package
│       └── worlds/             # Gazebo world files
```

## Running the Simulation

You need to open multiple terminal windows for different components:

### Terminal 1: Start the Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 2: Start PX4 SITL

```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
```

### Terminal 3: Launch Gazebo and ROS Bridge

```bash
source ~/uav_ws/install/setup.bash
ros2 run ros_gz_image image_bridge /camera
```

### Terminal 4: Launch Object Tracking

```bash
source ~/uav_ws/install/setup.bash
ros2 launch object_tracking tracking_launch.py
```

### Terminal 5: Visualize Camera Feed

```bash
source ~/uav_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

## All-in-One Launch (Experimental)

For convenience, you can try the combined launch file (note: some components might need to be started separately):

```bash
source ~/uav_ws/install/setup.bash
ros2 launch uav_simulation complete_launch.py
```

## YOLOv8 Integration

The YOLOv8 integration is currently commented out in the `object_tracking/tracking_node.py` file. To enable it:

1. Ensure the ultralytics package is installed: `pip install ultralytics`
2. Uncomment the YOLOv8 import line at the beginning of the file
3. Uncomment the model initialization in the `__init__` method
4. Replace the `detect_objects_by_color` call with `detect_objects_with_yolo` in the `process_images` method
5. Uncomment the contents of the `detect_objects_with_yolo` method

## Troubleshooting

- If Gazebo fails to start, try `killall gzserver` and restart
- Ensure PX4 is properly installed and built
- Check that the MicroXRCEAgent is running (required for PX4-ROS2 communication)
- If the model doesn't appear, check the environment variables and paths
- The SDF file for the world and models should have correct file permissions

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8)
- [drone-adventure](https://github.com/amirkhosrovosughi/drone-adventure) 