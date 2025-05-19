# UAV Object Detection and Tracking Simulation

A ROS 2 Humble-based project for UAV simulation with object detection and tracking capabilities using PX4 SITL, Gazebo Garden, and ROS 2.

## Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Garden
- PX4 Autopilot (v1.14+)
- Python 3.10+
- Micro XRCE-DDS Agent

## Quick Start Checklist

Below is a clean step-by-step checklist that starts inside your `~/uav_ws` workspace and ends with the simulation running. Just copy–paste each block in a fresh terminal tab.

### 0. Prerequisites Check (Do Once)

```bash
# Ubuntu 22.04 + ROS 2 Humble already installed?
source /opt/ros/humble/setup.bash || \
  { echo "ROS 2 Humble not found – install before continuing"; exit 1; }

# PX4 source tree cloned?
test -d ~/PX4-Autopilot || \
  git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
```

### 1. Workspace Layout

```bash
cd ~/uav_ws           # your catkin/colcon workspace
mkdir -p src          # if you don't already have it
```

Required repositories in `src/`:
- `uav_simulation` - Simulation package with world files and launch scripts
- `px4_msgs` - PX4 message interface for ROS 2
- `px4_ros_com` - PX4-ROS 2 communication bridge
- `Micro-XRCE-DDS-Agent` - DDS bridge between PX4 and ROS 2
- `object_tracking` - Object tracking package
- `object_detection` - Object detection package

If any are missing:

```bash
cd ~/uav_ws/src

# PX4 message interface – branch must match PX4 firmware
git clone -b release/1.14 https://github.com/PX4/px4_msgs.git

# PX4-ROS 2 communication bridge
git clone -b release/1.14 https://github.com/PX4/px4_ros_com.git

# Micro XRCE-DDS Agent (tiny bridge PX4↔ROS 2)
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
```

### 2. Install Missing Build Dependencies

```bash
sudo apt update
rosdep update
cd ~/uav_ws
rosdep install --from-paths src --ignore-src -y
```

### 3. Build the Workspace

```bash
cd ~/uav_ws
colcon build --symlink-install
```

Add the setup line to your shell once:

```bash
echo "source ~/uav_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## How to Run the Simulation

Follow these steps in order, using a separate terminal for each step:

### Terminal 1: Start the Micro XRCE-DDS Agent

```bash
cd ~/uav_ws
source install/setup.bash
MicroXRCEAgent udp4 -p 8888
```

You should see output like:
```
[1689245687.802256] info     | UXR_Agent::Root::init             | with session pool size: 128
[1689245687.802304] info     | UXR_Agent::Root::init             | with heartbeat period: 200
[1689245687.802315] info     | UXR_Agent::Root::init             | with max clients: 100
[1689245687.802324] info     | UXR_Agent::Root::init             | with max sessions per client: 10
[1689245687.802333] info     | UXR_Agent::Root::init             | with transport: udp4
```

Leave this terminal running. When PX4 connects, you'll see:
```
[timestamp] info     | UXR_Agent::UDP4Agent::recv_message | client connected: 127.0.0.1:12345
```

### Terminal 2: Launch PX4 SITL and Gazebo

```bash
cd ~/uav_ws
source install/setup.bash
ros2 launch uav_simulation minimal_launch.py
```

This will start:
1. Gazebo with the UAV world
2. PX4 SITL with the x500_depth model
3. Camera bridges for RGB and depth cameras

You should see Gazebo open with the UAV world, and the PX4 shell will show startup messages.

### Terminal 3: Verify PX4 Topics in ROS 2

Once PX4 and the DDS bridge are running, check that the topics are being published:

```bash
cd ~/uav_ws
source install/setup.bash
ros2 topic list | grep /fmu
```

You should see multiple `/fmu/out/...` topics, indicating successful communication between PX4 and ROS 2:

```
/fmu/out/battery_status
/fmu/out/estimator_status_flags
/fmu/out/failsafe_flags
/fmu/out/manual_control_setpoint
/fmu/out/position_setpoint_triplet
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/vehicle_attitude
...
```

### Terminal 4: Check Camera Topics

Verify that the camera topics are being published:

```bash
cd ~/uav_ws
source install/setup.bash
ros2 topic list | grep camera
```

You should see:
```
/camera
/depth_camera
```

### Terminal 5: Visualize Camera Feed

To see the camera output:

```bash
cd ~/uav_ws
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

In the dropdown menu, select `/camera` or `/depth_camera` to view the live feed.

### Terminal 6: Arm the Vehicle (Optional)

To arm the vehicle in simulation:

```bash
cd ~/uav_ws
source install/setup.bash
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{timestamp: 0, param1: 1.0, param2: 0.0, command: 176, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"
```

You should see the motors spin up in the simulation and the PX4 shell will show "ARMED" status.

## Detailed Installation

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

### 4. Setup Micro XRCE-DDS Agent

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

## Project Structure

```
uav_ws/                           # Main workspace directory
├── src/                          # Source packages
│   ├── object_detection/         # Object detection package
│   │   ├── launch/               # Launch files for detection
│   │   │   └── detection_launch.py  # Launch file for detection nodes
│   │   ├── object_detection/     # Python module
│   │   ├── models/               # ML models for object detection
│   │   ├── package.xml           # Package manifest
│   │   └── setup.py              # Package setup file
│   │
│   ├── object_tracking/          # Object tracking package
│   │   ├── launch/               # Launch files for tracking
│   │   │   └── tracking_launch.py   # Launch file for tracking nodes
│   │   ├── object_tracking/      # Python module
│   │   ├── package.xml           # Package manifest
│   │   └── setup.py              # Package setup file
│   │
│   ├── px4_msgs/                 # PX4 message definitions
│   │   ├── msg/                  # Message definition files
│   │   ├── package.xml           # Package manifest
│   │   └── CMakeLists.txt        # CMake build file
│   │
│   ├── px4_ros_com/              # PX4-ROS2 communication bridge
│   │   ├── launch/               # Launch files for PX4-ROS2 bridge
│   │   ├── src/                  # C++ source files
│   │   ├── package.xml           # Package manifest
│   │   └── CMakeLists.txt        # CMake build file
│   │
│   ├── Micro-XRCE-DDS-Agent/     # DDS bridge for PX4-ROS2 communication
│   │   ├── src/                  # C++ source files
│   │   ├── include/              # Header files
│   │   └── CMakeLists.txt        # CMake build file
│   │
│   └── uav_simulation/           # UAV simulation package
│       ├── launch/               # Launch files
│       │   ├── main_launch.py        # Original launch file (requires uav_control)
│       │   ├── complete_launch.py    # Combines main_launch.py with tracking
│       │   └── minimal_launch.py     # Minimal working launch file
│       ├── models/               # Gazebo models
│       │   └── x500_depth/           # PX4 UAV model with depth camera
│       ├── uav_simulation/       # Python package
│       │   ├── __init__.py           # Package initialization
│       │   └── uav_control.py        # UAV control node
│       ├── worlds/               # Gazebo world files
│       │   └── uav_world.sdf         # Main simulation world
│       ├── package.xml           # Package manifest
│       └── setup.py              # Package setup file
│
├── build/                        # Build artifacts (generated)
├── install/                      # Installation files (generated)
├── log/                          # Log files (generated)
├── start_simulation.sh           # Convenience script to start all components
└── README.md                     # This documentation file
```

### Package Dependencies

The packages in this workspace have the following dependencies:

- **uav_simulation**: Depends on Gazebo, ROS 2, and px4_msgs
- **object_detection**: Depends on ROS 2, OpenCV, and potentially YOLOv8
- **object_tracking**: Depends on ROS 2, object_detection, and potentially DeepSORT
- **px4_msgs**: Standalone package that defines message interfaces
- **px4_ros_com**: Depends on px4_msgs and ROS 2
- **Micro-XRCE-DDS-Agent**: Standalone package for DDS communication

### Data Flow

The data flow between components is as follows:

1. **PX4 SITL** → **Micro-XRCE-DDS-Agent** → **ROS 2** (via `/fmu/...` topics)
2. **Gazebo** → **ROS 2** (via camera bridge, `/camera` and `/depth_camera` topics)
3. **ROS 2** → **object_detection** → **object_tracking** → **uav_control** (for autonomous flight)
4. **uav_control** → **PX4 SITL** (via `/fmu/in/...` topics)

### Launch Files

The launch files are organized to provide different levels of functionality:

- **minimal_launch.py**: Core simulation without perception or control
- **main_launch.py**: Adds UAV control capabilities
- **complete_launch.py**: Full system with perception and control
- **tracking_launch.py**: Object tracking components only
- **detection_launch.py**: Object detection components only

## Automated Startup

For convenience, you can use the provided `start_simulation.sh` script to launch all components:

```bash
cd ~/uav_ws
./start_simulation.sh
```

This script:
1. Starts the Micro XRCE-DDS Agent
2. Starts PX4 SITL
3. Launches Gazebo with the UAV world
4. Sets up the camera bridge
5. Starts the object tracking system

## Troubleshooting

### Common Issues and Solutions

| Symptom | Cause | Solution |
|---------|-------|----------|
| `colcon build` fails with symlink errors | Previous build artifacts causing conflicts | `rm -rf ~/uav_ws/build ~/uav_ws/install ~/uav_ws/log` and rebuild |
| **`MicroXRCEAgent` shows no client** | PX4 not built with DDS bridge | Ensure you're on PX4 firmware ≥ v1.14 |
| **Gazebo doesn't show the drone** | Model path issues | Check `GZ_SIM_RESOURCE_PATH` includes `~/.gz/models` |
| **Camera topics not appearing** | Bridge not running or camera not in model | Check `ros2 topic list \| grep camera` and camera configuration in model SDF |
| **`colcon build` fails on C++17** | Compiler version too old | `export CC=gcc-10 CXX=g++-10` and rebuild (rare on Ubuntu 22.04) |
| **Gazebo camera FPS < 15** | Camera resolution too high | Lower camera `<width>`/`<height>` in the SDF to 640×360 |
| **PX4 fails to connect to Agent** | Wrong port or protocol | Ensure Agent is running on UDP port 8888 |

### Checking ROS 2 Topics

To verify that everything is working correctly, check for the following topics:

```bash
# PX4 topics
ros2 topic list | grep /fmu

# Camera topics
ros2 topic list | grep camera

# Object detection/tracking topics (if implemented)
ros2 topic list | grep detection
```

### Visualizing Camera Feeds

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera` or `/depth_camera` from the dropdown menu.

## Next Steps

1. Implement or fix the `uav_control` node to enable autonomous flight control.
2. Complete the object detection and tracking integration.
3. Create a custom world with moving objects for tracking.
4. Implement a mission planner for autonomous navigation.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Gazebo Garden](https://gazebosim.org/)
- [Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS) 