# Quickstart Guide: Digital Twin Module for Physical AI Textbook

## Overview
This quickstart guide provides the essential steps to set up and run the Digital Twin system with Gazebo physics simulation and Unity visualization for Module 2 of the AI-Native Textbook on Physical AI & Humanoid Robotics.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- 16GB+ RAM (32GB recommended for multi-robot simulation)
- NVIDIA GPU with CUDA support (for Unity visualization)
- 50GB+ free disk space

### Software Dependencies
```bash
# ROS 2 Humble or Iron
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-navigation2

# Python 3.10+
sudo apt install python3.10 python3.10-venv python3-pip

# Gazebo Fortress or Harmonic
sudo apt install gazebo11 libgazebo11-dev

# Git and build tools
sudo apt install git build-essential cmake
```

### Unity Setup
- Download and install Unity Hub LTS version
- Install Unity 2022.3.x LTS (Long Term Support)

## Installation Steps

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/ai-native-textbook.git
cd ai-native-textbook
```

### 2. Set up ROS 2 Workspace
```bash
# Create workspace
mkdir -p ~/textbook_ws/src
cd ~/textbook_ws/src

# Link the textbook packages
ln -s /path/to/ai-native-textbook/src ~/textbook_ws/src/textbook

# Install dependencies
cd ~/textbook_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-select textbook_gazebo textbook_unity_bridge textbook_sensors
source install/setup.bash
```

### 3. Install Python Dependencies
```bash
cd /path/to/ai-native-textbook
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Running the Digital Twin System

### 1. Basic Gazebo Simulation
```bash
# Source ROS 2
source ~/textbook_ws/install/setup.bash

# Launch basic humanoid simulation
ros2 launch textbook_gazebo basic_humanoid.launch.py
```

### 2. Sensor Simulation
```bash
# Launch with sensor simulation
ros2 launch textbook_gazebo sensor_simulation.launch.py

# Verify sensor data is streaming
ros2 topic list | grep /scan
ros2 topic list | grep /camera
ros2 topic list | grep /imu
```

### 3. Unity Visualization
```bash
# Start the Unity bridge
ros2 run textbook_unity_bridge unity_bridge_node

# In Unity Hub:
# 1. Open the project in /path/to/ai-native-textbook/unity/
# 2. Load the DigitalTwinScene
# 3. Run the scene to visualize the simulation
```

## Basic Examples

### Example 1: Physics Configuration
```bash
# Launch with custom physics parameters
ros2 launch textbook_gazebo physics_demo.launch.py \
  gravity_x:=0.0 gravity_y:=0.0 gravity_z:=-9.8 \
  solver_iterations:=200
```

### Example 2: Multi-Robot Simulation
```bash
# Launch 3 robots in the same environment
ros2 launch textbook_gazebo multi_robot.launch.py \
  robot_count:=3
```

### Example 3: Sensor Data Verification
```bash
# Monitor LiDAR data
ros2 topic echo /robot1/scan sensor_msgs/msg/LaserScan

# Monitor camera data
ros2 topic echo /robot1/camera/depth/image_raw sensor_msgs/msg/Image

# Monitor IMU data
ros2 topic echo /robot1/imu/data sensor_msgs/msg/Imu
```

## Common Commands

### Simulation Control
```bash
# Pause simulation
rosservice call /pause_physics std_srvs/srv/Empty

# Unpause simulation
rosservice call /unpause_physics std_srvs/srv/Empty

# Reset simulation
rosservice call /reset_simulation std_srvs/srv/Empty
```

### Model Spawning
```bash
# Spawn a model in simulation
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/model.urdf -x 0 -y 0 -z 1

# Delete a model
ros2 run gazebo_ros delete_entity.py -entity my_robot
```

## Verification Steps

### 1. Physics Validation
```bash
# Check if robot maintains stable contact with ground
ros2 topic echo /robot1/joint_states --field position

# Verify gravity is working (robot should fall if not supported)
```

### 2. Sensor Validation
```bash
# Check sensor data rates
ros2 topic hz /robot1/scan
ros2 topic hz /robot1/camera/depth/image_raw
ros2 topic hz /robot1/imu/data
```

### 3. ROS 2 Integration
```bash
# List all active topics
ros2 topic list

# Check TF tree
ros2 run tf2_tools view_frames
```

## Troubleshooting

### Performance Issues
- Reduce solver iterations for faster performance (may reduce accuracy)
- Use headless Gazebo mode: `gzserver` instead of `gazebo`
- Lower Unity visualization quality settings

### Connection Issues
- Ensure ROS 2 domains match between nodes
- Check network settings for Unity bridge
- Verify Gazebo and ROS 2 are properly sourced

### Sensor Data Problems
- Check sensor plugins are loaded in URDF
- Verify correct frame IDs in sensor configurations
- Confirm TF tree is properly populated

## Next Steps

1. Complete the hands-on labs in `/docs/module-2/labs/`
2. Explore the chapter-specific examples
3. Build your own custom digital twin configurations
4. Experiment with different physics parameters
5. Integrate with your own perception algorithms

## Getting Help

- Check the documentation in `/docs/module-2/`
- Review the troubleshooting section in the textbook
- Join the textbook community forum for additional support