# Gazebo Setup Tutorial

## Overview

This tutorial guides you through setting up Gazebo for robotics simulation with ROS 2 integration. We'll cover installation, configuration, and basic operation for the digital twin system.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill or Iron Irwini
- Basic Linux command line knowledge
- Minimum 8GB RAM (16GB+ recommended)

## Installation

### Step 1: Install ROS 2

First, ensure you have ROS 2 installed:

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-navigation2
```

### Step 2: Install Gazebo Fortress/Harmonic

```bash
# Add Gazebo repository
sudo apt install software-properties-common
sudo add-apt-repository universe
curl -sSL http://get.gazebosim.org | sh

# Install Gazebo
sudo apt install gz-fortress
# OR for Harmonic (if available)
sudo apt install gz-harmonic
```

### Step 3: Install Additional Dependencies

```bash
# Install build tools
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo apt install python3-vcstool

# Install graphics drivers (for visualization)
sudo apt install nvidia-driver-470  # Or latest appropriate driver for your GPU
sudo apt install mesa-utils
```

## Configuration

### Step 4: Set Up Environment

Add ROS 2 to your bash profile:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Create Workspace

Create a ROS 2 workspace for your robotics projects:

```bash
# Create workspace directory
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --packages-select gazebo_ros_pkgs
source install/setup.bash
```

## Basic Gazebo Operation

### Step 6: Launch Basic Simulation

Test your Gazebo installation:

```bash
# Launch Gazebo with default world
gz sim

# Or launch with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

### Step 7: Create Your First World

Create a simple world file `~/robotics_ws/src/my_robot/worlds/simple_room.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_room">
    <!-- Physics configuration -->
    <physics type="bullet">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Room walls -->
    <model name="wall_1">
      <pose>5 0 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_2">
      <pose>-5 0 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_3">
      <pose>0 5 1.5 0 0 1.57</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_4">
      <pose>0 -5 1.5 0 0 1.57</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 10 3</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Simple robot model -->
    <model name="simple_robot">
      <pose>0 0 0.2 0 0 0</pose>
      <link name="chassis">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166</iyy>
            <iyz>0</iyz>
            <izz>0.166</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.2</size></box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.3 1.0 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Step 8: Launch Your Custom World

```bash
# Launch with your custom world
gz sim ~/robotics_ws/src/my_robot/worlds/simple_room.sdf

# Or with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py world:=~/robotics_ws/src/my_robot/worlds/simple_room.sdf
```

## Robot Integration

### Step 9: Create a Robot Model

Create a basic robot model in `~/robotics_ws/src/my_robot/models/my_robot/model.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="my_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.416</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.416</iyy>
          <iyz>0</iyz>
          <izz>0.833</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box><size>0.5 0.5 0.2</size></box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box><size>0.5 0.5 0.2</size></box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>1.0 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Wheels -->
    <link name="wheel_front_left">
      <pose>0.2 0.3 0 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.02</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>

    <link name="wheel_front_right">
      <pose>0.2 -0.3 0 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.02</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Differential drive joint -->
    <joint name="left_wheel_hinge" type="revolute">
      <parent>chassis</parent>
      <child>wheel_front_left</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><effort>10000</effort><velocity>1000</velocity></limit>
      </axis>
      <pose>0.2 0.3 0 0 0 0</pose>
    </joint>

    <joint name="right_wheel_hinge" type="revolute">
      <parent>chassis</parent>
      <child>wheel_front_right</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><effort>10000</effort><velocity>1000</velocity></limit>
      </axis>
      <pose>0.2 -0.3 0 0 0 0</pose>
    </joint>
  </model>
</sdf>
```

### Step 10: Add to Your World

Update your world file to include the robot:

```xml
<!-- Add this inside your world tag -->
<include>
  <uri>model://my_robot</uri>
  <name>robot1</name>
  <pose>0 0 0.2 0 0 0</pose>
</include>
```

## Sensor Integration

### Step 11: Add Sensors to Your Robot

Update your robot model to include sensors:

```xml
<!-- Add this inside the model tag of your robot -->
<!-- LiDAR sensor -->
<sensor name="lidar_2d" type="ray">
  <pose>0.2 0 0.1 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>10</update_rate>

  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>

  <plugin name="lidar_controller" filename="gz-sim-ray-sensor-system">
    <topic>/robot1/scan</topic>
  </plugin>
</sensor>

<!-- Camera sensor -->
<sensor name="camera" type="camera">
  <pose>0.2 0 0.15 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>

  <plugin name="camera_controller" filename="gz-sim-camera-system">
    <topic>/robot1/camera/image</topic>
  </plugin>
</sensor>
```

## Troubleshooting

### Common Issues and Solutions

#### Issue 1: Gazebo Won't Launch
**Symptom**: Gazebo fails to start or crashes immediately
**Solution**:
```bash
# Check graphics drivers
glxinfo | grep "OpenGL renderer"
# Update drivers if needed
sudo apt update && sudo apt upgrade
# Check Gazebo installation
which gz
```

#### Issue 2: ROS 2 Integration Not Working
**Symptom**: No communication between Gazebo and ROS 2
**Solution**:
```bash
# Check if Gazebo ROS packages are installed
dpkg -l | grep gazebo-ros
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
# Check available topics
ros2 topic list
```

#### Issue 3: Poor Performance
**Symptom**: Low frame rates or simulation lag
**Solution**:
```bash
# Check system resources
htop
# Reduce physics complexity
# Use headless mode if visualization not needed
gz sim -s
```

## Performance Optimization

### Step 12: Optimize Physics Settings

For better performance, adjust physics settings in your world file:

```xml
<physics type="bullet">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.01</max_step_size>  <!-- Increased for performance -->
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>100</real_time_update_rate>  <!-- Reduced for performance -->
  <solver>
    <type>quick</type>
    <iters>50</iters>  <!-- Reduced from default for performance -->
    <sor>1.3</sor>
  </solver>
</physics>
```

## Testing Your Setup

### Step 13: Verify Installation

Test your complete setup:

```bash
# Launch Gazebo with your world
gz sim ~/robotics_ws/src/my_robot/worlds/simple_room.sdf

# In another terminal, check if robot is publishing sensor data
source /opt/ros/humble/setup.bash
ros2 topic list | grep robot1

# Check LiDAR data
ros2 topic echo /robot1/scan --field ranges | head -n 10

# Check camera data
ros2 run image_view image_view _image:=/robot1/camera/image
```

## Next Steps

Congratulations! You've successfully set up Gazebo for robotics simulation. Now you can:

1. Continue with the Unity visualization tutorial
2. Learn about sensor simulation in detail
3. Implement more complex robot models
4. Set up ROS 2 navigation stack
5. Create custom worlds and environments

## Resources

- Gazebo Documentation: http://gazebosim.org/
- ROS 2 with Gazebo: https://gazebosim.org/docs/harmonic/ros_integration/
- Tutorials: http://gazebosim.org/tutorials

## Summary

This tutorial covered the complete setup of Gazebo for robotics simulation, including:
- Installation of Gazebo and ROS 2 integration
- Configuration of environment and workspace
- Creation of basic world and robot models
- Addition of sensors for perception
- Performance optimization techniques
- Troubleshooting common issues

You now have a working Gazebo environment ready for more advanced robotics simulation tasks.