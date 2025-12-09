# ROS 2 Integration Tutorial

## Overview

This tutorial covers the integration of ROS 2 with Gazebo and Unity for creating a complete digital twin system. We'll cover ROS 2 setup, communication protocols, sensor data streaming, and system integration for robotics applications.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Gazebo Fortress/Harmonic installed
- Basic knowledge of ROS 2 concepts (topics, services, nodes)
- Understanding of robotics sensor data formats

## Installation and Setup

### Step 1: Install ROS 2 Humble

If not already installed:

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-rosbridge-suite
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo apt install python3-vcstool
```

### Step 2: Set Up ROS 2 Environment

Add ROS 2 to your bash profile:

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

### Step 3: Create ROS 2 Workspace

Create a workspace for your robotics packages:

```bash
# Create workspace
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Source the workspace
source /opt/ros/humble/setup.bash
```

## ROS 2 with Gazebo Integration

### Step 4: Verify Gazebo-ROS 2 Connection

Test the Gazebo-ROS 2 integration:

```bash
# Launch Gazebo with ROS 2 bridge
ros2 launch gazebo_ros gazebo.launch.py

# In another terminal, check available topics
source /opt/ros/humble/setup.bash
ros2 topic list

# You should see topics like:
# /clock
# /gazebo/link_states
# /gazebo/model_states
```

### Step 5: Create a Robot Model Package

Create a package for your robot model:

```bash
cd ~/robotics_ws/src
ros2 pkg create --build-type ament_python my_robot_description
cd my_robot_description

# Create directory structure
mkdir -p urdf meshes launch worlds
```

Create the robot URDF file `urdf/my_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.416" ixy="0.0" ixz="0.0" iyy="0.416" iyz="0.0" izz="0.833"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <link name="wheel_front_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <link name="wheel_front_right">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Wheel joints -->
  <joint name="wheel_front_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_front_left"/>
    <origin xyz="0.2 0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_front_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_front_right"/>
    <origin xyz="0.2 -0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Sensors -->
  <!-- LiDAR -->
  <gazebo reference="base_link">
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
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>robot1</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>robot1/lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Camera -->
  <gazebo reference="base_link">
    <sensor name="camera" type="camera">
      <pose>0.2 0 0.15 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>30</update_rate>
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
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>robot1</namespace>
          <remapping>~/image_raw:=camera/image_raw</remapping>
          <remapping>~/camera_info:=camera/camera_info</remapping>
        </ros>
        <frame_name>robot1/camera_optical_frame</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU -->
  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>robot1</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <frame_name>robot1/imu_link</frame_name>
        <body_name>base_link</body_name>
        <update_rate>100</update_rate>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### Step 6: Create Launch File

Create a launch file `launch/robot_world.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Choose one of the world files from `/usr/share/gazebo/worlds`')

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'worlds',
                'small_room.world'
            ]),
            'verbose': 'false'
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'urdf',
                'my_robot.urdf'
            ])
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Add nodes
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    return ld
```

### Step 7: Create World File

Create a simple world file `worlds/small_room.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="small_room">
    <physics type="bullet">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Room walls -->
    <model name="wall_1">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.2 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 3</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_2">
      <pose>0 5 1.5 0 0 1.57</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.2 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 3</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_3">
      <pose>0 -5 1.5 0 0 -1.57</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.2 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 3</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_4">
      <pose>5 0 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 10 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 10 3</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_5">
      <pose>-5 0 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 10 3</size></box></geometry>
        </location>
        <visual name="visual">
          <geometry><box><size>0.2 10 3</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Sensor Data Streaming

### Step 8: Create Sensor Data Publisher

Create a ROS 2 node to publish sensor data: `my_robot_description/my_robot_description/sensor_publisher.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import numpy as np
from cv_bridge import CvBridge
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publishers for sensor data
        self.lidar_publisher = self.create_publisher(
            LaserScan,
            '/robot1/scan',
            10
        )

        self.camera_publisher = self.create_publisher(
            Image,
            '/robot1/camera/image_raw',
            10
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            '/robot1/imu/data',
            10
        )

        self.joint_publisher = self.create_publisher(
            JointState,
            '/robot1/joint_states',
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/robot1/odom',
            10
        )

        # Create subscriber for velocity commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/robot1/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Initialize variables
        self.current_time = self.get_clock().now()
        self.previous_time = self.get_clock().now()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0

        # Timer for publishing sensor data
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # 10 Hz

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        self.get_logger().info('Sensor Publisher node initialized')

    def cmd_vel_callback(self, msg):
        """Callback for velocity commands"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def publish_sensor_data(self):
        """Publish all sensor data"""
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.previous_time.nanoseconds) / 1e9
        self.previous_time = current_time

        # Update robot position based on velocity
        self.x_pos += self.linear_velocity * math.cos(self.theta) * dt
        self.y_pos += self.linear_velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        # Publish LiDAR data
        self.publish_lidar_data(current_time)

        # Publish camera data
        self.publish_camera_data(current_time)

        # Publish IMU data
        self.publish_imu_data(current_time)

        # Publish joint states
        self.publish_joint_states(current_time)

        # Publish odometry
        self.publish_odom_data(current_time)

    def publish_lidar_data(self, timestamp):
        """Publish LiDAR scan data"""
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = timestamp.to_msg()
        scan_msg.header.frame_id = 'robot1/lidar_link'

        # Set LiDAR parameters
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = 2 * math.pi / 360  # 360 points
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 30.0

        # Generate simulated ranges (with some obstacles)
        num_ranges = 360
        ranges = []

        for i in range(num_ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            # Add some simulated obstacles
            distance = 30.0  # Default max range

            # Simulate a wall at x=3m
            if abs(math.cos(angle) * 3 - self.x_pos) < 0.5 and abs(math.sin(angle) * 3 - self.y_pos) < 0.5:
                distance = 3.0 + random.uniform(-0.1, 0.1)

            # Add some noise
            distance += random.uniform(-0.05, 0.05)
            ranges.append(max(0.1, min(30.0, distance)))

        scan_msg.ranges = ranges
        scan_msg.intensities = [100.0] * num_ranges

        self.lidar_publisher.publish(scan_msg)

    def publish_camera_data(self, timestamp):
        """Publish camera image data"""
        # Create a simple test image
        height, width = 480, 640
        image_array = np.zeros((height, width, 3), dtype=np.uint8)

        # Add some colored rectangles to simulate features
        image_array[100:200, 200:400] = [255, 0, 0]  # Red rectangle
        image_array[250:350, 100:300] = [0, 255, 0]  # Green rectangle
        image_array[300:400, 350:550] = [0, 0, 255]  # Blue rectangle

        # Add some noise
        noise = np.random.randint(0, 20, image_array.shape, dtype=np.uint8)
        image_array = np.clip(image_array.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(image_array, encoding='rgb8')
        image_msg.header.stamp = timestamp.to_msg()
        image_msg.header.frame_id = 'robot1/camera_optical_frame'

        self.camera_publisher.publish(image_msg)

    def publish_imu_data(self, timestamp):
        """Publish IMU data"""
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp.to_msg()
        imu_msg.header.frame_id = 'robot1/imu_link'

        # Set orientation (simplified - assuming robot is upright)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = math.sin(self.theta / 2)
        imu_msg.orientation.w = math.cos(self.theta / 2)

        # Set angular velocity (from commanded angular velocity with noise)
        imu_msg.angular_velocity.x = random.uniform(-0.01, 0.01)  # Small noise
        imu_msg.angular_velocity.y = random.uniform(-0.01, 0.01)
        imu_msg.angular_velocity.z = self.angular_velocity + random.uniform(-0.05, 0.05)

        # Set linear acceleration (gravity + motion)
        imu_msg.linear_acceleration.x = self.linear_velocity * math.cos(self.theta) + random.uniform(-0.1, 0.1)
        imu_msg.linear_acceleration.y = self.linear_velocity * math.sin(self.theta) + random.uniform(-0.1, 0.1)
        imu_msg.linear_acceleration.z = 9.81 + random.uniform(-0.1, 0.1)  # Gravity

        # Set covariance (simplified)
        imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        self.imu_publisher.publish(imu_msg)

    def publish_joint_states(self, timestamp):
        """Publish joint state data"""
        joint_msg = JointState()
        joint_msg.header.stamp = timestamp.to_msg()
        joint_msg.header.frame_id = 'robot1/base_link'

        joint_msg.name = ['wheel_front_left_joint', 'wheel_front_right_joint']

        # Calculate wheel positions based on robot motion
        left_wheel_pos = self.theta * 10  # Simplified relationship
        right_wheel_pos = self.theta * 10

        joint_msg.position = [left_wheel_pos, right_wheel_pos]
        joint_msg.velocity = [self.angular_velocity * 10, self.angular_velocity * 10]
        joint_msg.effort = [0.0, 0.0]

        self.joint_publisher.publish(joint_msg)

    def publish_odom_data(self, timestamp):
        """Publish odometry data"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'robot1/base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.y = self.y_pos
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        # Set velocities
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity

        # Set covariance (simplified)
        odom_msg.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                   0, 0.1, 0, 0, 0, 0,
                                   0, 0, 1000000, 0, 0, 0,
                                   0, 0, 0, 1000000, 0, 0,
                                   0, 0, 0, 0, 1000000, 0,
                                   0, 0, 0, 0, 0, 0.2]
        odom_msg.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                    0, 0.1, 0, 0, 0, 0,
                                    0, 0, 1000000, 0, 0, 0,
                                    0, 0, 0, 1000000, 0, 0,
                                    0, 0, 0, 0, 1000000, 0,
                                    0, 0, 0, 0, 0, 0.2]

        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 9: Update setup.py

Update the package setup file `setup.py`:

```python
from setuptools import setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/my_robot.urdf']),
        ('share/' + package_name + '/launch', ['launch/robot_world.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/small_room.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot description package for robotics simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = my_robot_description.sensor_publisher:main',
        ],
    },
)
```

### Step 10: Build and Test

Build your package:

```bash
cd ~/robotics_ws
colcon build --packages-select my_robot_description
source install/setup.bash

# Test the sensor publisher
ros2 run my_robot_description sensor_publisher

# In another terminal, listen to the topics
ros2 topic echo /robot1/scan
ros2 topic echo /robot1/camera/image_raw
ros2 topic echo /robot1/imu/data
```

## ROS Bridge for Unity Integration

### Step 11: Set Up ROS Bridge

Install and configure ROS Bridge for Unity communication:

```bash
# Install rosbridge suite if not already installed
sudo apt install ros-humble-rosbridge-suite

# Test the websocket server
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Step 12: Create Bridge Configuration

Create a bridge configuration file `launch/rosbridge.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    rosbridge_port = LaunchConfiguration('port', default='9090')
    rosbridge_address = LaunchConfiguration('address', default='127.0.0.1')

    # Declare launch arguments
    declare_port_cmd = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for rosbridge server')

    declare_address_cmd = DeclareLaunchArgument(
        'address',
        default_value='127.0.0.1',
        description='Address for rosbridge server')

    # Launch rosbridge server
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[
            {'port': rosbridge_port},
            {'address': rosbridge_address},
            {'authenticate': False},
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_port_cmd)
    ld.add_action(declare_address_cmd)

    # Add nodes
    ld.add_action(rosbridge_server)

    return ld
```

### Step 13: Create Unity Bridge Node

Create a specialized bridge node for Unity integration: `my_robot_description/my_robot_description/unity_bridge.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
import json
import asyncio
import websockets
from cv_bridge import CvBridge
import numpy as np

class UnityBridge(Node):
    def __init__(self):
        super().__init__('unity_bridge')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Store latest sensor data
        self.latest_lidar = None
        self.latest_camera = None
        self.latest_imu = None
        self.latest_odom = None

        # Create subscribers for sensor data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.lidar_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/robot1/camera/image_raw',
            self.camera_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/robot1/imu/data',
            self.imu_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10
        )

        # Create publishers for Unity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot1/cmd_vel',
            10
        )

        # Initialize WebSocket server
        self.websocket_server = None
        self.connected_clients = set()

        # Start WebSocket server in a separate thread
        self.start_websocket_server()

        self.get_logger().info('Unity Bridge initialized')

    def lidar_callback(self, msg):
        """Store latest LiDAR data"""
        self.latest_lidar = msg

    def camera_callback(self, msg):
        """Store latest camera data"""
        self.latest_camera = msg

    def imu_callback(self, msg):
        """Store latest IMU data"""
        self.latest_imu = msg

    def odom_callback(self, msg):
        """Store latest odometry data"""
        self.latest_odom = msg

    async def register_client(self, websocket):
        """Register a new client connection"""
        self.connected_clients.add(websocket)
        self.get_logger().info(f'New Unity client connected. Total clients: {len(self.connected_clients)}')

        try:
            # Send initial state
            await self.send_initial_state(websocket)

            # Handle messages from Unity
            async for message in websocket:
                await self.handle_unity_message(message, websocket)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.connected_clients.remove(websocket)
            self.get_logger().info(f'Unity client disconnected. Total clients: {len(self.connected_clients)}')

    async def send_initial_state(self, websocket):
        """Send initial robot state to Unity"""
        state_data = {
            'type': 'initial_state',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'has_lidar': self.latest_lidar is not None,
            'has_camera': self.latest_camera is not None,
            'has_imu': self.latest_imu is not None,
            'has_odom': self.latest_odom is not None
        }

        await websocket.send(json.dumps(state_data))

    async def handle_unity_message(self, message, websocket):
        """Handle messages received from Unity"""
        try:
            data = json.loads(message)
            msg_type = data.get('type', '')

            if msg_type == 'request_state':
                await self.send_current_state(websocket)
            elif msg_type == 'cmd_vel':
                self.handle_velocity_command(data)
            elif msg_type == 'request_sensor':
                await self.send_sensor_data(websocket, data.get('sensor_type'))
            else:
                self.get_logger().warn(f'Unknown message type: {msg_type}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON from Unity: {message}')
        except Exception as e:
            self.get_logger().error(f'Error handling Unity message: {e}')

    async def send_current_state(self, websocket):
        """Send current robot state to Unity"""
        state_data = {
            'type': 'robot_state',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        }

        if self.latest_odom:
            state_data['pose'] = {
                'position': {
                    'x': self.latest_odom.pose.pose.position.x,
                    'y': self.latest_odom.pose.pose.position.y,
                    'z': self.latest_odom.pose.pose.position.z
                },
                'orientation': {
                    'x': self.latest_odom.pose.pose.orientation.x,
                    'y': self.latest_odom.pose.pose.orientation.y,
                    'z': self.latest_odom.pose.pose.orientation.z,
                    'w': self.latest_odom.pose.pose.orientation.w
                }
            }
            state_data['twist'] = {
                'linear': {
                    'x': self.latest_odom.twist.twist.linear.x,
                    'y': self.latest_odom.twist.twist.linear.y,
                    'z': self.latest_odom.twist.twist.linear.z
                },
                'angular': {
                    'x': self.latest_odom.twist.twist.angular.x,
                    'y': self.latest_odom.twist.twist.angular.y,
                    'z': self.latest_odom.twist.twist.angular.z
                }
            }

        await websocket.send(json.dumps(state_data))

    async def send_sensor_data(self, websocket, sensor_type):
        """Send sensor data to Unity"""
        if sensor_type == 'lidar' and self.latest_lidar:
            lidar_data = {
                'type': 'lidar',
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'ranges': [float(r) if not np.isnan(r) and r > 0 else float('inf') for r in self.latest_lidar.ranges],
                'angle_min': self.latest_lidar.angle_min,
                'angle_max': self.latest_lidar.angle_max,
                'angle_increment': self.latest_lidar.angle_increment,
                'range_min': self.latest_lidar.range_min,
                'range_max': self.latest_lidar.range_max
            }
            await websocket.send(json.dumps(lidar_data))
        elif sensor_type == 'camera' and self.latest_camera:
            # Convert image to compressed format for transmission
            image_data = {
                'type': 'camera',
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'width': self.latest_camera.width,
                'height': self.latest_camera.height,
                'encoding': self.latest_camera.encoding,
                'step': self.latest_camera.step,
                # For simplicity, we're not sending the actual image data here
                # In practice, you'd want to compress and send image data efficiently
                'has_image': True
            }
            await websocket.send(json.dumps(image_data))
        elif sensor_type == 'imu' and self.latest_imu:
            imu_data = {
                'type': 'imu',
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'orientation': {
                    'x': self.latest_imu.orientation.x,
                    'y': self.latest_imu.orientation.y,
                    'z': self.latest_imu.orientation.z,
                    'w': self.latest_imu.orientation.w
                },
                'angular_velocity': {
                    'x': self.latest_imu.angular_velocity.x,
                    'y': self.latest_imu.angular_velocity.y,
                    'z': self.latest_imu.angular_velocity.z
                },
                'linear_acceleration': {
                    'x': self.latest_imu.linear_acceleration.x,
                    'y': self.latest_imu.linear_acceleration.y,
                    'z': self.latest_imu.linear_acceleration.z
                }
            }
            await websocket.send(json.dumps(imu_data))

    def handle_velocity_command(self, data):
        """Handle velocity commands from Unity"""
        cmd_vel = Twist()
        cmd_vel.linear.x = data.get('linear_x', 0.0)
        cmd_vel.linear.y = data.get('linear_y', 0.0)
        cmd_vel.linear.z = data.get('linear_z', 0.0)
        cmd_vel.angular.x = data.get('angular_x', 0.0)
        cmd_vel.angular.y = data.get('angular_y', 0.0)
        cmd_vel.angular.z = data.get('angular_z', 0.0)

        self.cmd_vel_publisher.publish(cmd_vel)

    def start_websocket_server(self):
        """Start the WebSocket server in a separate thread"""
        import threading
        thread = threading.Thread(target=self.run_websocket_server)
        thread.daemon = True
        thread.start()

    def run_websocket_server(self):
        """Run the WebSocket server"""
        asyncio.run(self.websocket_server_task())

    async def websocket_server_task(self):
        """WebSocket server task"""
        self.websocket_server = await websockets.serve(
            self.register_client,
            "localhost",
            8765
        )
        self.get_logger().info('Unity WebSocket server started on ws://localhost:8765')
        await self.websocket_server.wait_closed()

def main(args=None):
    rclpy.init(args=args)
    unity_bridge = UnityBridge()

    try:
        # Run the ROS node
        rclpy.spin(unity_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        unity_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing the Integration

### Step 14: Test ROS 2 Integration

Test your complete ROS 2 setup:

```bash
# Terminal 1: Start ROS Bridge
cd ~/robotics_ws
source install/setup.bash
ros2 launch my_robot_description rosbridge.launch.py

# Terminal 2: Start sensor publisher
cd ~/robotics_ws
source install/setup.bash
ros2 run my_robot_description sensor_publisher

# Terminal 3: Listen to topics
source /opt/ros/humble/setup.bash
ros2 topic echo /robot1/scan

# Terminal 4: Send velocity commands
source /opt/ros/humble/setup.bash
ros2 topic pub /robot1/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

### Step 15: Verify Unity Connection

Test the Unity connection:

```bash
# Terminal 1: Start Unity Bridge
cd ~/robotics_ws
source install/setup.bash
ros2 run my_robot_description unity_bridge

# You can test the WebSocket connection with a simple client
# Create test_websocket_client.py to test the connection
```

## Performance Optimization

### Step 16: Optimize Communication

Optimize ROS 2 communication for robotics applications:

1. **QoS Settings**: Use appropriate Quality of Service settings for different topics
2. **Throttling**: Implement message throttling for high-frequency topics
3. **Compression**: Use compression for large data like images

```python
# Example of optimized publisher with custom QoS
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For high-frequency sensor data
SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# For critical control commands
CONTROL_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# Use in publisher creation
lidar_publisher = self.create_publisher(LaserScan, '/robot1/scan', SENSOR_QOS)
```

## Troubleshooting

### Common Issues and Solutions

#### Issue 1: ROS Domain ID Conflicts
**Symptom**: ROS 2 nodes don't see each other
**Solution**:
```bash
# Set consistent domain ID
export ROS_DOMAIN_ID=0
# Or use different IDs for isolation
export ROS_DOMAIN_ID=1
```

#### Issue 2: Large Message Sizes
**Symptom**: Network performance issues with sensor data
**Solution**: Implement message compression or throttling

#### Issue 3: Timing Synchronization
**Symptom**: Data synchronization issues between systems
**Solution**: Use ROS 2 time synchronization and interpolation

## Next Steps

Congratulations! You've successfully set up ROS 2 integration for your digital twin system. Now you can:

1. Continue with advanced robotics algorithms
2. Integrate with navigation and perception stacks
3. Connect to Unity for visualization
4. Implement more complex sensor fusion
5. Set up synthetic data generation

## Resources

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Gazebo-ROS Integration: https://gazebosim.org/api/gazebo/6.0.0/ros_integration.html
- Unity ROS Bridge: https://github.com/Unity-Technologies/ROS-TCP-Connector
- Navigation2: https://navigation.ros.org/

## Summary

This tutorial covered the complete ROS 2 integration for robotics applications, including:
- ROS 2 installation and workspace setup
- Robot model creation with sensors
- Sensor data publishing and streaming
- ROS Bridge for Unity communication
- Performance optimization techniques
- Troubleshooting common issues

You now have a complete ROS 2 infrastructure ready for integration with Gazebo and Unity for digital twin applications.