---
sidebar_position: 5
---

# Lab 5: Sim-to-Real Deployment on Edge Hardware

## Objective

In this lab, students will deploy perception and navigation systems developed in simulation to real Jetson hardware. Students will learn to calibrate real sensors against simulation data, validate performance metrics, and implement safety procedures for real-world deployment.

## Prerequisites

- Completed Lab 1-4
- Jetson Orin Nano/NX with Ubuntu 22.04
- Isaac ROS packages installed
- Real sensors (camera, IMU, LiDAR if available)
- ROS 2 Humble on Jetson
- Trained perception models from simulation

## Estimated Time

5-6 hours

## Lab Overview

This lab focuses on the complete sim-to-real transfer process, from model export to real-world deployment. Students will learn to calibrate sensors, validate performance, and ensure safe operation of physical robots.

## Step 1: Prepare Models for Deployment

### Export Simulation-Trained Models

#### Export Perception Models
```bash
# Navigate to Isaac Sim workspace
cd ~/.local/share/ov/pkg/isaac_sim-*

# Export trained perception models from simulation
# (This is a general approach - specific to your trained models)

# For object detection models
python3 -c "
import omni
from omni.isaac.synthetic_utils.exportable_formats import ExportableModel

# Export model in TensorRT format for Jetson
model_exporter = ExportableModel('object_detection_model')
model_exporter.export_tensorrt('/path/to/export/detection_model.plan', precision='fp16')
"
```

#### Optimize Models for Jetson
```bash
# Install TensorRT optimization tools
sudo apt install tensorrt tensorrt-dev python3-libnvinfer-dev

# Optimize model for Jetson hardware
trtexec --onnx=model.onnx --saveEngine=model.plan --fp16 --workspace=1024 --device=0
```

### Prepare Navigation Maps
```bash
# Export navigation maps from simulation
# In Isaac Sim, save the generated map as a standard format
# Convert to ROS-compatible map format

# Create map configuration files
mkdir -p ~/nav2_ws/src/sim_to_real/maps
cp /path/to/exported/map.pgm ~/nav2_ws/src/sim_to_real/maps/real_world_map.pgm
cp /path/to/exported/map.yaml ~/nav2_ws/src/sim_to_real/maps/real_world_map.yaml
```

## Step 2: Sensor Calibration and Validation

### Camera Calibration
```bash
# Install camera calibration tools
sudo apt install ros-humble-camera-calibration

# Create calibration launch file
# ~/isaac_ros_ws/src/sim_to_real/launch/camera_calibration.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='camera_calibrator',
            arguments=[
                '--size', '8x6',  # Calibration board size
                '--square', '0.108',  # Square size in meters
                'image:=/camera/image_raw',
                'camera:=/camera'
            ]
        )
    ])
```

### Execute Camera Calibration
```bash
# Launch camera calibration
ros2 launch sim_to_real camera_calibration.launch.py

# Follow the calibration procedure:
# 1. Move the checkerboard pattern in front of the camera
# 2. Collect calibration images from different angles
# 3. Click "CALIBRATE" when sufficient images are collected
# 4. Click "SAVE" to save the calibration file
# 5. Click "COMMIT" to apply calibration

# The calibration file will be saved to ~/.ros/camera_info/
```

### IMU Calibration
```bash
# Install IMU calibration tools
sudo apt install ros-humble-rtimulib-ros2

# Create IMU calibration node
# ~/isaac_ros_ws/src/sim_to_real/scripts/imu_calibration.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.accel_data = []
        self.gyro_data = []
        self.calibration_samples = 1000

        self.get_logger().info('IMU Calibrator started. Keep robot stationary for calibration.')

    def imu_callback(self, msg):
        # Collect stationary IMU data for bias estimation
        self.accel_data.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.gyro_data.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        if len(self.accel_data) >= self.calibration_samples:
            self.compute_calibration()
            self.destroy_node()

    def compute_calibration(self):
        # Compute bias values
        accel_bias = np.mean(self.accel_data, axis=0)
        gyro_bias = np.mean(self.gyro_data, axis=0)

        # Expected gravity on Z-axis
        accel_bias[2] -= 9.81  # Remove gravity component

        self.get_logger().info(f'Calculated biases - Accel: {accel_bias}, Gyro: {gyro_bias}')

        # Save calibration to file
        np.save('imu_calibration.npy', {'accel_bias': accel_bias, 'gyro_bias': gyro_bias})

def main(args=None):
    rclpy.init(args=args)
    calibrator = IMUCalibrator()
    rclpy.spin(calibrator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 3: Real-World Sensor Integration

### Create Sensor Bridge Node
```python
# ~/isaac_ros_ws/src/sim_to_real/scripts/sensor_bridge.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')

        # Create publishers for ROS 2 topics
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Timer for sensor data publishing
        self.timer = self.create_timer(0.033, self.publish_sensor_data)  # ~30 FPS

        # Initialize camera capture
        self.cap = cv2.VideoCapture(0)  # Adjust index as needed
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')

        # Load camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_camera_calibration()

    def load_camera_calibration(self):
        """Load camera calibration parameters"""
        try:
            # Load calibration file (typically from camera_info_manager)
            # This is a simplified example - implement based on your camera setup
            self.get_logger().info('Camera calibration loaded')
        except Exception as e:
            self.get_logger().warn(f'Could not load camera calibration: {e}')

    def publish_sensor_data(self):
        """Publish sensor data from real hardware"""
        # Publish camera data
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_link'
            self.image_pub.publish(ros_image)

            # Publish camera info
            camera_info = CameraInfo()
            camera_info.header.stamp = ros_image.header.stamp
            camera_info.header.frame_id = 'camera_link'
            camera_info.width = frame.shape[1]
            camera_info.height = frame.shape[0]
            # Set camera matrix and distortion coefficients
            # (loaded from calibration file)
            self.info_pub.publish(camera_info)

        # Publish IMU data (simulated for this example)
        # In real implementation, read from actual IMU sensor
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        # Fill with actual IMU data in real implementation
        self.imu_pub.publish(imu_msg)

        # Publish laser scan data (simulated for this example)
        # In real implementation, read from actual LiDAR sensor
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_link'
        # Fill with actual scan data in real implementation
        self.scan_pub.publish(scan_msg)

    def destroy_node(self):
        """Clean up resources"""
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    sensor_bridge = SensorBridge()
    rclpy.spin(sensor_bridge)
    sensor_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 4: Deploy Perception Pipeline on Jetson

### Create Perception Launch File
```python
# ~/isaac_ros_ws/src/sim_to_real/launch/perception_pipeline.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    model_path = LaunchConfiguration('model_path', default='/path/to/model.plan')

    # Create container for perception pipeline
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image preprocessing
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect'),
                    ('camera_info_rect', '/camera/camera_info_rect'),
                ]
            ),
            # TensorRT inference
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::tensor_rt::TensorRtNode',
                name='tensor_rt_node',
                parameters=[{
                    'engine_file_path': model_path,
                    'input_tensor_names': ['input'],
                    'input_binding_names': ['input'],
                    'output_tensor_names': ['output'],
                    'output_binding_names': ['output'],
                    'max_batch_size': 1,
                    'input_tensor_formats': ['nitros_tensor_list_nchw'],
                    'output_tensor_formats': ['nitros_tensor_list_nchw'],
                    'verbose': False,
                    'enable_profiling': False,
                    'collect_performance_data': False,
                }],
                remappings=[
                    ('tensor_sub', '/rectify_node/image_rect'),
                    ('tensor_pub', 'detections'),
                ]
            ),
            # Detection processing
            ComposableNode(
                package='isaac_ros_detectnet',
                plugin='nvidia::isaac_ros::detection_based_segmentation::DetectNetNode',
                name='detectnet_node',
                parameters=[{
                    'input_image_width': 640,
                    'input_image_height': 480,
                    'confidence_threshold': 0.7,
                    'max_objects': 10,
                }],
                remappings=[
                    ('detections', 'tensor_pub'),
                    ('image', '/rectify_node/image_rect'),
                    ('camera_info', '/camera/camera_info_rect'),
                    ('detection_image', 'annotated_image'),
                    ('bbox_image', 'bbox_image'),
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        perception_container,
    ])
```

## Step 5: Deploy Navigation on Real Hardware

### Create Real Robot Configuration
```yaml
# ~/isaac_ros_ws/src/sim_to_real/config/real_robot.yaml
/**:
  ros__parameters:
    use_sim_time: false
    # Robot-specific parameters
    robot_description: "real_robot_description"
    # Controller parameters for real hardware
    controller:
      max_linear_velocity: 0.5  # Adjust for real robot capabilities
      max_angular_velocity: 0.75
      linear_acceleration: 0.5
      angular_acceleration: 1.0
    # Sensor parameters
    camera:
      frame_id: "camera_link"
      qos: 10
    imu:
      frame_id: "imu_link"
      qos: 10
```

### Create Deployment Launch File
```python
# ~/isaac_ros_ws/src/sim_to_real/launch/deploy_real.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration
    use_nav = LaunchConfiguration('use_nav', default='true')
    use_perception = LaunchConfiguration('use_perception', default='true')
    namespace = LaunchConfiguration('namespace', default='')

    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    sim_to_real_dir = get_package_share_directory('sim_to_real')

    # Group actions under namespace if specified
    launch_actions = []

    # Include perception pipeline if enabled
    perception_group = GroupAction(
        condition=IfCondition(use_perception),
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        get_package_share_directory('sim_to_real'),
                        'launch',
                        'perception_pipeline.launch.py'
                    ])
                ])
            )
        ]
    )
    launch_actions.append(perception_group)

    # Include navigation if enabled
    nav_group = GroupAction(
        condition=IfCondition(use_nav),
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        nav2_bringup_dir,
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': PathJoinSubstitution([
                        sim_to_real_dir,
                        'config',
                        'real_robot.yaml'
                    ])
                }.items()
            )
        ]
    )
    launch_actions.append(nav_group)

    # Add sensor bridge
    sensor_bridge_node = Node(
        package='sim_to_real',
        executable='sensor_bridge',
        name='sensor_bridge',
        parameters=[{
            'use_sim_time': False
        }],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/imu/data', '/imu/data'),
        ]
    )
    launch_actions.append(sensor_bridge_node)

    return LaunchDescription(launch_actions)
```

## Step 6: Safety and Validation Systems

### Create Safety Monitor Node
```python
# ~/isaac_ros_ws/src/sim_to_real/scripts/safety_monitor.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Bool
import numpy as np
import math

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publishers
        self.safety_pub = self.create_publisher(Bool, '/safety_status', 10)
        self.emergency_stop_pub = self.create_publisher(Twist, '/cmd_vel_emergency', 10)

        # Safety parameters
        self.min_obstacle_distance = 0.5  # meters
        self.max_angular_velocity = 1.0   # rad/s
        self.imu_calibration_threshold = 0.1  # m/s²

        # Robot state
        self.current_cmd = Twist()
        self.emergency_active = False
        self.safety_engaged = False

        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.check_safety)  # 10 Hz

    def cmd_vel_callback(self, msg):
        self.current_cmd = msg

    def scan_callback(self, msg):
        # Check for obstacles in path
        if len(msg.ranges) > 0:
            # Get ranges in forward direction (front 60 degrees)
            front_ranges = msg.ranges[:len(msg.ranges)//6] + msg.ranges[-len(msg.ranges)//6:]
            min_front_dist = min([r for r in front_ranges if not math.isnan(r)], default=float('inf'))

            if min_front_dist < self.min_obstacle_distance:
                self.get_logger().warn(f'Obstacle detected at {min_front_dist:.2f}m - TOO CLOSE!')

    def imu_callback(self, msg):
        # Check for unusual IMU readings (potential robot instability)
        linear_accel = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )

        if abs(linear_accel - 9.81) > self.imu_calibration_threshold:
            self.get_logger().warn(f'Unusual acceleration detected: {linear_accel:.2f} m/s²')

    def check_safety(self):
        """Main safety checking function"""
        safety_status = Bool()

        # Check velocity limits
        if abs(self.current_cmd.angular.z) > self.max_angular_velocity:
            self.get_logger().warn(f'Angular velocity limit exceeded: {self.current_cmd.angular.z:.2f}')

        # Determine overall safety status
        # In a real implementation, this would include more comprehensive checks
        safety_status.data = not self.emergency_active
        self.safety_pub.publish(safety_status)

        # Log safety status periodically
        if self.get_clock().now().nanoseconds % 5000000000 == 0:  # Every 5 seconds
            status_str = "SAFE" if safety_status.data else "UNSAFE"
            self.get_logger().info(f'Safety Status: {status_str}')

    def emergency_stop(self):
        """Execute emergency stop"""
        stop_cmd = Twist()
        self.emergency_stop_pub.publish(stop_cmd)
        self.emergency_active = True
        self.get_logger().error('EMERGENCY STOP ACTIVATED!')

def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()
    rclpy.spin(safety_monitor)
    safety_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 7: Performance Validation and Testing

### Create Performance Benchmarking Script
```python
# ~/isaac_ros_ws/src/sim_to_real/scripts/performance_benchmark.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
import numpy as np
from collections import deque

class PerformanceBenchmark(Node):
    def __init__(self):
        super().__init__('performance_benchmark')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publishers
        self.fps_pub = self.create_publisher(Float32, '/camera/fps', 10)
        self.cpu_usage_pub = self.create_publisher(Float32, '/system/cpu_usage', 10)
        self.memory_usage_pub = self.create_publisher(Float32, '/system/memory_usage', 10)

        # Performance tracking
        self.frame_times = deque(maxlen=30)  # Last 30 frames for FPS calculation
        self.processing_times = deque(maxlen=30)
        self.start_time = time.time()

        # Timer for performance reporting
        self.report_timer = self.create_timer(1.0, self.report_performance)

    def image_callback(self, msg):
        current_time = time.time()

        # Calculate frame time
        if hasattr(self, 'last_frame_time'):
            frame_time = current_time - self.last_frame_time
            self.frame_times.append(frame_time)

        self.last_frame_time = current_time

    def imu_callback(self, msg):
        # Track IMU processing time if needed
        pass

    def report_performance(self):
        if len(self.frame_times) > 0:
            avg_frame_time = np.mean(self.frame_times)
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0

            # Publish FPS
            fps_msg = Float32()
            fps_msg.data = float(fps)
            self.fps_pub.publish(fps_msg)

            # Get system metrics (simplified)
            # In real implementation, use psutil or similar
            cpu_usage_msg = Float32()
            cpu_usage_msg.data = 75.0  # Placeholder
            self.cpu_usage_pub.publish(cpu_usage_msg)

            memory_usage_msg = Float32()
            memory_usage_msg.data = 60.0  # Placeholder
            self.memory_usage_pub.publish(memory_usage_msg)

            # Log performance metrics
            self.get_logger().info(
                f'Performance - FPS: {fps:.2f}, '
                f'Avg Frame Time: {avg_frame_time*1000:.2f}ms, '
                f'CPU: {cpu_usage_msg.data}%, '
                f'Memory: {memory_usage_msg.data}%'
            )

def main(args=None):
    rclpy.init(args=args)
    benchmark = PerformanceBenchmark()
    rclpy.spin(benchmark)
    benchmark.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 8: Execute Real-World Deployment

### Build and Deploy
```bash
# Build the workspace
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sim_to_real
source install/setup.bash

# Verify all packages built successfully
colcon build --packages-select \
    isaac_ros_visual_slam \
    isaac_ros_image_pipeline \
    navigation2
```

### Launch Real-World System
```bash
# Launch the complete system
ros2 launch sim_to_real deploy_real.launch.py use_nav:=true use_perception:=true

# In another terminal, start safety monitoring
ros2 run sim_to_real safety_monitor

# In another terminal, start performance benchmarking
ros2 run sim_to_real performance_benchmark
```

### Execute Navigation Task
```bash
# Send a navigation goal
ros2 run nav2_msgs navigation_goal.py --x 2.0 --y 2.0 --theta 0.0

# Or use RViz2 for interactive navigation
source /opt/ros/humble/setup.bash
rviz2
```

## Step 9: Data Collection and Analysis

### Create Data Logging Script
```python
# ~/isaac_ros_ws/src/sim_to_real/scripts/data_collector.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import csv
import os
from datetime import datetime

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # Create timestamped directory for data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.data_dir = f'/home/jetson/data_collection_{timestamp}'
        os.makedirs(self.data_dir, exist_ok=True)

        # Initialize CSV files
        self.odom_file = open(f'{self.data_dir}/odometry.csv', 'w')
        self.imu_file = open(f'{self.data_dir}/imu.csv', 'w')

        self.odom_writer = csv.writer(self.odom_file)
        self.imu_writer = csv.writer(self.imu_file)

        # Write headers
        self.odom_writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.imu_writer.writerow(['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])

        # Subscriptions for data collection
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.get_logger().info(f'Data collection started in {self.data_dir}')

    def odom_callback(self, msg):
        # Log odometry data
        self.odom_writer.writerow([
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.odom_file.flush()

    def imu_callback(self, msg):
        # Log IMU data
        self.imu_writer.writerow([
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.imu_file.flush()

    def destroy_node(self):
        # Close files when node is destroyed
        self.odom_file.close()
        self.imu_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    collector = DataCollector()

    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('Data collection interrupted by user')
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab Assessment

### Knowledge Check Questions
1. What are the key challenges in transferring models from simulation to real hardware?
2. How does sensor calibration bridge the sim-to-real gap?
3. What safety measures are essential for real-world robot deployment?

### Practical Assessment
- Successfully deploy perception system on Jetson hardware
- Execute navigation in real-world environment
- Achieve performance within 80% of simulation benchmarks
- Demonstrate safety monitoring and emergency procedures

### Deliverables
1. Deployed perception and navigation system on Jetson
2. Performance comparison between simulation and real-world
3. Safety validation report
4. Data collection and analysis results

## Advanced Extensions

### Adaptive Transfer Learning
- Implement online learning to adapt models to real-world conditions
- Use domain adaptation techniques for continuous improvement
- Evaluate performance improvement over time

### Multi-Sensor Fusion Validation
- Validate sensor fusion algorithms in real-world conditions
- Compare simulation vs. real-world sensor data quality
- Implement sensor redundancy for robust operation

## Troubleshooting

### Common Issues
- **Model performance degradation**: Verify TensorRT optimization and calibration
- **Sensor synchronization**: Check timing and coordinate frame alignment
- **Performance bottlenecks**: Monitor CPU/GPU utilization and optimize pipeline
- **Safety system interference**: Ensure safety checks don't overly restrict operation

## References

1. NVIDIA Isaac ROS Sim-to-Real: https://github.com/NVIDIA-ISAAC-ROS
2. ROS 2 Safety Guidelines: https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html
3. Jetson Performance Optimization: https://developer.nvidia.com/embedded/jetson-developer-tools