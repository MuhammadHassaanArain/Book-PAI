---
sidebar_position: 3
---

# Lab 3: Isaac ROS VSLAM Pipeline on Jetson

## Objective

In this lab, students will implement and deploy a Visual Simultaneous Localization and Mapping (VSLAM) pipeline using NVIDIA Isaac ROS on a Jetson platform. Students will learn to configure Isaac ROS VSLAM components, integrate with sensor data, and evaluate performance in both simulated and real-world scenarios.

## Prerequisites

- Completed Lab 1: Isaac Sim Installation & Scene Setup
- Jetson Orin Nano/NX with Ubuntu 22.04
- Isaac ROS packages installed
- Basic understanding of SLAM concepts
- ROS 2 Humble installed on Jetson

## Estimated Time

4-5 hours

## Lab Overview

This lab focuses on implementing a VSLAM pipeline using Isaac ROS components on Jetson hardware. Students will learn to configure the pipeline, process sensor data in real-time, and evaluate mapping and localization performance.

## Step 1: Jetson Platform Setup

### Prepare Jetson Hardware
1. Ensure Jetson Orin Nano/NX has Ubuntu 22.04 installed
2. Update system packages:
```bash
sudo apt update && sudo apt upgrade -y
```

### Install CUDA and Dependencies
```bash
# Verify CUDA installation
nvcc --version

# Install additional dependencies
sudo apt install build-essential cmake pkg-config libeigen3-dev libopencv-dev -y
```

### Install ROS 2 Humble
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Install ROS 2 Humble packages
sudo apt update && sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
source /opt/ros/humble/setup.bash
```

## Step 2: Install Isaac ROS VSLAM Packages

### Set Up Workspace
```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
```

### Clone Isaac ROS VSLAM Packages
```bash
cd ~/isaac_ros_ws/src

# Clone essential Isaac ROS packages
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git  # For calibration
```

### Build Isaac ROS Packages
```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build packages
colcon build --symlink-install --packages-select \
    isaac_ros_visual_slam \
    isaac_ros_image_pipeline \
    isaac_ros_common \
    isaac_ros_nitros \
    isaac_ros_apriltag
```

## Step 3: Configure VSLAM Pipeline

### Create Launch File
Create a launch file for the VSLAM pipeline:

```python
# ~/isaac_ros_ws/src/vslam_pipeline/launch/vslam_pipeline.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for Isaac ROS VSLAM pipeline."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Create container for VSLAM pipeline
    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'enable_rectified_pose': True,
                    'map_frame': 'map',
                    'odometry_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_observations_view': True,
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_intra_proc': True
                }],
                remappings=[
                    ('/visual_slam/image_raw', '/camera/image_raw'),
                    ('/visual_slam/camera_info', '/camera/camera_info'),
                    ('/visual_slam/imu', '/imu/data')
                ]
            ),
            # Add image preprocessing nodes
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect')
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([vslam_container])
```

### Create Configuration Files
Create a configuration file for VSLAM parameters:

```yaml
# ~/isaac_ros_ws/src/vslam_pipeline/config/vslam_config.yaml
visual_slam:
  ros__parameters:
    enable_rectified_pose: true
    map_frame: "map"
    odometry_frame: "odom"
    base_frame: "base_link"
    enable_observations_view: true
    enable_slam_visualization: true
    enable_landmarks_view: true
    enable_intra_proc: true
    max_num_landmarks: 1000
    min_num_images_per_keyframe: 3
    min_num_images_per_graph_edge: 3
    max_num_features: 1000
    min_num_keyframes_for_local_ba: 5
    max_num_keyframes_in_local_ba: 15
    max_num_keyframes_in_global_ba: 50
    enable_localization: true
    enable_mapping: true
```

## Step 4: Sensor Integration

### Camera Setup
1. Connect a stereo camera or RGB-D camera to the Jetson
2. Configure camera drivers (e.g., for RealSense, ZED, or custom cameras)

### IMU Integration
1. Connect IMU sensor to Jetson
2. Configure IMU driver and calibration
3. Verify IMU data publication

### Create Sensor Bridge
```python
# ~/isaac_ros_ws/src/vslam_pipeline/scripts/sensor_bridge.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')

        # Create publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Timer for publishing data
        self.timer = self.create_timer(0.033, self.publish_data)  # ~30 FPS

    def publish_data(self):
        # Capture and publish camera data
        # This is a simplified example - implement based on your camera
        pass

def main(args=None):
    rclpy.init(args=args)
    sensor_bridge = SensorBridge()
    rclpy.spin(sensor_bridge)
    sensor_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 5: Run VSLAM Pipeline on Jetson

### Source Workspace
```bash
cd ~/isaac_ros_ws
source install/setup.bash
source /opt/ros/humble/setup.bash
```

### Launch VSLAM Pipeline
```bash
# Launch the VSLAM pipeline
ros2 launch vslam_pipeline vslam_pipeline.launch.py
```

### Visualize Results
```bash
# In another terminal, launch RViz2
source /opt/ros/humble/setup.bash
rviz2
```

### Configure RViz2 for VSLAM
1. Add TF display to visualize transforms
2. Add PointCloud2 display for landmarks
3. Add PoseStamped display for trajectory
4. Add Image display for camera feed

## Step 6: Performance Benchmarking

### Create Benchmarking Script
```python
# ~/isaac_ros_ws/src/vslam_pipeline/scripts/benchmark_vslam.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time
import numpy as np

class VSLAMBenchmark(Node):
    def __init__(self):
        super().__init__('vslam_benchmark')

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/pose',
            self.pose_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Initialize benchmarking variables
        self.processing_times = []
        self.fps_values = []
        self.start_time = time.time()

        # Timer for periodic benchmarking
        self.benchmark_timer = self.create_timer(1.0, self.report_benchmark)

    def image_callback(self, msg):
        start_time = time.time()
        # Simulate processing time
        # In real scenario, measure actual processing time
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)

    def pose_callback(self, msg):
        # Record pose processing
        pass

    def report_benchmark(self):
        if self.processing_times:
            avg_processing_time = np.mean(self.processing_times[-30:])  # Last 30 samples
            avg_fps = 1.0 / avg_processing_time if avg_processing_time > 0 else 0

            self.get_logger().info(
                f'VSLAM Performance - '
                f'Avg Processing Time: {avg_processing_time:.4f}s, '
                f'Avg FPS: {avg_fps:.2f}, '
                f'Latency: {msg.header.stamp.sec - self.start_time:.2f}s'
            )

            # Clear old measurements
            self.processing_times = self.processing_times[-30:]

def main(args=None):
    rclpy.init(args=args)
    benchmark = VSLAMBenchmark()
    rclpy.spin(benchmark)
    benchmark.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run Benchmarking
```bash
# Run benchmarking in parallel with VSLAM
ros2 run vslam_pipeline benchmark_vslam.py
```

## Step 7: Evaluation and Testing

### Map Quality Assessment
1. Evaluate map completeness and accuracy
2. Check for loop closure performance
3. Assess trajectory estimation quality
4. Measure computational resource usage

### Localization Accuracy
1. Test relocalization capabilities
2. Measure pose estimation accuracy
3. Evaluate performance in different environments
4. Assess robustness to lighting changes

### Resource Utilization
1. Monitor GPU utilization: `nvidia-smi`
2. Check CPU usage: `htop`
3. Monitor memory usage: `free -h`
4. Track power consumption on Jetson

## Lab Assessment

### Knowledge Check Questions
1. What are the key components of a VSLAM system?
2. How does Isaac ROS optimize VSLAM performance on Jetson hardware?
3. What are the advantages of visual-inertial SLAM over visual-only SLAM?

### Practical Assessment
- Successfully deploy VSLAM pipeline on Jetson
- Achieve real-time performance (30+ FPS)
- Generate accurate maps and trajectories
- Demonstrate localization capabilities

### Deliverables
1. Working VSLAM pipeline on Jetson
2. Performance benchmarking report
3. Map and trajectory visualization
4. Resource utilization analysis

## Advanced Extensions

### Multi-Sensor Fusion
- Integrate LiDAR with visual SLAM
- Implement sensor fusion algorithms
- Evaluate performance improvements

### Dynamic Object Handling
- Detect and track dynamic objects
- Exclude dynamic objects from mapping
- Implement dynamic scene understanding

## Troubleshooting

### Common Issues
- **Performance issues**: Check GPU utilization and optimize pipeline
- **Calibration errors**: Verify camera and IMU calibration
- **Tracking failures**: Adjust VSLAM parameters for better tracking
- **Memory issues**: Monitor and optimize memory usage

## References

1. NVIDIA Isaac ROS Visual SLAM: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
2. Real-Time Visual SLAM: https://ieeexplore.ieee.org/document/9099132
3. Jetson Performance Optimization: https://developer.nvidia.com/embedded/jetson-developer-tools