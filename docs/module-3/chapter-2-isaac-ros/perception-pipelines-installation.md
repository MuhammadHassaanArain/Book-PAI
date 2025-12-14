# Isaac ROS Perception Pipelines Installation Guide

## Overview

This guide provides comprehensive instructions for installing and configuring Isaac ROS perception pipelines on NVIDIA Jetson platforms. Isaac ROS perception packages leverage NVIDIA's GPU acceleration to provide high-performance computer vision capabilities for robotics applications.

## Prerequisites

### System Requirements
- **Platform**: NVIDIA Jetson Orin Nano/NX with JetPack 5.1.2+
- **OS**: Ubuntu 20.04 (recommended) or Ubuntu 22.04
- **ROS 2**: Humble Hawksbill installed
- **CUDA**: 11.4 or later (included with JetPack)
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 10GB free space for packages

### Required Software
- ROS 2 Humble installed and configured
- Isaac ROS common packages installed
- NVIDIA drivers and CUDA properly configured

## Installation Methods

### Method 1: Binary Installation (Recommended)

#### Update System
```bash
# Update package lists
sudo apt update

# Upgrade existing packages
sudo apt upgrade -y
```

#### Install Isaac ROS Perception Packages
```bash
# Install core perception packages
sudo apt install -y \
    ros-humble-isaac-ros-image-pipeline \
    ros-humble-isaac-ros-stereo-image-pipeline \
    ros-humble-isaac-ros-point-cloud \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-apriltag \
    ros-humble-isaac-ros-detection2d-ros \
    ros-humble-isaac-ros-segmentation \
    ros-humble-isaac-ros-gxf \
    ros-humble-isaac-ros-ros-bridge
```

#### Install Additional Dependencies
```bash
# Install additional dependencies for perception
sudo apt install -y \
    ros-humble-vision-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-message-filters \
    ros-humble-tf2-ros \
    ros-humble-cv-bridge \
    ros-humble-ros2-control \
    ros-humble-hardware-interface
```

### Method 2: Source Installation

#### Create Workspace
```bash
# Create Isaac ROS workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws
```

#### Clone Isaac ROS Perception Repositories
```bash
cd ~/isaac_ros_ws/src

# Clone Isaac ROS common packages
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_stereo_image_pipeline.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_detection2d.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_segmentation.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_point_cloud.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark.git
```

#### Install Dependencies
```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash

# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y
```

#### Build Packages
```bash
cd ~/isaac_ros_ws

# Build all Isaac ROS packages
colcon build --symlink-install --packages-select \
    isaac_ros_common \
    isaac_ros_image_pipeline \
    isaac_ros_stereo_image_pipeline \
    isaac_ros_visual_slam \
    isaac_ros_apriltag \
    isaac_ros_detection2d \
    isaac_ros_segmentation \
    isaac_ros_point_cloud
```

#### Source the Workspace
```bash
# Source the built workspace
source ~/isaac_ros_ws/install/setup.bash

# Add to bashrc for persistence
echo 'source ~/isaac_ros_ws/install/setup.bash' >> ~/.bashrc
```

## Individual Pipeline Installation

### 1. Image Pipeline
```bash
# Install image pipeline packages
sudo apt install -y ros-humble-isaac-ros-image-pipeline

# Verify installation
dpkg -l | grep isaac-ros-image-pipeline
```

#### Components:
- **Image Rectification**: Corrects lens distortion
- **Image Resizing**: Adjusts image dimensions
- **Format Conversion**: Converts between image formats

### 2. Stereo Image Pipeline
```bash
# Install stereo image pipeline
sudo apt install -y ros-humble-isaac-ros-stereo-image-pipeline

# Verify installation
dpkg -l | grep isaac-ros-stereo
```

#### Components:
- **Stereo Rectification**: Rectifies stereo image pairs
- **Disparity Computation**: Computes depth maps
- **Point Cloud Generation**: Converts disparity to 3D points

### 3. Visual SLAM Pipeline
```bash
# Install Visual SLAM packages
sudo apt install -y ros-humble-isaac-ros-visual-slam

# Verify installation
dpkg -l | grep isaac-ros-visual-slam
```

#### Components:
- **Feature Detection**: Detects visual features
- **Feature Tracking**: Tracks features across frames
- **Pose Estimation**: Estimates camera pose
- **Map Building**: Builds sparse map of environment

### 4. AprilTag Detection
```bash
# Install AprilTag packages
sudo apt install -y ros-humble-isaac-ros-apriltag

# Verify installation
dpkg -l | grep isaac-ros-apriltag
```

#### Components:
- **AprilTag Detection**: Detects AprilTag markers
- **Pose Estimation**: Estimates marker poses
- **Visualization**: Visualizes detected tags

### 5. 2D Detection Pipeline
```bash
# Install 2D detection packages
sudo apt install -y ros-humble-isaac-ros-detection2d-ros

# Verify installation
dpkg -l | grep isaac-ros-detection2d
```

#### Components:
- **Object Detection**: Detects objects in images
- **Classification**: Classifies detected objects
- **Tracking**: Tracks objects across frames

## Configuration and Optimization

### 1. GPU Memory Configuration
```bash
# Check available GPU memory
nvidia-smi

# For perception pipelines that require more memory
# Consider reducing resolution or batch size
```

### 2. Performance Tuning
```bash
# Set Jetson to maximum performance mode
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks  # Lock clocks to maximum
```

### 3. Environment Variables
```bash
# Add performance optimization variables to ~/.bashrc
echo 'export CUDA_DEVICE_ORDER=PCI_BUS_ID' >> ~/.bashrc
echo 'export CUDA_VISIBLE_DEVICES=0' >> ~/.bashrc
echo 'export GOMP_CPU_AFFINITY=0-5' >> ~/.bashrc  # For 6-core CPU
```

## Testing Individual Pipelines

### 1. Test Image Pipeline
```bash
# Source ROS 2 and Isaac ROS
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash  # if built from source

# Test image rectification node
ros2 run isaac_ros_image_pipeline rectify_node
```

### 2. Test AprilTag Pipeline
```bash
# Launch AprilTag detection
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

### 3. Test Visual SLAM
```bash
# Launch Visual SLAM pipeline
ros2 launch isaac_ros_visual_slam visual_slam_node.launch.py
```

## Container-Based Installation (Optional)

### Using Docker
```bash
# Install Docker if not already installed
sudo apt install docker.io -y
sudo usermod -aG docker $USER

# Pull Isaac ROS perception containers
docker pull nvcr.io/nvidia/isaac-ros/isaac_ros_visual_slam:latest
docker pull nvcr.io/nvidia/isaac-ros/isaac_ros_apriltag:latest
docker pull nvcr.io/nvidia/isaac-ros/isaac_ros_image_pipeline:latest
```

### Run Perception Pipeline in Container
```bash
# Example: Run AprilTag detection in container
docker run --gpus all \
    --rm -it \
    --network host \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env DISPLAY=$DISPLAY \
    --env TERM=xterm-256color \
    --env QT_X11_NO_MITSHM=1 \
    nvcr.io/nvidia/isaac-ros/isaac_ros_apriltag:latest
```

## Verification and Validation

### 1. Package Verification
```bash
# Check all installed Isaac ROS packages
dpkg -l | grep isaac-ros

# Verify specific packages
ros2 pkg list | grep isaac
```

### 2. Node Verification
```bash
# List available Isaac ROS nodes
ros2 node list

# Check if perception nodes are available
ros2 run | grep isaac
```

### 3. Launch File Verification
```bash
# Find available launch files
find /opt/ros/humble/share -name "*launch*" | grep isaac
```

## Troubleshooting

### 1. Installation Failures
**Issue**: Package installation fails with dependency errors
**Solution**:
```bash
# Update package lists
sudo apt update

# Fix broken dependencies
sudo apt --fix-broken install

# Try installing packages individually
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-image-pipeline
```

### 2. Missing Dependencies
**Issue**: Missing dependencies during installation
**Solution**:
```bash
# Install rosdep if not present
sudo apt install python3-rosdep

# Update rosdep database
sudo rosdep init
rosdep update

# Install missing dependencies
rosdep install --from-paths ~/isaac_ros_ws/src --ignore-src -r -y
```

### 3. GPU Memory Issues
**Issue**: Perception nodes fail due to insufficient GPU memory
**Solution**:
```bash
# Check GPU memory usage
nvidia-smi

# Reduce image resolution in pipeline configuration
# Or run fewer nodes simultaneously
```

### 4. Permission Issues
**Issue**: Cannot access GPU or run nodes
**Solution**:
```bash
# Add user to necessary groups
sudo usermod -a -G video $USER
sudo usermod -a -G render $USER

# Log out and log back in for changes to take effect
```

## Performance Optimization

### 1. Pipeline Optimization
```bash
# Reduce image resolution for faster processing
# Modify camera parameters to lower resolution
# Use TensorRT optimization where available
```

### 2. Memory Management
```bash
# Monitor memory usage
htop
nvidia-smi

# Configure swap space if needed
sudo fallocate -l 2G /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## Integration Testing

### Complete Pipeline Test
```bash
# Create a test launch file that combines multiple perception nodes
mkdir -p ~/isaac_ros_ws/src/test_launch
cd ~/isaac_ros_ws/src/test_launch

# Create test launch file
cat > test_perception_pipeline.launch.py << EOF
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_image_pipeline',
                plugin='nvidia::isaac_ros::image_pipeline::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect'),
                ]
            ),
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                parameters=[{
                    'input_width': 640,
                    'input_height': 480,
                }],
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('camera_info', '/camera/camera_info'),
                    ('detections', '/apriltag_detections'),
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([perception_container])
EOF

# Test the combined pipeline
source ~/isaac_ros_ws/install/setup.bash
ros2 launch test_launch test_perception_pipeline.launch.py
```

## Next Steps

After successful installation of Isaac ROS perception pipelines:

1. **Configure specific sensors** (cameras, LiDAR, etc.)
2. **Calibrate cameras** for accurate perception
3. **Test individual components** with sample data
4. **Integrate with navigation systems**
5. **Optimize for specific use cases**

This installation provides the foundation for advanced perception capabilities using Isaac ROS on Jetson platforms.