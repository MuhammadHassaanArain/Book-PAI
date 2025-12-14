# Isaac ROS Dependencies for Jetson Orin Platforms

## Jetson Orin Platform Overview

The NVIDIA Jetson Orin platform provides powerful AI computing capabilities for robotics applications. This document details the dependencies required for Isaac ROS on Jetson Orin platforms, including both Orin Nano and Orin NX variants.

## Hardware Specifications

### Jetson Orin Nano
- **GPU**: 1024-core NVIDIA Ampere architecture GPU
- **CPU**: 6-core ARM v8.2 64-bit CPU complex
- **Memory**: 4GB or 8GB LPDDR5
- **Power**: 7W to 15W (configurable)

### Jetson Orin NX
- **GPU**: 1024-core NVIDIA Ampere architecture GPU
- **CPU**: 8-core ARM v8.2 64-bit CPU complex
- **Memory**: 8GB or 16GB LPDDR5
- **Power**: 10W to 25W (configurable)

## Software Prerequisites

### 1. JetPack SDK
- **Required Version**: JetPack 5.1.2 or later
- **Includes**:
  - Linux for Tegra (L4T) R35.x
  - CUDA 11.4 or later
  - TensorRT 8.5 or later
  - cuDNN 8.6 or later
  - OpenCV 4.5.4 or later
  - VPI 2.0 or later

### 2. Ubuntu Version
- **Base OS**: Ubuntu 20.04 LTS (Ubuntu 22.04 support in development)
- **Kernel**: 5.10 or higher

## Isaac ROS Core Dependencies

### 1. ROS 2 Distribution
- **Required**: ROS 2 Humble Hawksbill (recommended)
- **Alternative**: ROS 2 Iron Irwini (for newer applications)

### 2. System Dependencies
```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    curl \
    gnupg \
    lsb-release \
    software-properties-common
```

### 3. Isaac ROS Specific Dependencies
```bash
# Install Isaac ROS dependencies
sudo apt install -y \
    ros-humble-isaac-ros-common \
    ros-humble-isaac-ros-image-ros \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-point-cloud-ros \
    ros-humble-isaac-ros-nitros
```

## CUDA and TensorRT Dependencies

### 1. CUDA Configuration
```bash
# Verify CUDA installation
nvidia-smi
nvcc --version

# Set CUDA environment variables
echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
echo 'export PATH=$CUDA_HOME/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 2. TensorRT Installation
```bash
# Install TensorRT
sudo apt install -y libnvinfer8 libnvonnxparsers8 libnvparsers8
sudo apt install -y libnvinfer-dev libnvonnxparsers-dev libnvparsers-dev
sudo apt install -y libnvinfer-plugin8
```

## Isaac ROS Package Dependencies

### 1. Perception Packages
```bash
# Isaac ROS AprilTag
sudo apt install -y ros-humble-isaac-ros-apriltag

# Isaac ROS AprilTag Detection
sudo apt install -r ros-humble-isaac-ros-detection2d-ros

# Isaac ROS Image Pipeline
sudo apt install -y ros-humble-isaac-ros-image-pipeline
```

### 2. Navigation Packages
```bash
# Isaac ROS Visual SLAM
sudo apt install -y ros-humble-isaac-ros-visual-slam

# Isaac ROS Point Cloud
sudo apt install -y ros-humble-isaac-ros-point-cloud

# Isaac ROS OAK Camera (for depth sensing)
sudo apt install -y ros-humble-isaac-ros-oak
```

### 3. Common Utilities
```bash
# Isaac ROS Common
sudo apt install -y ros-humble-isaac-ros-common

# Isaac ROS Message Types
sudo apt install -y ros-humble-isaac-ros-messages

# Isaac ROS Nitros (ROS 2 bridge)
sudo apt install -y ros-humble-isaac-ros-nitros
```

## Build Dependencies

### 1. Development Tools
```bash
# Install build tools
sudo apt install -y \
    python3-dev \
    python3-pip \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    python3-osrf-pycommon
```

### 2. Additional Libraries
```bash
# Install additional libraries
sudo apt install -y \
    libeigen3-dev \
    libopencv-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libpcl-dev \
    libgstreamer-plugins-base1.0-dev
```

## Memory and Storage Requirements

### 1. RAM Allocation
- **Minimum**: 4GB RAM (for Orin Nano)
- **Recommended**: 8GB+ RAM (for complex perception tasks)
- **Available for Applications**: ~70% of total RAM after system allocation

### 2. Storage Requirements
- **Root Filesystem**: 16GB minimum, 32GB recommended
- **Isaac ROS Installation**: 2-4GB
- **Model Storage**: 1-5GB per neural network model
- **Dataset Storage**: Variable (10GB+ for large datasets)

## Performance Optimization

### 1. Power Mode Configuration
```bash
# Check current power mode
sudo jetson_clocks --show

# Set to maximum performance mode
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks  # Lock clocks to maximum
```

### 2. Swap Configuration
```bash
# Create swap file for additional virtual memory
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 3. CPU Governor Optimization
```bash
# Set CPU governor to performance mode
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

## Network Configuration

### 1. ROS 2 Network Setup
```bash
# Set up ROS 2 domain
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# Configure Cyclone DDS
cat > ~/.ros/cyclone_dds_profile.xml << EOF
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="https://www.omg.org/dds">
  <qos_profile>
    <participant_qos>
      <rtps>
        <builtin>
          <discovery_config>
            <lease_duration>
              <sec>10</sec>
            </lease_duration>
          </discovery_config>
        </builtin>
      </rtps>
    </participant_qos>
  </qos_profile>
</dds>
EOF
```

### 2. Network Performance
```bash
# Optimize network buffers
echo 'net.core.rmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## Thermal Management

### 1. Temperature Monitoring
```bash
# Monitor Jetson thermal zones
sudo tegrastats  # Real-time monitoring
cat /sys/class/thermal/thermal_zone*/temp  # Temperature reading
```

### 2. Cooling Configuration
```bash
# Configure fan curve (if applicable)
sudo jetson_clocks --fan --show  # Show fan settings
```

## Troubleshooting Common Issues

### 1. CUDA Memory Issues
- **Issue**: Out of memory during inference
- **Solution**: Reduce batch size or use TensorRT optimization

### 2. ROS 2 Communication Issues
- **Issue**: Nodes not communicating
- **Solution**: Check domain ID, network configuration, and firewall settings

### 3. Performance Degradation
- **Issue**: Slower than expected inference
- **Solution**: Verify power mode, check thermal throttling, optimize models

## Verification Steps

### 1. System Verification
```bash
# Check Jetson platform
sudo jetson_release

# Verify CUDA
nvidia-smi
nvcc --version

# Check Isaac ROS packages
dpkg -l | grep isaac-ros
```

### 2. Isaac ROS Test
```bash
# Test Isaac ROS installation
ros2 run isaac_ros_common test_tensor_list
```

### 3. Performance Benchmark
```bash
# Run Isaac ROS benchmarks
ros2 launch isaac_ros_benchmark benchmark.launch.py
```

## Updating Dependencies

### 1. Package Updates
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Update Isaac ROS packages
sudo apt update
sudo apt upgrade ros-humble-isaac-ros-*
```

### 2. JetPack Updates
- Use NVIDIA SDK Manager for JetPack updates
- Always backup before updating JetPack
- Verify Isaac ROS compatibility after updates

This comprehensive dependency guide ensures proper setup of Isaac ROS on Jetson Orin platforms for optimal performance in robotics applications.