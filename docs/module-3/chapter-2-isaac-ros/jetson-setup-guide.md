# Jetson Orin Nano/NX Setup Guide with Ubuntu 22.04 and ROS 2

## Overview

This guide provides detailed instructions for setting up NVIDIA Jetson Orin Nano and Orin NX development platforms with Ubuntu 22.04 and ROS 2 Humble Hawksbill. This setup is essential for deploying Isaac ROS perception and navigation pipelines on edge hardware.

## Hardware Requirements

### Jetson Orin Nano
- **Model**: NVIDIA Jetson Orin Nano Developer Kit
- **Power Supply**: 19V/65W AC adapter (included) or 5V/20V USB-C PD
- **Storage**: MicroSD card (32GB minimum, 64GB+ recommended)
- **Connectivity**: Ethernet cable or Wi-Fi for initial setup

### Jetson Orin NX
- **Model**: NVIDIA Jetson Orin NX Developer Kit
- **Power Supply**: 19V/65W AC adapter or 5V/20V USB-C PD
- **Storage**: Jetson Orin NX requires eMMC or NVMe SSD
- **Connectivity**: Ethernet recommended for initial setup

## Pre-Installation Requirements

### 1. Host System Requirements
- **Operating System**: Ubuntu 20.04/22.04, Windows 10/11, or macOS
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 10GB free space for SDK Manager
- **Internet**: Stable connection for downloads

### 2. Required Software
- **NVIDIA SDK Manager**: Latest version from NVIDIA Developer website
- **USB-C Cable**: For device connection and power
- **MicroSD Card Reader**: For Nano model setup (if using SD card)

## Installation Process

### Step 1: Install NVIDIA SDK Manager

#### On Ubuntu Host
```bash
# Download SDK Manager AppImage
wget https://developer.download.nvidia.com/devzone/devcenter/installer/serializer/IsV1Y25sYWRfbWFuYWdlcl9saW51eC54NjRfNjQ=/latest/sdmanager_cli/sdmanager_cli_latest_amd64.deb

# Install the package
sudo apt install ./sdmanager_cli_latest_amd64.deb

# Or run directly as AppImage
chmod +x sdkmanager*.AppImage
./sdkmanager*.AppImage
```

#### On Windows/macOS
- Download SDK Manager from NVIDIA Developer website
- Run the installer with administrative privileges

### Step 2: Configure SDK Manager

1. **Launch SDK Manager**
   - Sign in with NVIDIA Developer account
   - Accept terms and conditions

2. **Select Target Hardware**
   - Choose "Jetson Orin Nano" or "Jetson Orin NX"
   - Select "Setup" option

3. **Select Software Configuration**
   - **Target OS**: Linux (Ubuntu 22.04)
   - **JetPack Version**: 5.1.2 or latest LTS
   - **Additional Features**:
     - ROS 2 Humble Hawksbill
     - Isaac ROS packages
     - CUDA Toolkit
     - TensorRT
     - OpenCV

### Step 3: Prepare Jetson Device

#### For Jetson Orin Nano (SD Card)
1. Insert microSD card into card reader
2. Ensure card is properly detected by host system
3. Connect Jetson Nano to host via USB-C (both data and power)

#### For Jetson Orin NX (eMMC/SSD)
1. Connect Jetson NX to host via USB-C (both data and power)
2. Ensure device is in recovery mode (short pins 10 and 12 on J48 header)

### Step 4: Flash Jetson Device

1. **Begin Flashing Process**
   - SDK Manager will download required files
   - Process typically takes 30-60 minutes
   - Monitor progress in SDK Manager interface

2. **Post-Flash Configuration**
   - Device will reboot automatically
   - Complete initial Ubuntu setup:
     - Create user account
     - Set password
     - Configure network settings

### Step 5: Initial System Configuration

#### Connect to Network
```bash
# For wired connection (recommended for initial setup)
# Connect Ethernet cable to Jetson and network

# For wireless connection
nmcli device wifi list
nmcli device wifi connect "YourNetworkName" password "YourPassword"
```

#### Update System Packages
```bash
# Update package lists
sudo apt update

# Upgrade system packages
sudo apt upgrade -y

# Install additional utilities
sudo apt install -y vim htop curl wget git
```

## ROS 2 Humble Installation

### Verify ROS 2 Installation
```bash
# Check if ROS 2 Humble is already installed via SDK Manager
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO  # Should output "humble"
```

### If ROS 2 Not Installed via SDK Manager
```bash
# Set locale
locale  # Check for UTF-8
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

# Add ROS 2 repository
sudo apt update
sudo apt install -y ros-humble-desktop
```

### Install ROS 2 Dependencies
```bash
# Install additional ROS 2 tools
sudo apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
sudo rosdep init
rosdep update
```

## Isaac ROS Package Installation

### Install Core Isaac ROS Packages
```bash
# Update package lists
sudo apt update

# Install Isaac ROS common packages
sudo apt install -y \
    ros-humble-isaac-ros-common \
    ros-humble-isaac-ros-messages \
    ros-humble-isaac-ros-nitros

# Install Isaac ROS perception packages
sudo apt install -y \
    ros-humble-isaac-ros-image-pipeline \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-apriltag \
    ros-humble-isaac-ros-detection2d-ros \
    ros-humble-isaac-ros-point-cloud

# Install Isaac ROS sensor packages
sudo apt install -y \
    ros-humble-isaac-ros-gxf \
    ros-humble-isaac-ros-ros-bridge
```

### Verify Installation
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check installed packages
dpkg -l | grep isaac-ros

# Test basic Isaac ROS functionality
ros2 run isaac_ros_common test_tensor_list
```

## Environment Configuration

### Create ROS 2 Workspace
```bash
# Create workspace directory
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build workspace (even if empty initially)
colcon build --symlink-install
```

### Configure Environment Variables
```bash
# Add to ~/.bashrc for persistent configuration
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/isaac_ros_ws/install/setup.bash' >> ~/.bashrc

# Add Isaac ROS specific configurations
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
```

## System Optimization for Jetson

### Power Mode Configuration
```bash
# Check current power mode
sudo jetson_clocks --show

# Set to MAXN mode for maximum performance during development
sudo nvpmodel -m 0
sudo jetson_clocks

# For production, consider setting to lower power mode
# sudo nvpmodel -m 1  # For 15W mode
```

### Memory Configuration
```bash
# Increase swap space for compilation (optional)
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### CUDA Configuration
```bash
# Verify CUDA installation
nvidia-smi
nvcc --version

# Add CUDA to PATH (if not already done by JetPack)
echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
echo 'export PATH=$CUDA_HOME/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
```

## Network Configuration for ROS 2

### Configure DDS Middleware
```bash
# Create CycloneDDS configuration
mkdir -p ~/.ros
cat > ~/.ros/cyclonedds.xml << EOF
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="https://www.omg.org/dds">
  <qos_profile>
    <participant_qos>
      <rtps>
        <builtin>
          <participant_type>
            <static_discovery>
              <xml_file>~/.ros/cyclonedds_static_config.xml</xml_file>
            </static_discovery>
          </participant_type>
        </builtin>
      </rtps>
    </participant_qos>
  </qos_profile>
</dds>
EOF

# Set environment variable
echo 'export CYCLONEDDS_URI=file:///home/$(whoami)/.ros/cyclonedds.xml' >> ~/.bashrc
```

### Network Performance Optimization
```bash
# Optimize network buffers
echo 'net.core.rmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
echo 'net.core.rmem_default = 134217728' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## Testing the Setup

### Basic ROS 2 Test
```bash
# Open new terminal (to source environment)
# Or run: source /opt/ros/humble/setup.bash

# Test ROS 2 installation
ros2 topic list
ros2 service list
```

### Isaac ROS Test
```bash
# Test Isaac ROS installation
ros2 run isaac_ros_apriltag apriltag_node

# If successful, you should see node startup messages
# Press Ctrl+C to stop the node
```

### Hardware Verification
```bash
# Check GPU status
sudo tegrastats --one-shot

# Check available memory
free -h

# Check disk space
df -h
```

## Troubleshooting Common Issues

### 1. ROS 2 Environment Not Found
**Issue**: Command not found when running ROS 2 commands
**Solution**:
```bash
# Source the setup file manually
source /opt/ros/humble/setup.bash

# Or add to ~/.bashrc permanently
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 2. Isaac ROS Packages Not Installing
**Issue**: apt install fails for Isaac ROS packages
**Solution**:
```bash
# Update package lists
sudo apt update

# Check for held packages
sudo apt-mark showhold

# If needed, force configuration
sudo dpkg --configure -a
sudo apt install -f
```

### 3. CUDA Not Working
**Issue**: CUDA samples fail or nvidia-smi shows errors
**Solution**:
```bash
# Reinstall CUDA packages
sudo apt install --reinstall nvidia-jetpack

# Or check for driver issues
sudo apt install --reinstall nvidia-driver-*
```

### 4. Performance Issues
**Issue**: Low performance during perception tasks
**Solution**:
```bash
# Ensure power mode is set correctly
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks  # Lock clocks to maximum

# Check thermal status
sudo tegrastats  # Monitor for thermal throttling
```

## Security Considerations

### 1. User Permissions
```bash
# Add user to necessary groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
sudo usermod -a -G render $USER
```

### 2. Firewall Configuration
```bash
# Configure firewall for ROS 2 communication
sudo ufw allow 8883/tcp  # DDS communication
sudo ufw allow 11311/tcp # ROS master
sudo ufw allow 55555:55565/tcp  # Isaac Sim communication
```

## Next Steps

After completing this setup, you should be able to:

1. Run basic ROS 2 commands successfully
2. Launch Isaac ROS nodes
3. Execute perception and navigation pipelines
4. Connect to and communicate with sensors

Proceed to the next modules to implement specific perception and navigation capabilities using the Isaac ROS framework.