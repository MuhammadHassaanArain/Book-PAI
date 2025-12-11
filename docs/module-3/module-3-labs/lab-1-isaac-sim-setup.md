---
sidebar_position: 1
---

# Lab 1: Isaac Sim Installation & Scene Setup

## Objective

In this lab, students will install NVIDIA Isaac Sim, configure the environment, and create a basic humanoid robot scene to verify the installation and understand the fundamental concepts of photorealistic simulation.

## Prerequisites

- Ubuntu 22.04 LTS
- NVIDIA GPU with CUDA support (RTX series recommended)
- NVIDIA Driver version 535 or higher
- ROS 2 Humble Hawksbill
- Docker (optional but recommended)

## Estimated Time

2-3 hours

## Lab Environment Requirements

### Hardware Requirements
- CPU: 8+ cores (Intel i7 or AMD Ryzen 7)
- RAM: 32GB or more
- GPU: NVIDIA RTX 3070 or better (12GB+ VRAM)
- Storage: 50GB free space for Isaac Sim installation

### Software Requirements
- Ubuntu 22.04 LTS
- NVIDIA GPU drivers (535+)
- CUDA 11.8 or higher
- Isaac Sim LTS (2023.1.0 or later)
- Omniverse Launcher

## Step 1: System Preparation

### Update System Packages
```bash
sudo apt update && sudo apt upgrade -y
```

### Install NVIDIA Drivers
```bash
sudo apt install nvidia-driver-535 nvidia-utils-535 -y
sudo reboot
```

### Verify GPU Installation
```bash
nvidia-smi
```

## Step 2: Install Isaac Sim

### Download Omniverse Launcher
1. Go to [NVIDIA Omniverse](https://developer.nvidia.com/isaac-sim)
2. Create an NVIDIA Developer account if you don't have one
3. Download the Omniverse Launcher for Linux

### Install and Configure Omniverse Launcher
```bash
# Make the installer executable
chmod +x omniverse-launcher-linux.AppImage

# Run the installer
./omniverse-launcher-linux.AppImage
```

### Install Isaac Sim through Omniverse Launcher
1. Launch Omniverse Launcher
2. Sign in with your NVIDIA Developer account
3. Search for "Isaac Sim"
4. Click "Install" to install Isaac Sim LTS

## Step 3: Verify Isaac Sim Installation

### Launch Isaac Sim
```bash
# Navigate to Isaac Sim installation directory (typically in ~/.local/share/ov/pkg/isaac_sim-*)
cd ~/.local/share/ov/pkg/isaac_sim-*
./isaac-sim.sh
```

### Basic Scene Creation
1. Open Isaac Sim
2. Create a new scene (File → New Scene)
3. Verify that the viewport renders correctly
4. Test basic navigation controls:
   - Orbit: Right mouse button + drag
   - Pan: Middle mouse button + drag
   - Zoom: Mouse wheel

## Step 4: Install ROS 2 Bridge

### Clone Isaac ROS Bridge
```bash
# Create workspace
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws/src

# Clone Isaac ROS bridge
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
```

### Build Isaac ROS Bridge
```bash
cd ~/isaac_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select isaac_ros_common
```

## Step 5: Create Basic Humanoid Scene

### Import a Simple Robot
1. In Isaac Sim, go to Window → Content → Content Browser
2. Navigate to `/Isaac/Robots/`
3. Drag and drop a simple robot (e.g., `Carter/carter_vda5.urdf`) into the scene
4. Verify that the robot appears in the viewport

### Set Up Basic Lighting
1. Add a dome light: Create → Light → Dome Light
2. Set the dome light texture to `/Isaac/Environments/Simple_Room_TXO/simple_room_2k.hdr`
3. Add a key light: Create → Light → Distant Light
4. Position the distant light to illuminate the scene

### Configure Physics
1. Ensure Physics is enabled (Window → Simulation → Physics)
2. Set gravity to -9.81 m/s² in the Z direction
3. Verify that rigid bodies respond to physics

## Step 6: Test Simulation

### Run Basic Simulation
1. Press the "Play" button in the timeline
2. Observe the robot's response to physics
3. Stop the simulation using the "Stop" button

### Test Camera View
1. Create a camera: Create → Camera
2. Position the camera to view the robot
3. Test real-time rendering by moving the camera

## Step 7: Export URDF (Optional)

### Prepare Robot for URDF Export
1. Select your robot in the scene
2. Go to Isaac Sim menu: Isaac → URDF Exporter
3. Configure export settings for ROS 2 compatibility
4. Export the robot as URDF

## Lab Assessment

### Knowledge Check Questions
1. What is the minimum GPU requirement for running Isaac Sim effectively?
2. What is the purpose of domain randomization in Isaac Sim?
3. How does Isaac Sim integrate with ROS 2?

### Practical Assessment
- Successfully install Isaac Sim without errors
- Create a basic scene with a robot model
- Verify that physics simulation works correctly
- Demonstrate basic camera and lighting setup

### Deliverables
1. Screenshot of Isaac Sim with your basic scene
2. Console output showing successful installation
3. Brief report on any challenges encountered during installation

## Troubleshooting

### Common Installation Issues
- **GPU not detected**: Ensure NVIDIA drivers are properly installed
- **CUDA version mismatch**: Verify CUDA 11.8+ is installed
- **Insufficient permissions**: Run Isaac Sim with appropriate user permissions
- **OpenGL errors**: Check graphics driver compatibility

### Performance Issues
- **Slow rendering**: Reduce viewport quality settings temporarily
- **Memory errors**: Close other applications to free up RAM/GPU memory
- **Physics instability**: Check timestep settings in physics configuration

## References

1. NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html
2. NVIDIA Omniverse System Requirements: https://docs.omniverse.nvidia.com/sys/system-requirements.html
3. ROS 2 Humble Installation Guide: https://docs.ros.org/en/humble/Installation.html