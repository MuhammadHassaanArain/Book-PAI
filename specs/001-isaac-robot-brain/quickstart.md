# Quickstart Guide: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

## Prerequisites

Before starting this module, ensure you have:

- Ubuntu 22.04 LTS installed on your development machine
- NVIDIA GPU with CUDA support (RTX series recommended)
- ROS 2 Humble Hawksbill installed
- NVIDIA Isaac Sim LTS version installed
- Jetson Orin Nano/NX development kit (for real-world deployment)
- Basic knowledge of ROS 2 concepts and Python programming

## Environment Setup

### 1. Install ROS 2 Humble
```bash
# Follow official ROS 2 Humble installation guide for Ubuntu 22.04
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-ros-base
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 2. Install NVIDIA Isaac Sim
- Download Isaac Sim LTS from NVIDIA Developer website
- Follow installation instructions for your platform
- Verify installation by launching Isaac Sim

### 3. Set up Isaac ROS Workspace
```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Getting Started with the Module

### Chapter 1: Isaac Sim
1. Navigate to `/docs/module-3/chapter-1-isaac-sim/`
2. Start with `1.1-ai-robot-brain-intro.md`
3. Follow the exercises to set up your first simulation environment

### Chapter 2: Isaac ROS
1. Navigate to `/docs/module-3/chapter-2-isaac-ros/`
2. Start with `2.1-isaac-ros-overview.md`
3. Set up perception pipelines on your Jetson hardware

### Chapter 3: Nav2
1. Navigate to `/docs/module-3/chapter-3-nav2/`
2. Start with `3.1-nav-introduction.md`
3. Configure navigation for humanoid robots

### Chapter 4: Sim-to-Real Transfer
1. Navigate to `/docs/module-3/chapter-4-sim-to-real/`
2. Start with `4.1-sim-to-real-principles.md`
3. Learn to transfer models and navigation stacks to real hardware

## Lab Exercises

Complete the lab exercises in order:

1. `module-3-labs/lab-1-isaac-sim-setup.md` - Isaac Sim Installation & Scene Setup
2. `module-3-labs/lab-2-synthetic-dataset-generation.md` - Synthetic Dataset Generation for Object Detection
3. `module-3-labs/lab-3-isaac-ros-vslam.md` - Isaac ROS VSLAM Pipeline on Jetson
4. `module-3-labs/lab-4-nav2-simulation.md` - Nav2 Autonomous Navigation in Simulation
5. `module-3-labs/lab-5-sim-to-real-deployment.md` - Sim-to-Real Deployment on Edge Hardware
6. `module-3-labs/mini-project.md` - GPU-Accelerated Perception & Navigation Stack

## Assessment

Complete the assessment in `module-3-assessments/evaluation-methodology.md` to validate your understanding of the module concepts.

## Troubleshooting

- If Isaac Sim fails to launch, verify GPU drivers and CUDA installation
- For ROS 2 issues, ensure proper workspace sourcing
- For Jetson deployment issues, check network connectivity and hardware configuration