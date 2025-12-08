# Quickstart Guide: ROS 2 Robotic Nervous System Module

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed (or ROS 2 Iron Irwini)
- Python 3.10 or higher
- Docusaurus-compatible environment (Node.js, npm/yarn)

## Setup Instructions

### 1. Environment Setup
```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-base

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Install Python dependencies
pip3 install rclpy

# Install Docusaurus prerequisites
npm install
```

### 2. Clone and Prepare Repository
```bash
# Clone the textbook repository
git clone <repository-url>
cd <repository-name>

# Install dependencies
npm install

# Start the Docusaurus development server
npm run start
```

### 3. Verify ROS 2 Installation
```bash
# Check ROS 2 installation
ros2 --version

# Check available nodes, topics, services
ros2 node list
ros2 topic list
ros2 service list
```

## Running the Examples

### 1. Basic Publisher/Subscriber
```bash
# Terminal 1: Run the publisher
source /opt/ros/humble/setup.bash
cd <workspace>
ros2 run demo_nodes_cpp talker

# Terminal 2: Run the subscriber
source /opt/ros/humble/setup.bash
cd <workspace>
ros2 run demo_nodes_py listener
```

### 2. Service Client/Server
```bash
# Terminal 1: Run the service server
ros2 run demo_nodes_cpp add_two_ints_server

# Terminal 2: Run the service client
ros2 run demo_nodes_cpp add_two_ints_client 2 3
```

### 3. URDF Validation
```bash
# Validate a URDF file
check_urdf /path/to/your/robot.urdf

# Visualize in RViz
ros2 launch urdf_tutorial display.launch.py model:=/path/to/your/robot.urdf
```

## Module Structure

### Chapter 1: ROS 2 Architecture & Core Communication
- 1.1 Introduction to the Robotic Nervous System
- 1.2 ROS 2 vs ROS 1
- 1.3 DDS and ROS 2 Communication Model
- 1.4 Nodes, Executors, and Lifecycle
- 1.5 Domains, Namespaces, Remapping
- 1.6 Quality of Service (QoS) Policies

### Chapter 2: ROS 2 Nodes, Topics, Services & AI Bridging
- 2.1 Creating ROS 2 Python Packages
- 2.2 Publisher & Subscriber Nodes (rclpy)
- 2.3 Message Types and Interfaces
- 2.4 ROS 2 Services (Client–Server Model)
- 2.5 ROS 2 Actions for Long Tasks
- 2.6 Bridging Python AI Agents with ROS 2
- 2.7 Safety & Real-Time Constraints

### Chapter 3: Humanoid Robot Modeling with URDF & Launch Systems
- 3.1 Introduction to URDF for Humanoid Robots
- 3.2 Links, Joints, and Kinematic Chains
- 3.3 Actuators, Transmissions, and Sensors
- 3.4 Inertial, Visual, and Collision Properties
- 3.5 URDF Validation and Visualization Tools
- 3.6 ROS 2 Launch Files (Python)
- 3.7 Parameter Management with YAML

## Lab Exercises

### Lab 1: ROS 2 Publisher–Subscriber System
Create a simple publisher that sends robot sensor data and a subscriber that processes this data.

### Lab 2: Service & Action-Based Robot Control
Implement a service for robot commands and an action for long-running navigation tasks.

### Lab 3: Python AI Agent to ROS 2 Bridge
Connect a Python-based AI agent to ROS 2 topics and services.

### Lab 4: Humanoid URDF Design & Validation
Design a basic humanoid robot model in URDF and validate it.

### Mini Project: Full Software Nervous System
Integrate all concepts into a complete robotic system with AI integration.

## Common Issues and Solutions

### ROS 2 Environment Not Sourced
**Issue**: Commands like `ros2` not found
**Solution**: Run `source /opt/ros/humble/setup.bash` or add it to your `.bashrc`

### Python Import Errors
**Issue**: Cannot import rclpy
**Solution**: Ensure Python environment is properly configured and rclpy is installed

### URDF Validation Failures
**Issue**: check_urdf reports errors
**Solution**: Verify all joint connections and link definitions are correct

## Next Steps

After completing this module, you'll be ready to proceed to Module 2: Digital Twin (Simulation) where you'll learn to use these ROS 2 concepts in simulation environments.