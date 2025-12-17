# Quickstart Guide: Module 1 — The Robotic Nervous System (ROS 2)

## Prerequisites

Before starting Module 1, ensure you have:

1. **Ubuntu 22.04 LTS** (recommended) or Ubuntu 24.04 LTS installed
2. **ROS 2 Humble Hawksbill** installed (or ROS 2 Iron Irwini)
3. **Python 3.10+**
4. **Basic Python programming knowledge**
5. **Linux command line familiarity**

### ROS 2 Installation (Ubuntu 22.04 - Humble)

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep2 python3-argcomplete
```

### Environment Setup

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to your bashrc to auto-source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Setting Up Your Workspace

Create a ROS 2 workspace for the textbook examples:

```bash
# Create workspace directory
mkdir -p ~/ros2_textbook_ws/src
cd ~/ros2_textbook_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace (even though it's empty)
colcon build

# Source the workspace
source install/setup.bash
```

## Running Your First ROS 2 Node

1. **Open a new terminal** and source your environment:
   ```bash
   cd ~/ros2_textbook_ws
   source install/setup.bash
   ```

2. **Run a simple publisher** (in first terminal):
   ```bash
   ros2 run demo_nodes_cpp talker
   ```

3. **In a second terminal**, source the environment and run a subscriber:
   ```bash
   cd ~/ros2_textbook_ws
   source install/setup.bash
   ros2 run demo_nodes_py listener
   ```

4. You should see the talker publishing messages and the listener receiving them.

## Understanding the ROS 2 Graph

Check the current ROS 2 graph state:

```bash
# List active nodes
ros2 node list

# List active topics
ros2 topic list

# Echo messages from a topic (while talker is running)
ros2 topic echo /chatter std_msgs/msg/String
```

## Working with the Textbook Content

The textbook content for Module 1 is organized as follows:

### Chapter 1: ROS 2 Architecture & Core Communication
- Start with `docs/module-1/chapter-1-ros2-architecture/1.1-introduction.md`
- Follow the content in order to build foundational knowledge
- Practice each code example in your workspace

### Chapter 2: ROS 2 Nodes, Topics, Services & AI Bridging
- Begin after completing Chapter 1
- Focus on practical implementation of communication patterns
- Includes AI integration examples

### Chapter 3: Humanoid Robot Modeling with URDF & Launch Systems
- Requires understanding from Chapters 1 and 2
- Focus on robot modeling and system orchestration
- Includes validation techniques

## Validation Tools

### URDF Validation
```bash
# Check URDF syntax
check_urdf /path/to/your/robot.urdf

# Parse URDF and show model structure
urdf_to_graphiz /path/to/your/robot.urdf
```

### Launch Files
```bash
# Run a launch file
ros2 launch package_name launch_file.py

# Run with arguments
ros2 launch package_name launch_file.py param_name:=value
```

## Docusaurus Setup for Textbook Viewing

If you want to view the textbook content in Docusaurus format:

1. **Install Node.js** (version 18.x or higher):
   ```bash
   curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
   sudo apt-get install -y nodejs
   ```

2. **Install project dependencies**:
   ```bash
   cd /path/to/your/textbook/repository
   npm install
   ```

3. **Start Docusaurus development server**:
   ```bash
   npm run start
   ```

4. Open your browser to `http://localhost:3000` to view the textbook.

## Troubleshooting Common Issues

### ROS 2 Environment Not Sourced
**Problem**: Commands like `ros2` not found
**Solution**: Source your ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

### Package Not Found
**Problem**: `Package 'package_name' not found`
**Solution**: Ensure your workspace is built and sourced:
```bash
cd ~/ros2_textbook_ws
colcon build
source install/setup.bash
```

### Permission Issues
**Problem**: Permission denied errors
**Solution**: Check file permissions or run with appropriate user rights.

## Next Steps

After completing this quickstart:
1. Proceed to Chapter 1, Topic 1.1 (Introduction to ROS 2)
2. Set up your development environment as described
3. Begin working through the textbook content systematically
4. Complete the lab exercises as you progress
5. Work on the mini-project at the end of Module 1