# Deployment Guide for Models and Navigation Stack to Jetson Hardware

## Overview

This guide provides step-by-step instructions for deploying perception models and navigation stacks from simulation to NVIDIA Jetson hardware platforms, including setup, configuration, and validation procedures.

## Jetson Hardware Setup

### 1. Initial Setup
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Verify Jetson model and JetPack version
sudo jetson_release

# Set power mode for maximum performance during deployment
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks  # Lock clocks to maximum
```

### 2. Install Dependencies
```bash
# Install ROS 2 Humble
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Install Isaac ROS packages
sudo apt install -y \
    ros-humble-isaac-ros-common \
    ros-humble-isaac-ros-image-pipeline \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-point-cloud \
    ros-humble-isaac-ros-nitros
```

## Model Deployment

### 1. Copy Optimized Models
```bash
# Create model directory
mkdir -p ~/isaac_ros_ws/models

# Copy optimized models from development machine
# (Assuming models were optimized using TensorRT as per previous guide)
scp optimized_models/* jetson@jetson_ip:~/isaac_ros_ws/models/

# Set appropriate permissions
chmod -R 644 ~/isaac_ros_ws/models/
```

### 2. Navigation Stack Deployment

#### Build Navigation Stack
```bash
# Create workspace
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Clone Nav2 packages
git clone -b humble https://github.com/ros-planning/navigation2.git
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_navigator.git

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build navigation stack
colcon build --symlink-install --packages-select \
    nav2_common \
    nav2_map_server \
    nav2_amcl \
    nav2_navfn_planner \
    nav2_behavior_tree \
    nav2_bt_navigator \
    nav2_plan_utils \
    nav2_dwb_controller \
    nav2_smac_planner
```

#### Configuration Files
```yaml
# jetson_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.5
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0  # Higher for real-time performance
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 0.75
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: False

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: False
```

## Launch and Testing

### 1. Deployment Launch File
```python
# jetson_deployment_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default='jetson_nav2_params.yaml')

    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    jetson_config_dir = get_package_share_directory('jetson_deployment_config')

    # Navigation stack
    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup_dir,
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                jetson_config_dir,
                'config',
                params_file
            ])
        }.items()
    )

    # Isaac ROS perception container
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::tensor_rt::TensorRtNode',
                name='tensor_rt_node',
                parameters=[{
                    'engine_file_path': '/home/jetson/isaac_ros_ws/models/perception_model.plan',
                    'input_tensor_names': ['input'],
                    'output_tensor_names': ['output'],
                    'max_batch_size': 1,
                    'input_tensor_formats': ['nitros_tensor_list_nchw'],
                    'output_tensor_formats': ['nitros_tensor_list_nchw'],
                    'verbose': False,
                    'enable_profiling': False,
                    'collect_performance_data': False,
                }],
                remappings=[
                    ('tensor_sub', '/camera/image_rect'),
                    ('tensor_pub', '/detections'),
                ]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([
        navigation_stack,
        perception_container
    ])
```

### 2. Hardware Integration

#### Sensor Configuration
```bash
# Configure sensors for Jetson deployment
# This assumes you have a camera and LiDAR connected

# Start camera driver
ros2 launch realsense2_camera rs_launch.py

# Or start your specific camera driver
# ros2 launch your_camera_driver camera_launch.py

# Start LiDAR driver
ros2 launch your_lidar_driver lidar_launch.py
```

## Validation and Testing

### 1. System Health Check
```bash
# Check if all nodes are running
ros2 node list

# Check if all topics are publishing
ros2 topic list

# Monitor system resources
htop
nvidia-smi
```

### 2. Performance Validation
```bash
# Test perception pipeline
ros2 topic echo /detections --field header

# Test navigation
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "pose:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: 'map'
  pose:
    position:
      x: 1.0
      y: 1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0"
```

This deployment guide provides the essential steps for deploying perception models and navigation stacks from simulation to Jetson hardware platforms.