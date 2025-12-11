---
sidebar_position: 4
---

# Lab 4: Nav2 Autonomous Navigation in Simulation

## Objective

In this lab, students will configure and test the Navigation2 (Nav2) stack in simulation using Isaac Sim and ROS 2. Students will learn to set up navigation parameters for humanoid robots, implement global and local planning, and evaluate navigation performance in simulated environments.

## Prerequisites

- Completed Lab 1: Isaac Sim Installation & Scene Setup
- ROS 2 Humble installed
- Isaac Sim with ROS 2 bridge configured
- Basic understanding of navigation concepts
- Nav2 packages installed

## Estimated Time

4-5 hours

## Lab Overview

This lab focuses on configuring and testing the Nav2 stack for humanoid robot navigation in simulated environments. Students will learn to configure costmaps, planners, controllers, and recovery behaviors for bipedal navigation.

## Step 1: Install and Configure Nav2

### Install Nav2 Packages
```bash
# Install Nav2 packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-gui ros-humble-nav2-common -y

# Install additional dependencies
sudo apt install ros-humble-dwb-core ros-humble-dwb-msgs ros-humble-dwb-plugins ros-humble-nav2-msgs -y
sudo apt install ros-humble-robot-localization ros-humble-robot-state-publisher ros-humble-joint-state-publisher -y
```

### Create Navigation Workspace
```bash
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src

# Clone humanoid navigation packages if needed
git clone https://github.com/ros-planning/navigation2_tutorials.git
```

### Build Workspace
```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Step 2: Create Humanoid Robot Model

### URDF Model for Humanoid Navigation
Create a simplified humanoid robot model for navigation:

```xml
<!-- ~/nav2_ws/src/humanoid_robot/urdf/humanoid_base.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <xacro:property name="base_width" value="0.6"/>
  <xacro:property name="base_length" value="0.4"/>
  <xacro:property name="base_height" value="0.8"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Foot Links for Bipedal Support -->
  <joint name="left_foot_joint" type="fixed">
    <parent link="base_link"/>
    <child link="left_foot"/>
    <origin xyz="-0.1 -0.2 -0.4" rpy="0 0 0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.3 0.15 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
  </link>

  <joint name="right_foot_joint" type="fixed">
    <parent link="base_link"/>
    <child link="right_foot"/>
    <origin xyz="-0.1 0.2 -0.4" rpy="0 0 0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.3 0.15 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
  </link>

  <!-- Hokuyo Laser Range Finder -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="${base_width/2-0.05} 0 ${base_height/2}" rpy="0 0 0"/>
  </joint>

  <link name="laser_frame">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
  </link>

</robot>
```

## Step 3: Configure Nav2 for Humanoid Navigation

### Create Navigation Configuration
Create a configuration file for Nav2 parameters:

```yaml
# ~/nav2_ws/src/humanoid_robot/config/nav2_params_humanoid.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
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
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    navigate_through_poses_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    navigate_to_pose_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    behavior_tree:
      # The behavior tree to use for navigation
      default_nav_to_pose_bt_xml: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml
      default_nav_through_poses_bt_xml: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_have_feedback_condition_bt_node
    - nav2_have_odom_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_condition_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller configuration
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5  # Reduced for humanoid stability
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

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3  # Humanoid-specific radius
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
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
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3  # Humanoid-specific radius
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
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "turtlebot3_world.yaml"

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
      sim_frequency: 20.0
      forward_speed: 0.0
      rotate_speed: 1.0
      max_rotational_displacement: 1.57
    backup:
      plugin: "nav2_recoveries::BackUp"
      sim_frequency: 20.0
      forward_speed: -0.065
      backup_dist: 0.15
      cmd_topic: /cmd_vel
    wait:
      plugin: "nav2_recoveries::Wait"
      sim_frequency: 20.0
      wait_duration: 1.0

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

## Step 4: Create Navigation Launch Files

### Main Navigation Launch File
Create the main launch file for Nav2:

```python
# ~/nav2_ws/src/humanoid_robot/launch/nav2_humanoid_launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params/nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees/navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
            remappings=remappings),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=remappings)
    ])
```

## Step 5: Set Up Simulation Environment

### Create Simulation Launch File
```python
# ~/nav2_ws/src/humanoid_robot/launch/sim_nav2_humanoid.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Get package directories
    isaac_sim_dir = get_package_share_directory('isaac_ros_launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Isaac Sim bringup
    isaac_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(isaac_sim_dir, 'launch', 'isaac_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(get_package_share_directory('humanoid_robot'), 'config', 'nav2_params_humanoid.yaml')
        }.items()
    )

    # RViz2 configuration
    rviz_config = os.path.join(
        get_package_share_directory('humanoid_robot'),
        'rviz',
        'nav2_humanoid.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        isaac_sim_launch,
        nav2_launch,
        rviz_node
    ])
```

## Step 6: Configure RViz for Navigation

### Create RViz Configuration
```yaml
# ~/nav2_ws/src/humanoid_robot/rviz/nav2_humanoid.rviz
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /TF1/Frames1
        - /TF1/Tree1
        - /LaserScan1
        - /Map1
        - /Local Costmap1
        - /Global Costmap1
        - /Amcl Particle Cloud1
        - /Path1
        - /Robot1
      Splitter Ratio: 0.5833333134651184
    Tree Height: 865
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Local Costmap
      Topic:
        Depth: 1
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /local_costmap/costmap
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /local_costmap/costmap_updates
      Use Timestamp: false
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: costmap
      Draw Behind: false
      Enabled: true
      Name: Global Costmap
      Topic:
        Depth: 1
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /global_costmap/costmap
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /global_costmap/costmap_updates
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Path
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: Arrows
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /plan
      Value: true
    - Alpha: 0.6000000238418579
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: Robot
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: nav2_rviz_plugins/Navigation2Goal
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.7853981852531433
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.7853981852531433
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1043
  Hide Left Dock: false
  Hide Right Dock: true
  Navigation2Goal:
    collapsed: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001560000039bfc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000039b000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f0000039bfc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d0000039b000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000003a30000039b00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: true
  Width: 1853
  X: 67
  Y: 27
```

## Step 7: Execute Navigation in Simulation

### Build and Source Workspace
```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select humanoid_robot
source install/setup.bash
```

### Launch Simulation and Navigation
```bash
# Launch Isaac Sim with humanoid robot and Nav2
ros2 launch humanoid_robot sim_nav2_humanoid.launch.py
```

### Send Navigation Goals
In a new terminal:
```bash
# Source the workspace
cd ~/nav2_ws
source install/setup.bash

# Send a navigation goal using the Nav2 goal tool
ros2 run nav2_msgs navigation_goal.py --x 2.0 --y 2.0 --theta 0.0
```

## Step 8: Navigation Evaluation

### Create Evaluation Script
```python
# ~/nav2_ws/src/humanoid_robot/scripts/evaluate_navigation.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from collections import deque

class NavigationEvaluator(Node):
    def __init__(self):
        super().__init__('navigation_evaluator')

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Storage for evaluation metrics
        self.trajectory = []
        self.start_pose = None
        self.goal_pose = None
        self.path_length = 0.0
        self.executed_path = []

        # Timers for periodic evaluation
        self.eval_timer = self.create_timer(1.0, self.evaluate_performance)

    def path_callback(self, msg):
        self.path_length = self.calculate_path_length(msg.poses)

    def odom_callback(self, msg):
        # Store current position for trajectory analysis
        current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.executed_path.append(current_pos)

        if self.start_pose is None:
            self.start_pose = current_pos

    def scan_callback(self, msg):
        # Analyze scan data for obstacle proximity
        min_distance = min([d for d in msg.ranges if not math.isnan(d)])
        if min_distance < 0.5:  # Too close to obstacles
            self.get_logger().warn(f'Robot too close to obstacles: {min_distance:.2f}m')

    def calculate_path_length(self, poses):
        if len(poses) < 2:
            return 0.0

        length = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i-1].pose.position.x
            dy = poses[i].pose.position.y - poses[i-1].pose.position.y
            length += math.sqrt(dx*dx + dy*dy)

        return length

    def evaluate_performance(self):
        if len(self.executed_path) > 1:
            # Calculate trajectory efficiency
            start = self.executed_path[0]
            end = self.executed_path[-1]

            direct_distance = math.sqrt(
                (end[0] - start[0])**2 + (end[1] - start[1])**2
            )

            actual_distance = sum([
                math.sqrt((self.executed_path[i][0] - self.executed_path[i-1][0])**2 +
                         (self.executed_path[i][1] - self.executed_path[i-1][1])**2)
                for i in range(1, len(self.executed_path))
            ])

            efficiency = direct_distance / actual_distance if actual_distance > 0 else 0

            self.get_logger().info(
                f'Navigation Performance - '
                f'Direct Distance: {direct_distance:.2f}m, '
                f'Actual Distance: {actual_distance:.2f}m, '
                f'Efficiency: {efficiency:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    evaluator = NavigationEvaluator()
    rclpy.spin(evaluator)
    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab Assessment

### Knowledge Check Questions
1. What are the key differences between global and local planners in Nav2?
2. How do costmaps contribute to safe navigation for humanoid robots?
3. What are recovery behaviors and why are they important?

### Practical Assessment
- Successfully configure Nav2 for humanoid navigation
- Execute navigation in simulated environment
- Achieve 90%+ navigation success rate
- Demonstrate proper obstacle avoidance

### Deliverables
1. Configured Nav2 system for humanoid navigation
2. Navigation performance evaluation report
3. Video demonstration of navigation execution
4. Analysis of costmap and planner parameters

## Advanced Extensions

### Bipedal Constraint Navigation
- Implement step-by-step planning for bipedal locomotion
- Add balance constraints to navigation planning
- Evaluate navigation with bipedal-specific constraints

### Multi-Robot Navigation
- Extend to multi-robot navigation scenarios
- Implement collision avoidance between robots
- Evaluate coordination strategies

## Troubleshooting

### Common Issues
- **Localization failures**: Check sensor data quality and map accuracy
- **Planning failures**: Verify costmap configuration and inflation parameters
- **Oscillation**: Adjust controller parameters and goal tolerances
- **Performance issues**: Optimize costmap resolution and update rates

## References

1. Navigation2 Documentation: https://navigation.ros.org/
2. Nav2 Humanoid Navigation: https://github.com/ros-planning/navigation2
3. Costmap Configuration Guide: https://navigation.ros.org/configuration/index.html