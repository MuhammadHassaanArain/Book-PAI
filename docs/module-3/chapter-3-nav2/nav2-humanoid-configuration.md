# Nav2 Configuration Requirements for Humanoid Navigation

## Introduction

Navigation2 (Nav2) is the state-of-the-art navigation stack for ROS 2, designed for autonomous mobile robots. When configuring Nav2 for humanoid robots, special considerations must be made due to the unique kinematic and dynamic constraints of bipedal locomotion. This document outlines the specific configuration requirements for humanoid navigation using Nav2.

## Humanoid Robot Characteristics

### Kinematic Constraints
- **Degrees of Freedom**: Complex multi-link structure with 2+ legs
- **Balance Requirements**: Maintaining center of mass within support polygon
- **Step Planning**: Discrete footstep planning required
- **Limited Turning Radius**: Unlike wheeled robots, turning requires stepping

### Dynamic Constraints
- **Stability**: Balance must be maintained during motion
- **Inertia**: Higher moment of inertia compared to wheeled robots
- **Actuator Limits**: Joint torque and velocity constraints
- **Power Consumption**: Battery life considerations

## Nav2 Architecture Overview

### Core Components
1. **Global Planner**: Generates optimal path considering environment
2. **Local Planner**: Executes real-time trajectory generation and obstacle avoidance
3. **Controller**: Manages robot motion execution
4. **Recovery Behaviors**: Handles navigation failures

### Navigation System Architecture
```
[Global Planner] → [Local Planner] → [Controller] → [Robot]
      ↑                 ↑              ↑
[Costmap Global]  [Costmap Local]  [Sensors]
```

## Specific Configuration Requirements

### 1. Costmap Configuration

#### Global Costmap
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.4  # Larger radius for humanoid safety
      resolution: 0.05   # Fine resolution for precision
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
        cost_scaling_factor: 5.0  # Higher for humanoid safety
        inflation_radius: 0.8     # Larger safety buffer
      always_send_full_costmap: False
```

#### Local Costmap
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6  # Adjust based on humanoid size
      height: 6
      resolution: 0.05
      robot_radius: 0.4  # Match global costmap
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.8
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
```

### 2. Global Planner Configuration

#### A* or NavFn Planner for Humanoid
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5      # Larger tolerance for humanoid
      use_astar: true     # Use A* for better path quality
      allow_unknown: true
```

### 3. Local Planner Configuration

#### DWB Controller for Humanoid-Specific Motion
```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0  # Lower for humanoid stability
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.05
    min_theta_velocity_threshold: 0.05
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.05      # Minimum forward speed
      min_vel_y: 0.0
      max_vel_x: 0.3       # Reduced for stability
      max_vel_y: 0.0
      max_vel_theta: 0.3   # Reduced for stability
      min_speed_xy: 0.05
      max_speed_xy: 0.3
      min_speed_theta: 0.05
      acc_lim_x: 0.5       # Lower acceleration for balance
      acc_lim_y: 0.0
      acc_lim_theta: 0.5
      decel_lim_x: -0.5    # Lower deceleration for balance
      decel_lim_y: 0.0
      decel_lim_theta: -0.5
      vx_samples: 10       # Fewer samples for humanoid
      vy_samples: 1
      vtheta_samples: 20
      sim_time: 2.0        # Longer simulation time
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.3  # Larger tolerance for humanoid
      trans_stopped_velocity: 0.1
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.1
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

### 4. Behavior Tree Configuration

#### Custom Behavior Tree for Humanoid Navigation
```yaml
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
      xy_goal_tolerance: 0.3    # Larger for humanoid
      yaw_goal_tolerance: 0.3   # Larger for humanoid
      stateful: True
    navigate_to_pose_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.3    # Larger for humanoid
      yaw_goal_tolerance: 0.3   # Larger for humanoid
      stateful: True
    behavior_tree:
      default_nav_to_pose_bt_xml: /path/to/humanoid_nav_tree.xml
      default_nav_through_poses_bt_xml: /path/to/humanoid_nav_through_poses_tree.xml
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
```

### 5. Recovery Behaviors

#### Custom Recovery Behaviors for Humanoid
```yaml
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait", "humanoid_step_back"]
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait", "custom_humanoid_recovery::StepBack"]
    spin:
      plugin: "nav2_recoveries::Spin"
      sim_frequency: 10.0  # Lower for humanoid
      forward_speed: 0.0
      rotate_speed: 0.2    # Slower rotation for stability
      max_rotational_displacement: 1.57
    backup:
      plugin: "nav2_recoveries::BackUp"
      sim_frequency: 10.0
      forward_speed: -0.1  # Slower backward movement
      backup_dist: 0.3     # Shorter backup distance
      cmd_topic: /cmd_vel
    wait:
      plugin: "nav2_recoveries::Wait"
      sim_frequency: 20.0
      wait_duration: 2.0   # Shorter wait time
```

## Humanoid-Specific Considerations

### 1. Balance and Stability
- **Lower speeds**: Reduce maximum velocities for stability
- **Lower accelerations**: Use conservative acceleration limits
- **Larger safety margins**: Increase inflation radius and robot radius
- **Extended path planning**: Use longer simulation times

### 2. Footstep Planning Integration
- **Discrete path following**: Plan discrete steps rather than continuous paths
- **Balance constraints**: Ensure center of mass remains within support polygon
- **Step height limitations**: Consider maximum step height capabilities

### 3. Sensor Configuration
- **IMU integration**: Critical for balance feedback
- **Force/torque sensors**: For balance control
- **Additional safety sensors**: Multiple sensor types for redundancy

## Parameter Tuning Guidelines

### 1. Initial Configuration
- Start with conservative parameters
- Gradually increase velocities and accelerations
- Monitor balance and stability during testing

### 2. Tuning Process
1. **Costmap tuning**: Start with costmap parameters
2. **Controller tuning**: Adjust controller parameters
3. **Behavior tuning**: Fine-tune behavior tree and recovery behaviors
4. **Performance optimization**: Optimize for specific tasks

### 3. Safety Parameters
- **Emergency stops**: Configure emergency stop conditions
- **Timeouts**: Set appropriate timeout values
- **Error bounds**: Define acceptable error thresholds

## Testing and Validation

### 1. Simulation Testing
- Test in various simulated environments
- Validate safety parameters
- Verify recovery behaviors

### 2. Real-World Testing
- Start with simple, safe environments
- Gradually increase complexity
- Monitor all safety systems

### 3. Performance Metrics
- **Success rate**: Percentage of successful navigation
- **Path efficiency**: Actual vs optimal path length
- **Stability**: Balance maintenance during navigation
- **Safety**: Number of safety interventions

## Troubleshooting Common Issues

### 1. Oscillation
- **Cause**: Too aggressive controller parameters
- **Solution**: Reduce velocities and accelerations

### 2. Failure to Navigate
- **Cause**: Overly conservative parameters
- **Solution**: Gradually increase parameters

### 3. Balance Issues
- **Cause**: Insufficient safety margins
- **Solution**: Increase inflation radius and safety buffers

This configuration ensures safe and effective navigation for humanoid robots using the Nav2 stack, taking into account the unique challenges of bipedal locomotion and balance requirements.