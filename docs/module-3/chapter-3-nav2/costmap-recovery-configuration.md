# Configuring Costmaps, Obstacle Layers, and Recovery Behaviors

## Overview

This tutorial provides comprehensive guidance on configuring costmaps, obstacle layers, and recovery behaviors in the Navigation2 (Nav2) stack. The tutorial covers both theoretical concepts and practical implementation, with a focus on humanoid robot navigation requirements.

## Understanding Costmaps in Nav2

### Costmap Fundamentals

#### What are Costmaps?
Costmaps in Nav2 are 2D or 3D grid-based representations of the environment that assign costs to different areas based on obstacles, inflation, and other factors. They serve as the primary representation for navigation algorithms to understand the environment.

#### Types of Costmaps
- **Global Costmap**: Used by global planners to understand static obstacles and map layout
- **Local Costmap**: Used by local planners to handle dynamic obstacles and immediate environment

#### Cost Values
- **FREE_SPACE (0)**: Navigable area
- **LETHAL_OBSTACLE (254)**: Definite obstacle
- **INSCRIBED_INFLATED_OBSTACLE (253)**: Area too close to obstacles
- **UNKNOWN (255)**: Unknown space (if track_unknown_space is enabled)

### Costmap Configuration Structure

#### Basic Costmap Parameters
```yaml
# costmap_common_params.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # Update frequency in Hz
      update_frequency: 1.0

      # Publish frequency in Hz
      publish_frequency: 1.0

      # Global frame of reference (usually "map" for global, "odom" for local)
      global_frame: map

      # Robot base frame
      robot_base_frame: base_link

      # Use simulation time (for testing with bag files or simulation)
      use_sim_time: false

      # Robot radius for collision checking
      robot_radius: 0.3

      # Resolution of the costmap (meters per cell)
      resolution: 0.05

      # Track unknown space (assigns cost 255 to unknown areas)
      track_unknown_space: true

      # Plugins to be loaded for this costmap
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

local_costmap:
  local_costmap:
    ros__parameters:
      # Same parameters as global but with local-specific values
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom  # Usually "odom" for local costmap
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true  # Local costmap rolls with robot
      width: 6.0  # Width of local costmap window (meters)
      height: 6.0  # Height of local costmap window (meters)
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
```

## Configuring Costmap Layers

### 1. Static Layer Configuration

#### Static Layer for Map Integration
```yaml
# static_layer_config.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        # Subscribe to map topic with transient local durability
        map_subscribe_transient_local: true
        # Transform timeout for map to robot transforms
        transform_tolerance: 0.5
        # Whether to publish the static layer separately
        enabled: true
        # Topic to subscribe to for the static map
        map_topic: map
        # Whether to subscribe with transient local durability
        subscribe_to_updates: false
```

### 2. Obstacle Layer Configuration

#### 2D Obstacle Layer for LiDAR Data
```yaml
# obstacle_layer_config.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        # How often to clear obstacles
        clearing: true
        # How often to mark obstacles
        marking: true
        # Observation sources (sensors) to use
        observation_sources: scan
        # Configuration for the LiDAR sensor
        scan:
          # Topic name
          topic: /scan
          # Maximum height of obstacles to consider
          max_obstacle_height: 2.0
          # Minimum height of obstacles to consider
          min_obstacle_height: 0.0
          # Whether to clear obstacles
          clearing: true
          # Whether to mark obstacles
          marking: true
          # Data type (LaserScan, PointCloud2, etc.)
          data_type: "LaserScan"
          # Maximum range for raytracing (clearing)
          raytrace_max_range: 5.0
          # Minimum range for raytracing
          raytrace_min_range: 0.0
          # Maximum range for obstacle detection
          obstacle_max_range: 4.0
          # Minimum range for obstacle detection
          obstacle_min_range: 0.0
          # Transform tolerance
          transform_tolerance: 0.2
```

#### 3D Obstacle Layer (Voxel Layer) for Depth Sensors
```yaml
# voxel_layer_config.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        # Publish voxel map for visualization
        publish_voxel_map: true
        # Origin Z for voxel grid
        origin_z: 0.0
        # Resolution in Z direction
        z_resolution: 0.2
        # Number of Z voxels
        z_voxels: 10
        # Maximum obstacle height
        max_obstacle_height: 2.0
        # Threshold for marking a cell as obstacle
        mark_threshold: 0
        # Observation sources
        observation_sources: pointcloud
        # Point cloud configuration
        pointcloud:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          # Whether to clear obstacles
          clearing: true
          # Whether to mark obstacles
          marking: true
          # Data type
          data_type: "PointCloud2"
          # Obstacle range
          obstacle_range: 3.0
          # Raytrace range
          raytrace_range: 4.0
          # Transform tolerance
          transform_tolerance: 0.2
```

### 3. Inflation Layer Configuration

#### Inflation Layer for Safety Buffer
```yaml
# inflation_layer_config.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        # Factor to scale cost inflation
        cost_scaling_factor: 5.0
        # Radius to which obstacles are inflated
        inflation_radius: 0.55

local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        # Higher scaling factor for local planning safety
        cost_scaling_factor: 10.0
        # Larger inflation for humanoid safety
        inflation_radius: 0.8
```

## Complete Costmap Configuration for Humanoid Robots

### 1. Global Costmap for Humanoid Navigation
```yaml
# humanoid_global_costmap_params.yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      robot_radius: 0.4  # Larger for humanoid safety
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 8.0  # Higher for humanoid safety
        inflation_radius: 0.8     # Larger safety buffer
```

### 2. Local Costmap for Humanoid Navigation
```yaml
# humanoid_local_costmap_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 8.0   # Larger for humanoid safety
      height: 8.0
      resolution: 0.05
      robot_radius: 0.4  # Larger for humanoid safety
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          clearing: true
          marking: true
          data_type: "PointCloud2"
          obstacle_range: 3.0
          raytrace_range: 4.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 10.0  # Higher for safety
        inflation_radius: 0.8     # Larger safety buffer
```

## Recovery Behaviors Configuration

### 1. Understanding Recovery Behaviors

#### What are Recovery Behaviors?
Recovery behaviors are actions taken by the navigation system when it detects that the robot is stuck, has failed to make progress, or has encountered an obstacle that prevents navigation. These behaviors help the robot recover from difficult situations.

#### Common Recovery Behaviors
- **Spin**: Rotate in place to clear local minima
- **Back Up**: Move backward to get out of tight spaces
- **Wait**: Pause navigation to allow dynamic obstacles to clear

### 2. Recovery Server Configuration

#### Basic Recovery Configuration
```yaml
# recovery_server_params.yaml
recoveries_server:
  ros__parameters:
    # Topic for costmap
    costmap_topic: local_costmap/costmap_raw
    # Topic for footprint
    footprint_topic: local_costmap/published_footprint
    # Cycle frequency for checking recovery conditions
    cycle_frequency: 10.0
    # Recovery plugins to load
    recovery_plugins: ["spin", "backup", "wait"]
    # Types of recovery plugins
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait"]

    # Spin recovery configuration
    spin:
      plugin: "nav2_recoveries::Spin"
      # Simulation frequency
      sim_frequency: 20.0
      # Forward speed (should be 0 for spin)
      forward_speed: 0.0
      # Rotate speed (radians per second)
      rotate_speed: 1.0
      # Maximum rotational displacement (radians)
      max_rotational_displacement: 1.57  # 90 degrees

    # Back Up recovery configuration
    backup:
      plugin: "nav2_recoveries::BackUp"
      sim_frequency: 20.0
      # Forward speed (negative for backward)
      forward_speed: -0.065
      # Distance to backup (meters)
      backup_dist: 0.15
      # Command velocity topic
      cmd_topic: /cmd_vel

    # Wait recovery configuration
    wait:
      plugin: "nav2_recoveries::Wait"
      sim_frequency: 20.0
      # Duration to wait (seconds)
      wait_duration: 1.0
```

### 3. Humanoid-Specific Recovery Behaviors

#### Custom Recovery for Humanoid Robots
```python
# humanoid_recovery_behaviors.py
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import time

class HumanoidRecoveryBehaviors(Node):
    def __init__(self):
        super().__init__('humanoid_recovery_behaviors')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.recovery_status_pub = self.create_publisher(Bool, '/recovery_active', 10)

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Internal state
        self.current_pose = None
        self.recovery_active = False

        self.get_logger().info('Humanoid Recovery Behaviors initialized')

    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose

    def step_back_recovery(self):
        """Humanoid-specific recovery: Step back carefully"""
        self.get_logger().info('Executing step back recovery')

        # Indicate recovery is active
        status_msg = Bool()
        status_msg.data = True
        self.recovery_status_pub.publish(status_msg)
        self.recovery_active = True

        # Create backward movement command
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.1  # Slow backward movement
        cmd_vel.angular.z = 0.0

        # Execute for a short duration (for humanoid stepping)
        start_time = time.time()
        duration = 2.0  # seconds

        while time.time() - start_time < duration and rclpy.ok():
            if self.recovery_active:
                self.cmd_vel_pub.publish(cmd_vel)
                time.sleep(0.1)  # 10 Hz

        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Reset recovery status
        status_msg.data = False
        self.recovery_status_pub.publish(status_msg)
        self.recovery_active = False

        self.get_logger().info('Step back recovery completed')

    def rotate_in_place_recovery(self):
        """Humanoid-specific recovery: Careful rotation"""
        self.get_logger().info('Executing rotate in place recovery')

        # Indicate recovery is active
        status_msg = Bool()
        status_msg.data = True
        self.recovery_status_pub.publish(status_msg)
        self.recovery_active = True

        # Rotate slowly to clear obstacles
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.2  # Slow rotation

        # Execute rotation
        start_time = time.time()
        duration = 3.0  # seconds for 90-degree turn at 0.2 rad/s

        while time.time() - start_time < duration and rclpy.ok():
            if self.recovery_active:
                self.cmd_vel_pub.publish(cmd_vel)
                time.sleep(0.1)

        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Reset recovery status
        status_msg.data = False
        self.recovery_status_pub.publish(status_msg)
        self.recovery_active = False

        self.get_logger().info('Rotate in place recovery completed')

    def wait_and_assess_recovery(self):
        """Humanoid-specific recovery: Wait and reassess"""
        self.get_logger().info('Executing wait and assess recovery')

        # Indicate recovery is active
        status_msg = Bool()
        status_msg.data = True
        self.recovery_status_pub.publish(status_msg)
        self.recovery_active = True

        # Wait for a period to allow dynamic obstacles to clear
        wait_duration = 5.0  # seconds
        start_time = time.time()

        while time.time() - start_time < wait_duration and rclpy.ok():
            # Publish zero velocity to stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            time.sleep(0.1)

        # Reset recovery status
        status_msg.data = False
        self.recovery_status_pub.publish(status_msg)
        self.recovery_active = False

        self.get_logger().info('Wait and assess recovery completed')

def main(args=None):
    rclpy.init(args=args)
    recovery_node = HumanoidRecoveryBehaviors()

    # The recovery behaviors would be called by the navigation system
    # when recovery is needed, not run continuously
    recovery_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Advanced Recovery Configuration

#### Custom Recovery Server Configuration
```yaml
# advanced_recovery_params.yaml
bt_navigator:
  ros__parameters:
    # Behavior tree configuration for recovery
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

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait", "humanoid_step_back"]
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait", "humanoid_nav2::StepBackRecovery"]

    spin:
      plugin: "nav2_recoveries::Spin"
      sim_frequency: 10.0  # Slower for humanoid stability
      forward_speed: 0.0
      rotate_speed: 0.5    # Slower rotation for humanoid
      max_rotational_displacement: 1.57

    backup:
      plugin: "nav2_recoveries::BackUp"
      sim_frequency: 10.0
      forward_speed: -0.1  # Slower backward movement
      backup_dist: 0.3     # Longer backup distance for humanoid
      cmd_topic: /cmd_vel

    wait:
      plugin: "nav2_recoveries::Wait"
      sim_frequency: 20.0
      wait_duration: 5.0   # Longer wait for humanoid navigation

    humanoid_step_back:
      plugin: "humanoid_nav2::StepBackRecovery"
      sim_frequency: 20.0
      step_size: 0.3       # Step size for humanoid
      max_steps: 3         # Maximum steps to take back
      step_delay: 0.5      # Delay between steps
```

## Costmap Visualization and Debugging

### 1. Costmap Visualization Tools

#### Costmap Visualization Configuration
```yaml
# costmap_viz_params.yaml
costmap_visualizer:
  ros__parameters:
    # Update rate for visualization
    update_frequency: 10.0
    # Topic for global costmap visualization
    global_costmap_topic: /global_costmap/costmap
    # Topic for local costmap visualization
    local_costmap_topic: /local_costmap/costmap
    # Frame for visualization
    visualization_frame: map
    # Whether to publish markers
    publish_markers: true
    # Marker lifetime (seconds)
    marker_lifetime: 1.0
```

### 2. Costmap Analysis Node

#### Costmap Analysis and Statistics
```python
# costmap_analyzer.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Point
import numpy as np

class CostmapAnalyzer(Node):
    def __init__(self):
        super().__init__('costmap_analyzer')

        # Subscriptions
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.global_costmap_callback, 10)
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, 10)

        # Publishers for analysis results
        self.obstacle_density_pub = self.create_publisher(Float32, '/costmap/obstacle_density', 10)
        self.free_space_pub = self.create_publisher(Float32, '/costmap/free_space_ratio', 10)
        self.unknown_space_pub = self.create_publisher(Float32, '/costmap/unknown_space_ratio', 10)
        self.obstacle_count_pub = self.create_publisher(Int32, '/costmap/obstacle_count', 10)

        self.get_logger().info('Costmap Analyzer initialized')

    def global_costmap_callback(self, msg):
        """Analyze global costmap data"""
        self.analyze_costmap(msg, 'global')

    def local_costmap_callback(self, msg):
        """Analyze local costmap data"""
        self.analyze_costmap(msg, 'local')

    def analyze_costmap(self, costmap_msg, costmap_type):
        """Analyze costmap and publish statistics"""
        # Convert costmap data to numpy array
        data = np.array(costmap_msg.data).reshape((costmap_msg.info.height, costmap_msg.info.width))

        # Calculate statistics
        total_cells = data.size
        obstacle_cells = np.sum(data > 50)  # Consider values > 50 as obstacles
        free_cells = np.sum(data == 0)      # Consider value 0 as free space
        unknown_cells = np.sum(data == -1)  # Consider value -1 as unknown

        # Calculate ratios
        obstacle_density = obstacle_cells / total_cells if total_cells > 0 else 0
        free_space_ratio = free_cells / total_cells if total_cells > 0 else 0
        unknown_space_ratio = unknown_cells / total_cells if total_cells > 0 else 0

        # Publish statistics
        obstacle_density_msg = Float32()
        obstacle_density_msg.data = float(obstacle_density)
        self.obstacle_density_pub.publish(obstacle_density_msg)

        free_space_msg = Float32()
        free_space_msg.data = float(free_space_ratio)
        self.free_space_pub.publish(free_space_msg)

        unknown_space_msg = Float32()
        unknown_space_msg.data = float(unknown_space_ratio)
        self.unknown_space_pub.publish(unknown_space_msg)

        obstacle_count_msg = Int32()
        obstacle_count_msg.data = int(obstacle_cells)
        self.obstacle_count_pub.publish(obstacle_count_msg)

        # Log statistics
        self.get_logger().info(
            f'{costmap_type.capitalize()} Costmap - '
            f'Obstacle Density: {obstacle_density:.3f}, '
            f'Free Space: {free_space_ratio:.3f}, '
            f'Unknown: {unknown_space_ratio:.3f}, '
            f'Obstacles: {obstacle_cells}'
        )

def main(args=None):
    rclpy.init(args=args)
    analyzer = CostmapAnalyzer()
    rclpy.spin(analyzer)
    analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Implementation Tips

### 1. Parameter Tuning Guidelines

#### Costmap Parameter Tuning
- **Resolution**: Start with 0.05m, adjust based on robot size and sensor accuracy
- **Robot Radius**: Set slightly larger than actual robot for safety
- **Inflation Radius**: 0.5-1.0m for safety, larger for humanoid robots
- **Update Frequency**: 1Hz for global, 5-10Hz for local costmaps

#### Recovery Behavior Tuning
- **Spin**: Use slower rotation speeds for humanoid stability
- **Back Up**: Use shorter distances and slower speeds
- **Wait**: Use longer durations to allow dynamic obstacles to clear

### 2. Common Configuration Issues and Solutions

#### Issue: Costmap Not Updating
**Solution**: Check TF tree, ensure proper frame relationships between map, odom, and base_link

#### Issue: Robot Too Conservative
**Solution**: Reduce inflation radius, lower cost scaling factor, adjust robot radius

#### Issue: Robot Gets Stuck Frequently
**Solution**: Increase inflation radius, tune recovery behaviors, adjust local costmap size

### 3. Performance Optimization

#### Costmap Optimization Strategies
- Use appropriate resolution for your robot and environment
- Limit costmap size for local costmap to reduce computation
- Use rolling window for local costmap to maintain fixed size
- Optimize sensor data rates to match costmap update rates

This comprehensive tutorial provides the knowledge and tools needed to properly configure costmaps, obstacle layers, and recovery behaviors in Nav2 for both general and humanoid robot applications.