# Configuring Nav2 Stack for Bipedal Humanoid Navigation

## Overview

This tutorial provides comprehensive instructions for configuring the Navigation2 (Nav2) stack specifically for bipedal humanoid robots. Unlike wheeled robots, humanoid robots have unique kinematic and dynamic constraints that require specialized configuration of the navigation stack.

## Understanding Humanoid Navigation Constraints

### Bipedal Locomotion Challenges

#### Kinematic Constraints
- **Discrete footstep planning**: Navigation must be planned in discrete steps rather than continuous paths
- **Balance maintenance**: Center of mass must remain within support polygon during motion
- **Limited turning radius**: Turning requires stepping, not instantaneous rotation
- **Step height limitations**: Maximum step height constrains terrain traversal

#### Dynamic Constraints
- **Stability requirements**: Each step must maintain balance before executing the next
- **Inertia considerations**: Higher moment of inertia compared to wheeled robots
- **Actuator limitations**: Joint torque and velocity constraints
- **Power consumption**: Battery life considerations during navigation

### Navigation Requirements for Humanoids
- **Safe path planning**: Paths must account for balance and step feasibility
- **Dynamic obstacle avoidance**: Real-time adjustment for moving obstacles
- **Terrain assessment**: Evaluate terrain traversability for bipedal locomotion
- **Recovery behaviors**: Specialized recovery for humanoid-specific failures

## Nav2 Architecture for Humanoids

### Core Components Overview
```
[Global Planner] → [Local Planner] → [Footstep Planner] → [Biped Controller] → [Humanoid]
      ↑                 ↑                  ↑                   ↑
[Costmap Global]  [Costmap Local]  [Step Costmap]    [Sensors/IMU]
```

### Humanoid-Specific Modifications

#### 1. Global Planner Considerations
- **Discrete path planning**: Generate paths suitable for footstep planning
- **Step feasibility**: Ensure path can be traversed with discrete steps
- **Balance constraints**: Consider balance during path generation

#### 2. Local Planner Adaptations
- **Step-by-step execution**: Execute navigation in discrete steps
- **Balance-aware control**: Adjust velocities for stability
- **Reactive obstacle avoidance**: Handle obstacles with step planning

#### 3. Specialized Nodes
- **Footstep Planner**: Plan discrete footsteps for bipedal locomotion
- **Balance Controller**: Maintain balance during navigation
- **Step State Machine**: Coordinate discrete step execution

## Installation and Setup

### Install Nav2 Packages
```bash
# Install Nav2 base packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install additional dependencies for humanoid navigation
sudo apt install ros-humble-dwb-core ros-humble-dwb-msgs ros-humble-dwb-plugins
sudo apt install ros-humble-robot-localization ros-humble-robot-state-publisher
```

### Create Humanoid-Specific Package
```bash
# Create workspace for humanoid navigation
mkdir -p ~/humanoid_nav_ws/src
cd ~/humanoid_nav_ws/src

# Create package for humanoid navigation configurations
ros2 pkg create --build-type ament_cmake humanoid_nav2_config --dependencies rclcpp
```

## Configuration Files for Humanoid Navigation

### 1. Main Navigation Parameters

#### Main Nav2 Parameters File
```yaml
# humanoid_nav2_params.yaml
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
      xy_goal_tolerance: 0.5  # Increased for humanoid
      yaw_goal_tolerance: 0.5
      stateful: True
    navigate_to_pose_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.5  # Increased for humanoid
      yaw_goal_tolerance: 0.5
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

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

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

    # Humanoid-specific controller configuration
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.05  # Minimum forward speed for stability
      min_vel_y: 0.0
      max_vel_x: 0.3   # Reduced for stability
      max_vel_y: 0.0
      max_vel_theta: 0.3  # Reduced for stability
      min_speed_xy: 0.05
      max_speed_xy: 0.3
      min_speed_theta: 0.05
      acc_lim_x: 0.5    # Lower acceleration for balance
      acc_lim_y: 0.0
      acc_lim_theta: 0.5
      decel_lim_x: -0.5 # Lower deceleration for balance
      decel_lim_y: 0.0
      decel_lim_theta: -0.5
      vx_samples: 10    # Fewer samples for humanoid
      vy_samples: 1
      vtheta_samples: 20
      sim_time: 2.0     # Longer simulation time
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.5  # Larger for humanoid
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
      width: 8    # Larger for humanoid safety
      height: 8
      resolution: 0.05
      robot_radius: 0.5  # Larger for humanoid safety
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0  # Higher for humanoid safety
        inflation_radius: 1.0     # Larger safety buffer
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 16  # More layers for humanoid
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0  # Longer range for humanoid
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0  # Longer range for humanoid
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
      robot_radius: 0.5  # Larger for humanoid
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
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 1.0
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
    yaml_filename: "humanoid_map.yaml"

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0  # Lower for humanoid
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5  # Larger tolerance for humanoid
      use_astar: true
      allow_unknown: true

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait", "humanoid_step_back"]
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait", "humanoid_nav2::StepBackRecovery"]
    spin:
      plugin: "nav2_recoveries::Spin"
      sim_frequency: 5.0  # Slower for humanoid
      forward_speed: 0.0
      rotate_speed: 0.2   # Slower rotation for stability
      max_rotational_displacement: 1.57
    backup:
      plugin: "nav2_recoveries::BackUp"
      sim_frequency: 5.0
      forward_speed: -0.1 # Slower backward movement
      backup_dist: 0.5    # Longer backup distance
      cmd_topic: /cmd_vel
    wait:
      plugin: "nav2_recoveries::Wait"
      sim_frequency: 20.0
      wait_duration: 5.0  # Longer wait for humanoid
```

### 2. Humanoid-Specific Costmap Configuration

#### Step Costmap Parameters
```yaml
# step_costmap_params.yaml
step_costmap:
  ros__parameters:
    update_frequency: 10.0
    publish_frequency: 5.0
    global_frame: map
    robot_base_frame: base_link
    use_sim_time: True
    rolling_window: false
    width: 20.0
    height: 20.0
    resolution: 0.1
    track_unknown_space: false

    plugins: ["footprint_layer", "step_height_layer", "slope_layer"]

    footprint_layer:
      plugin: "nav2_costmap_2d::FootprintLayer"
      enabled: true
      footprint_topic: "local_costmap/published_footprint"
      robot_radius: 0.3

    step_height_layer:
      plugin: "humanoid_nav2::StepHeightLayer"
      enabled: true
      max_step_height: 0.2  # Maximum step height for humanoid
      min_step_height: -0.1 # Minimum step down height

    slope_layer:
      plugin: "humanoid_nav2::SlopeLayer"
      enabled: true
      max_slope: 0.3  # Maximum traversable slope (30 degrees)
      robot_width: 0.6
```

## Launch Files for Humanoid Navigation

### 1. Main Launch File

#### Complete Humanoid Nav2 Launch
```python
# humanoid_nav2_launch.py
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
    params_file = LaunchConfiguration('params_file', default='humanoid_nav2_params.yaml')

    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    humanoid_nav_dir = get_package_share_directory('humanoid_nav2_config')

    # Include main Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
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
                humanoid_nav_dir,
                'config',
                params_file
            ])
        }.items()
    )

    # Humanoid-specific nodes container
    humanoid_container = ComposableNodeContainer(
        name='humanoid_nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Footstep planner
            ComposableNode(
                package='humanoid_nav2',
                plugin='humanoid_nav2::FootstepPlanner',
                name='footstep_planner',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'step_width': 0.2,      # Distance between feet
                    'step_length': 0.3,     # Step length
                    'step_height': 0.05,    # Maximum step height
                    'max_turn_angle': 0.5   # Maximum turn per step
                }]
            ),
            # Balance controller
            ComposableNode(
                package='humanoid_nav2',
                plugin='humanoid_nav2::BalanceController',
                name='balance_controller',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'com_height': 0.8,      # Center of mass height
                    'control_rate': 100.0,  # Balance control rate
                    'stability_threshold': 0.1
                }]
            )
        ],
        output='screen'
    )

    # RViz2 configuration
    rviz_config = PathJoinSubstitution([
        humanoid_nav_dir,
        'rviz',
        'humanoid_nav2.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        nav2_bringup_launch,
        humanoid_container,
        rviz_node
    ])
```

### 2. Behavior Tree Configuration

#### Humanoid-Specific Behavior Tree
```xml
<!-- humanoid_nav_tree.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithRecovery">
            <RateController hz="1.0">
                <RecoveryNode number_of_retries="6" name="NavigateRecovery">
                    <PipelineSequence name="Navigate">
                        <GoalUpdated/>
                        <ComputePathToPose goal="current_goal" path="path"/>
                        <FollowPath path="path" velocity="cmd_vel"/>
                    </PipelineSequence>
                    <ReactiveFallback name="RecoveryFallback">
                        <Spin spin_dist="1.57"/>
                        <BackUp backup_dist="0.30" backup_speed="0.05"/>
                        <Wait wait_duration="5"/>
                    </ReactiveFallback>
                </RecoveryNode>
            </RateController>
        </PipelineSequence>
    </BehaviorTree>
</root>
```

## Humanoid-Specific Navigation Nodes

### 1. Footstep Planner Implementation

#### Footstep Planner Node
```python
# footstep_planner.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial.transform import Rotation as R

class FootstepPlanner(Node):
    def __init__(self):
        super().__init__('footstep_planner')

        # Parameters
        self.declare_parameter('step_width', 0.2)
        self.declare_parameter('step_length', 0.3)
        self.declare_parameter('step_height', 0.05)
        self.declare_parameter('max_turn_angle', 0.5)

        self.step_width = self.get_parameter('step_width').get_parameter_value().double_value
        self.step_length = self.get_parameter('step_length').get_parameter_value().double_value
        self.step_height = self.get_parameter('step_height').get_parameter_value().double_value
        self.max_turn_angle = self.get_parameter('max_turn_angle').get_parameter_value().double_value

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.footstep_pub = self.create_publisher(Path, '/footsteps', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.current_pose = None
        self.balance_ok = True

        self.get_logger().info('Footstep Planner initialized')

    def path_callback(self, msg):
        """Convert global path to discrete footsteps for humanoid"""
        if not self.current_pose:
            return

        # Convert continuous path to discrete footsteps
        footsteps = self.convert_path_to_footsteps(msg.poses)

        # Publish footsteps
        footstep_msg = Path()
        footstep_msg.header = msg.header
        footstep_msg.poses = footsteps
        self.footstep_pub.publish(footstep_msg)

        # Execute footsteps if balance is OK
        if self.balance_ok:
            self.execute_footsteps(footsteps)

    def convert_path_to_footsteps(self, path_poses):
        """Convert continuous path to discrete footsteps"""
        footsteps = []

        if len(path_poses) < 2:
            return footsteps

        # Start with current pose
        footsteps.append(self.current_pose)

        # Calculate required steps based on step length and turn constraints
        for i in range(1, len(path_poses)):
            current_step = footsteps[-1]
            target_pose = path_poses[i]

            # Calculate required movement
            dx = target_pose.pose.position.x - current_step.pose.position.x
            dy = target_pose.pose.position.y - current_step.pose.position.y
            distance = np.sqrt(dx*dx + dy*dy)

            # Check if movement is within step constraints
            if distance <= self.step_length:
                # Add the target pose as a step
                footsteps.append(target_pose)
            else:
                # Break into multiple steps
                num_steps = int(np.ceil(distance / self.step_length))
                step_dx = dx / num_steps
                step_dy = dy / num_steps

                for step in range(1, num_steps + 1):
                    step_pose = PoseStamped()
                    step_pose.header = target_pose.header
                    step_pose.pose.position.x = current_step.pose.position.x + step * step_dx
                    step_pose.pose.position.y = current_step.pose.position.y + step * step_dy
                    step_pose.pose.position.z = current_step.pose.position.z

                    # Interpolate orientation
                    q_start = [current_step.pose.orientation.x, current_step.pose.orientation.y,
                              current_step.pose.orientation.z, current_step.pose.orientation.w]
                    q_end = [target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                            target_pose.pose.orientation.z, target_pose.pose.orientation.w]

                    # Slerp for quaternion interpolation
                    step_pose.pose.orientation = self.slerp_quaternions(q_start, q_end, step/num_steps)

                    footsteps.append(step_pose)

        return footsteps

    def slerp_quaternions(self, q1, q2, t):
        """Spherical linear interpolation between quaternions"""
        # Convert to numpy arrays
        q1 = np.array(q1)
        q2 = np.array(q2)

        # Calculate dot product
        dot = np.dot(q1, q2)

        # If dot product is negative, negate one quaternion
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        # Calculate interpolation
        if dot > 0.9995:
            # Linear interpolation for very similar quaternions
            result = q1 + t * (q2 - q1)
        else:
            # Spherical linear interpolation
            theta_0 = np.arccos(dot)
            sin_theta_0 = np.sin(theta_0)
            theta = theta_0 * t
            sin_theta = np.sin(theta)

            s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
            s1 = sin_theta / sin_theta_0

            result = s0 * q1 + s1 * q2

        # Normalize result
        result = result / np.linalg.norm(result)

        from geometry_msgs.msg import Quaternion
        q_msg = Quaternion()
        q_msg.x = result[0]
        q_msg.y = result[1]
        q_msg.z = result[2]
        q_msg.w = result[3]

        return q_msg

    def execute_footsteps(self, footsteps):
        """Execute the planned footsteps"""
        if len(footsteps) < 2:
            return

        # Calculate command to reach next step
        next_step = footsteps[1]
        cmd_vel = Twist()

        # Calculate required movement
        dx = next_step.pose.position.x - self.current_pose.pose.position.x
        dy = next_step.pose.position.y - self.current_pose.pose.position.y
        distance = np.sqrt(dx*dx + dy*dy)

        # Calculate required rotation
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
        target_yaw = self.get_yaw_from_quaternion(next_step.pose.orientation)
        yaw_diff = target_yaw - current_yaw

        # Limit movement to step constraints
        if distance > self.step_length:
            dx = dx * (self.step_length / distance)
            dy = dy * (self.step_length / distance)
            distance = self.step_length

        # Set velocity commands
        cmd_vel.linear.x = min(0.1, distance * 2.0)  # Scale with distance
        cmd_vel.angular.z = max(-self.max_turn_angle, min(self.max_turn_angle, yaw_diff * 2.0))

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

    def get_yaw_from_quaternion(self, q):
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        # Check for obstacles in path
        min_distance = min([r for r in msg.ranges if not (np.isnan(r) or np.isinf(r))], default=float('inf'))

        if min_distance < 0.5:  # Safety distance
            self.balance_ok = False
            # Stop movement
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
        else:
            self.balance_ok = True

    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

def main(args=None):
    rclpy.init(args=args)
    planner = FootstepPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Footstep planner stopped')
    finally:
        # Stop robot on shutdown
        stop_cmd = Twist()
        planner.cmd_vel_pub.publish(stop_cmd)
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing and Validation

### 1. Simulation Testing

#### Gazebo/Isaac Sim Integration
```python
# humanoid_nav_test_launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch Gazebo simulation with humanoid robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        )
    )

    # Launch humanoid navigation
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('humanoid_nav2_config'),
                'launch',
                'humanoid_nav2_launch.py'
            ])
        )
    )

    return LaunchDescription([
        gazebo_launch,
        nav_launch
    ])
```

### 2. Performance Validation

#### Navigation Performance Metrics
```python
# nav_performance_evaluator.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32
import time
import numpy as np

class NavigationPerformanceEvaluator(Node):
    def __init__(self):
        super().__init__('nav_performance_evaluator')

        # Subscriptions
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)

        # Publishers
        self.success_pub = self.create_publisher(Float32, '/nav_success_rate', 10)
        self.efficiency_pub = self.create_publisher(Float32, '/nav_efficiency', 10)
        self.safety_pub = self.create_publisher(Float32, '/nav_safety', 10)

        # Variables
        self.start_time = None
        self.start_pos = None
        self.goal_pos = None
        self.current_pos = None
        self.path_length = 0.0
        self.executed_path = []

        self.get_logger().info('Navigation Performance Evaluator initialized')

    def goal_callback(self, msg):
        """Record goal position and start timing"""
        self.goal_pos = (msg.pose.position.x, msg.pose.position.y)
        self.start_time = time.time()
        if self.current_pos:
            self.start_pos = self.current_pos

    def odom_callback(self, msg):
        """Track robot position"""
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if self.start_pos and self.goal_pos:
            # Add to executed path
            self.executed_path.append(self.current_pos)

    def path_callback(self, msg):
        """Calculate optimal path length"""
        self.path_length = self.calculate_path_length(msg.poses)

    def calculate_path_length(self, poses):
        """Calculate total path length"""
        if len(poses) < 2:
            return 0.0

        length = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i-1].pose.position.x
            dy = poses[i].pose.position.y - poses[i-1].pose.position.y
            length += np.sqrt(dx*dx + dy*dy)

        return length

    def evaluate_performance(self):
        """Calculate and publish performance metrics"""
        if not (self.start_pos and self.goal_pos and self.current_pos):
            return

        # Calculate direct distance
        direct_distance = np.sqrt(
            (self.goal_pos[0] - self.start_pos[0])**2 +
            (self.goal_pos[1] - self.start_pos[1])**2
        )

        # Calculate actual distance traveled
        actual_distance = sum([
            np.sqrt((self.executed_path[i][0] - self.executed_path[i-1][0])**2 +
                   (self.executed_path[i][1] - self.executed_path[i-1][1])**2)
            for i in range(1, len(self.executed_path))
        ])

        # Calculate efficiency
        efficiency = direct_distance / actual_distance if actual_distance > 0 else 0

        # Calculate success (if within goal tolerance)
        goal_distance = np.sqrt(
            (self.current_pos[0] - self.goal_pos[0])**2 +
            (self.current_pos[1] - self.goal_pos[1])**2
        )
        success = 1.0 if goal_distance < 0.5 else 0.0  # 50cm tolerance

        # Publish metrics
        eff_msg = Float32()
        eff_msg.data = float(efficiency)
        self.efficiency_pub.publish(eff_msg)

        success_msg = Float32()
        success_msg.data = float(success)
        self.success_pub.publish(success_msg)

        self.get_logger().info(
            f'Navigation - Success: {success}, '
            f'Efficiency: {efficiency:.2f}, '
            f'Direct: {direct_distance:.2f}m, '
            f'Actual: {actual_distance:.2f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    evaluator = NavigationPerformanceEvaluator()

    # Timer for periodic evaluation
    timer = evaluator.create_timer(2.0, evaluator.evaluate_performance)

    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        evaluator.get_logger().info('Performance evaluator stopped')
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety Considerations

### 1. Humanoid-Specific Safety Features

#### Balance Monitoring
```python
# balance_monitor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class BalanceMonitor(Node):
    def __init__(self):
        super().__init__('balance_monitor')

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publishers
        self.safety_pub = self.create_publisher(Bool, '/balance_safe', 10)
        self.emergency_stop_pub = self.create_publisher(Twist, '/cmd_vel_emergency', 10)

        # Parameters
        self.balance_threshold = 0.3  # Radians
        self.safety_engaged = False

        self.get_logger().info('Balance monitor initialized')

    def imu_callback(self, msg):
        """Monitor robot balance from IMU data"""
        # Extract roll and pitch from quaternion
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = np.arcsin(sinp)

        # Check if balance is within safe limits
        if abs(roll) > self.balance_threshold or abs(pitch) > self.balance_threshold:
            self.safety_engaged = True
            self.get_logger().error(f'BALANCE COMPROMISED - Roll: {roll:.3f}, Pitch: {pitch:.3f}')

            # Emergency stop
            stop_cmd = Twist()
            self.emergency_stop_pub.publish(stop_cmd)
        else:
            self.safety_engaged = False

        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = not self.safety_engaged
        self.safety_pub.publish(safety_msg)

def main(args=None):
    rclpy.init(args=args)
    monitor = BalanceMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting Humanoid Navigation

### 1. Common Issues and Solutions

#### Issue: Oscillation During Navigation
**Cause**: Aggressive control parameters for humanoid dynamics
**Solution**:
- Reduce maximum velocities and accelerations
- Increase controller frequency for better stability
- Implement smoother trajectory generation

#### Issue: Failure to Navigate Around Obstacles
**Cause**: Conservative safety margins or costmap issues
**Solution**:
- Adjust inflation radius and cost scaling factors
- Verify sensor data quality and range
- Check TF transforms between sensors and base frame

#### Issue: Balance Loss During Navigation
**Cause**: Navigation commands too aggressive for bipedal stability
**Solution**:
- Implement balance-aware velocity limiting
- Use footstep planning for discrete navigation
- Add balance feedback control loop

### 2. Performance Optimization

#### Tuning Guidelines
- Start with conservative parameters and gradually increase
- Test in controlled environments before complex scenarios
- Monitor balance and stability metrics continuously
- Use simulation extensively before real-world testing

This comprehensive configuration guide provides the foundation for implementing Nav2 navigation specifically designed for bipedal humanoid robots, addressing their unique kinematic and dynamic constraints.