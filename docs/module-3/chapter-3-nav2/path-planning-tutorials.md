# Global and Local Path Planning Tutorials

## Overview

This tutorial provides comprehensive guidance on implementing global and local path planning for robotics applications using the Navigation2 (Nav2) stack. The tutorial covers both theoretical concepts and practical implementation, with a focus on humanoid robot navigation requirements.

## Understanding Path Planning

### Global vs Local Path Planning

#### Global Path Planning
- **Purpose**: Generate optimal path from start to goal considering static obstacles
- **Input**: Map, start pose, goal pose
- **Output**: Waypoints forming complete path
- **Frequency**: Lower frequency (e.g., once per navigation request)
- **Considerations**: Optimality, completeness, computational efficiency

#### Local Path Planning
- **Purpose**: Execute path while avoiding dynamic obstacles and adjusting for real-time conditions
- **Input**: Global path, local costmap, robot state
- **Output**: Velocity commands for robot motion
- **Frequency**: Higher frequency (e.g., 10-20 Hz)
- **Considerations**: Safety, real-time performance, obstacle avoidance

### Path Planning for Humanoid Robots

#### Unique Challenges
- **Discrete motion**: Bipedal locomotion requires discrete steps
- **Balance constraints**: Path must maintain center of mass stability
- **Terrain limitations**: Step height and slope constraints
- **Turning mechanics**: Turning requires multiple steps, not instantaneous rotation

## Global Path Planning

### 1. Navigation2 Global Planners

#### Available Planners in Nav2
- **NavFn**: Fast-marching method, good for global planning
- **Global Planner**: A* and Dijkstra implementations
- **Smac Planner**: Sampling-based motion planning for SE2 state lattice
- **Theta*: Curvature-constrained path planning

#### Configuration for Humanoid Navigation
```yaml
# global_planner_params.yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0  # Lower frequency for humanoid
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5      # Larger tolerance for humanoid
      use_astar: true     # Use A* for better path quality
      allow_unknown: true # Plan through unknown areas if needed
```

### 2. Custom Global Planner Implementation

#### Creating a Humanoid-Aware Global Planner
```python
# humanoid_global_planner.py
import rclpy
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from builtin_interfaces.msg import Duration
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile
import numpy as np
from scipy.spatial import distance
import math

class HumanoidGlobalPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_global_planner')

        # Action server for path computation
        self._action_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            self.execute_callback)

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Internal variables
        self.map_data = None
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_origin = None

        self.get_logger().info('Humanoid Global Planner initialized')

    def map_callback(self, msg):
        """Store map data for path planning"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = msg.info.origin

    def execute_callback(self, goal_handle):
        """Execute path planning request"""
        self.get_logger().info('Received path planning request')

        # Extract start and goal poses
        start_pose = goal_handle.request.start
        goal_pose = goal_handle.request.pose

        # Check if map is available
        if self.map_data is None:
            self.get_logger().error('Map not available for path planning')
            result = ComputePathToPose.Result()
            result.path = Path()
            goal_handle.abort()
            return result

        # Plan path considering humanoid constraints
        path = self.plan_path_with_constraints(start_pose, goal_pose)

        # Create result
        result = ComputePathToPose.Result()
        result.path = path

        # Check if path was found
        if len(path.poses) > 0:
            goal_handle.succeed()
            self.get_logger().info('Path planning succeeded')
        else:
            goal_handle.abort()
            self.get_logger().error('Path planning failed')

        return result

    def plan_path_with_constraints(self, start_pose, goal_pose):
        """Plan path considering humanoid-specific constraints"""
        # Convert poses to map coordinates
        start_map = self.world_to_map(start_pose.pose.position)
        goal_map = self.world_to_map(goal_pose.pose.position)

        # Validate coordinates
        if not self.is_valid_coordinate(start_map) or not self.is_valid_coordinate(goal_map):
            self.get_logger().error('Invalid start or goal coordinates')
            return Path()

        # Check if start and goal are in free space
        if self.is_occupied(start_map) or self.is_occupied(goal_map):
            self.get_logger().error('Start or goal pose in occupied space')
            return Path()

        # Plan path using A* algorithm
        path_nodes = self.a_star_planning(start_map, goal_map)

        if not path_nodes:
            self.get_logger().error('No path found')
            return Path()

        # Convert path nodes to ROS Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for node in path_nodes:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position = self.map_to_world(node)
            # Set orientation to face next point
            if path_nodes.index(node) < len(path_nodes) - 1:
                next_node = path_nodes[path_nodes.index(node) + 1]
                yaw = math.atan2(next_node[1] - node[1], next_node[0] - node[0])
                pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
                pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # Keep same orientation as previous point or use goal orientation
                pose_stamped.pose.orientation.w = 1.0

            path_msg.poses.append(pose_stamped)

        # Post-process path for humanoid requirements
        path_msg = self.post_process_humanoid_path(path_msg)

        return path_msg

    def a_star_planning(self, start, goal):
        """A* path planning algorithm"""
        # Implementation of A* algorithm
        # This is a simplified version - in practice, you'd use Nav2's built-in planners
        # or implement a more sophisticated version

        open_set = [start]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            # Find node with lowest f_score
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            open_set.remove(current)

            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                if self.is_occupied(neighbor):
                    continue

                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    if neighbor not in open_set:
                        open_set.append(neighbor)

        return []  # No path found

    def heuristic(self, a, b):
        """Heuristic function for A* (Euclidean distance)"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, node):
        """Get 8-connected neighbors"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (node[0] + dx, node[1] + dy)
                if self.is_valid_coordinate(neighbor):
                    neighbors.append(neighbor)
        return neighbors

    def distance(self, a, b):
        """Calculate distance between two nodes"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def is_valid_coordinate(self, coord):
        """Check if coordinate is within map bounds"""
        x, y = coord
        return 0 <= x < self.map_width and 0 <= y < self.map_height

    def is_occupied(self, coord):
        """Check if coordinate is occupied in map"""
        x, y = coord
        if not self.is_valid_coordinate(coord):
            return True

        # Check occupancy value (typically > 50 is considered occupied)
        occupancy_value = self.map_data[y, x]
        return occupancy_value > 50

    def world_to_map(self, point):
        """Convert world coordinates to map coordinates"""
        if self.map_origin is None:
            return (0, 0)

        map_x = int((point.x - self.map_origin.position.x) / self.map_resolution)
        map_y = int((point.y - self.map_origin.position.y) / self.map_resolution)
        return (map_x, map_y)

    def map_to_world(self, coord):
        """Convert map coordinates to world coordinates"""
        from geometry_msgs.msg import Point
        x, y = coord
        world_point = Point()
        world_point.x = x * self.map_resolution + self.map_origin.position.x
        world_point.y = y * self.map_resolution + self.map_origin.position.y
        world_point.z = 0.0
        return world_point

    def post_process_humanoid_path(self, path_msg):
        """Post-process path for humanoid navigation requirements"""
        if len(path_msg.poses) < 2:
            return path_msg

        # Smooth path to reduce sharp turns (important for humanoid balance)
        smoothed_poses = self.smooth_path(path_msg.poses)

        # Ensure path respects step length constraints
        constrained_poses = self.apply_step_constraints(smoothed_poses)

        # Create new path message
        result_path = Path()
        result_path.header = path_msg.header
        result_path.poses = constrained_poses

        return result_path

    def smooth_path(self, poses):
        """Apply path smoothing to reduce sharp turns"""
        if len(poses) < 3:
            return poses

        smoothed = [poses[0]]  # Keep first pose

        for i in range(1, len(poses) - 1):
            prev_pose = poses[i-1]
            curr_pose = poses[i]
            next_pose = poses[i+1]

            # Calculate direction vectors
            dir1_x = curr_pose.pose.position.x - prev_pose.pose.position.x
            dir1_y = curr_pose.pose.position.y - prev_pose.pose.position.y
            dir2_x = next_pose.pose.position.x - curr_pose.pose.position.x
            dir2_y = next_pose.pose.position.y - curr_pose.pose.position.y

            # Calculate angle between directions
            dot_product = dir1_x * dir2_x + dir1_y * dir2_y
            mag1 = math.sqrt(dir1_x**2 + dir1_y**2)
            mag2 = math.sqrt(dir2_x**2 + dir2_y**2)

            if mag1 > 0 and mag2 > 0:
                cos_angle = dot_product / (mag1 * mag2)
                angle = math.acos(max(-1, min(1, cos_angle)))  # Clamp to valid range

                # If angle is too sharp, interpolate intermediate points
                if angle > math.pi / 3:  # More than 60 degrees
                    # Add intermediate point
                    interp_pose = PoseStamped()
                    interp_pose.header = curr_pose.header
                    interp_pose.pose.position.x = (curr_pose.pose.position.x + prev_pose.pose.position.x) / 2
                    interp_pose.pose.position.y = (curr_pose.pose.position.y + prev_pose.pose.position.y) / 2
                    interp_pose.pose.position.z = curr_pose.pose.position.z
                    smoothed.append(interp_pose)

            smoothed.append(curr_pose)

        smoothed.append(poses[-1])  # Keep last pose
        return smoothed

    def apply_step_constraints(self, poses):
        """Apply humanoid step length constraints to path"""
        if len(poses) < 2:
            return poses

        constrained_poses = [poses[0]]  # Start with first pose

        max_step_length = 0.4  # Maximum step length for humanoid (meters)

        for i in range(1, len(poses)):
            prev_pose = constrained_poses[-1]
            curr_pose = poses[i]

            # Calculate distance between previous and current pose
            dx = curr_pose.pose.position.x - prev_pose.pose.position.x
            dy = curr_pose.pose.position.y - prev_pose.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            # If distance is too large, add intermediate poses
            if distance > max_step_length:
                num_steps = int(math.ceil(distance / max_step_length))
                step_dx = dx / num_steps
                step_dy = dy / num_steps

                for step in range(1, num_steps):
                    interp_pose = PoseStamped()
                    interp_pose.header = curr_pose.header
                    interp_pose.pose.position.x = prev_pose.pose.position.x + step * step_dx
                    interp_pose.pose.position.y = prev_pose.pose.position.y + step * step_dy
                    interp_pose.pose.position.z = curr_pose.pose.position.z

                    # Interpolate orientation
                    constrained_poses.append(interp_pose)

            # Add the original pose
            constrained_poses.append(curr_pose)

        return constrained_poses

def main(args=None):
    rclpy.init(args=args)
    planner = HumanoidGlobalPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Global planner stopped')
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Path Planning Quality Metrics

#### Evaluating Global Path Quality
```python
# path_quality_evaluator.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import numpy as np
import math

class PathQualityEvaluator(Node):
    def __init__(self):
        super().__init__('path_quality_evaluator')

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)

        # Publishers
        self.length_pub = self.create_publisher(Float32, '/path/length', 10)
        self.smoothness_pub = self.create_publisher(Float32, '/path/smoothness', 10)
        self.collision_free_pub = self.create_publisher(Float32, '/path/collision_free', 10)

        self.get_logger().info('Path Quality Evaluator initialized')

    def path_callback(self, msg):
        """Evaluate path quality metrics"""
        if len(msg.poses) < 2:
            return

        # Calculate path length
        length = self.calculate_path_length(msg.poses)
        length_msg = Float32()
        length_msg.data = float(length)
        self.length_pub.publish(length_msg)

        # Calculate smoothness (curvature-based)
        smoothness = self.calculate_smoothness(msg.poses)
        smoothness_msg = Float32()
        smoothness_msg.data = float(smoothness)
        self.smoothness_pub.publish(smoothness_msg)

        # Log metrics
        self.get_logger().info(
            f'Path Quality - Length: {length:.2f}m, '
            f'Smoothness: {smoothness:.3f}'
        )

    def calculate_path_length(self, poses):
        """Calculate total path length"""
        if len(poses) < 2:
            return 0.0

        length = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i-1].pose.position.x
            dy = poses[i].pose.position.y - poses[i-1].pose.position.y
            length += math.sqrt(dx*dx + dy*dy)

        return length

    def calculate_smoothness(self, poses):
        """Calculate path smoothness based on curvature"""
        if len(poses) < 3:
            return 1.0  # Perfectly smooth for short paths

        total_curvature = 0.0
        segment_count = 0

        for i in range(1, len(poses) - 1):
            p1 = np.array([poses[i-1].pose.position.x, poses[i-1].pose.position.y])
            p2 = np.array([poses[i].pose.position.x, poses[i].pose.position.y])
            p3 = np.array([poses[i+1].pose.position.x, poses[i+1].pose.position.y])

            # Calculate angle at point p2
            v1 = p2 - p1
            v2 = p3 - p2
            dot_product = np.dot(v1, v2)
            norms = np.linalg.norm(v1) * np.linalg.norm(v2)

            if norms > 0:
                cos_angle = dot_product / norms
                # Clamp to valid range to avoid numerical errors
                cos_angle = max(-1, min(1, cos_angle))
                angle = math.acos(cos_angle)
                # Curvature is related to how much the direction changes
                curvature = math.pi - angle  # Convert to deviation from straight line
                total_curvature += curvature
                segment_count += 1

        if segment_count > 0:
            avg_curvature = total_curvature / segment_count
            # Convert to smoothness (lower curvature = higher smoothness)
            smoothness = 1.0 / (1.0 + avg_curvature)
        else:
            smoothness = 1.0

        return smoothness

def main(args=None):
    rclpy.init(args=args)
    evaluator = PathQualityEvaluator()
    rclpy.spin(evaluator)
    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Local Path Planning

### 1. Navigation2 Local Planners

#### DWB (Dynamic Window Approach) Configuration
```yaml
# local_planner_params.yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0  # Higher frequency for local planning
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.05
    min_theta_velocity_threshold: 0.05
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # DWB Local Planner Configuration
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.05      # Minimum forward speed
      min_vel_y: 0.0
      max_vel_x: 0.5       # Maximum forward speed
      max_vel_y: 0.0
      max_vel_theta: 1.0   # Maximum angular speed
      min_speed_xy: 0.1
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5       # Linear acceleration limit
      acc_lim_y: 0.0
      acc_lim_theta: 3.2   # Angular acceleration limit
      decel_lim_x: -2.5    # Linear deceleration limit
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20       # Velocity samples in x direction
      vy_samples: 5        # Velocity samples in y direction
      vtheta_samples: 40   # Velocity samples in theta direction
      sim_time: 1.7        # Simulation time for trajectory evaluation
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25  # Tolerance for reaching goal xy position
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
```

### 2. Custom Local Planner for Humanoid Robots

#### Humanoid-Aware Local Planner
```python
# humanoid_local_planner.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import VelocitySmoother
from std_msgs.msg import Header
import numpy as np
import math

class HumanoidLocalPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_local_planner')

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.velocity_smoother_pub = self.create_publisher(VelocitySmoother, '/velocity_smoother', 10)

        # Parameters
        self.declare_parameter('max_linear_vel', 0.3)  # Reduced for humanoid
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('min_linear_vel', 0.05)
        self.declare_parameter('min_angular_vel', 0.1)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('obstacle_threshold', 0.5)

        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.min_linear_vel = self.get_parameter('min_linear_vel').get_parameter_value().double_value
        self.min_angular_vel = self.get_parameter('min_angular_vel').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value

        # Internal state
        self.current_path = None
        self.current_pose = None
        self.obstacle_distances = []
        self.path_index = 0
        self.balance_ok = True

        # Timer for local planning
        self.planning_timer = self.create_timer(0.05, self.local_planning_callback)  # 20 Hz

        self.get_logger().info('Humanoid Local Planner initialized')

    def path_callback(self, msg):
        """Receive global path from global planner"""
        self.current_path = msg
        self.path_index = 0  # Reset path index when new path received

    def odom_callback(self, msg):
        """Receive robot odometry"""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Receive laser scan data"""
        self.obstacle_distances = [
            r for r in msg.ranges
            if not (math.isnan(r) or math.isinf(r))
        ]

    def local_planning_callback(self):
        """Main local planning loop"""
        if not self.current_pose or not self.current_path or len(self.current_path.poses) == 0:
            return

        # Check if we've reached the goal
        if self.has_reached_goal():
            self.stop_robot()
            return

        # Get next path segment to follow
        next_waypoint = self.get_next_waypoint()
        if not next_waypoint:
            self.stop_robot()
            return

        # Calculate control commands
        cmd_vel = self.calculate_control_command(next_waypoint)

        # Check for obstacles and adjust if needed
        cmd_vel = self.avoid_obstacles(cmd_vel)

        # Apply humanoid-specific constraints
        cmd_vel = self.apply_humanoid_constraints(cmd_vel)

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)

    def get_next_waypoint(self):
        """Get the next waypoint from the global path"""
        if not self.current_path or self.path_index >= len(self.current_path.poses):
            return None

        # Find the next waypoint that's far enough from current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        while self.path_index < len(self.current_path.poses):
            waypoint = self.current_path.poses[self.path_index]
            dx = waypoint.pose.position.x - current_x
            dy = waypoint.pose.position.y - current_y
            distance = math.sqrt(dx*dx + dy*dy)

            # If waypoint is far enough, return it
            if distance > 0.5:  # Look ahead distance
                return waypoint

            # Move to next waypoint
            self.path_index += 1

        # If we've reached the end of the path, return the last waypoint
        if len(self.current_path.poses) > 0:
            return self.current_path.poses[-1]

        return None

    def calculate_control_command(self, waypoint):
        """Calculate velocity command to reach the given waypoint"""
        cmd_vel = Twist()

        if not self.current_pose:
            return cmd_vel

        # Calculate desired direction
        desired_x = waypoint.pose.position.x
        desired_y = waypoint.pose.position.y
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        dx = desired_x - current_x
        dy = desired_y - current_y

        # Calculate distance to waypoint
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate desired angle
        desired_angle = math.atan2(dy, dx)

        # Get current orientation
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # Calculate angle difference
        angle_diff = desired_angle - current_yaw
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Calculate linear velocity based on distance
        if distance > self.goal_tolerance:
            linear_vel = min(self.max_linear_vel, max(self.min_linear_vel, distance * 1.0))
        else:
            linear_vel = 0.0

        # Calculate angular velocity based on angle difference
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angle_diff * 2.0))

        # Apply humanoid-specific constraints
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel

        return cmd_vel

    def avoid_obstacles(self, cmd_vel):
        """Modify velocity command to avoid obstacles"""
        if not self.obstacle_distances:
            return cmd_vel

        # Check for obstacles in front
        front_distances = self.obstacle_distances[len(self.obstacle_distances)//2-50:len(self.obstacle_distances)//2+50]
        if front_distances:
            min_front_distance = min(front_distances) if front_distances else float('inf')

            if min_front_distance < self.obstacle_threshold:
                # Reduce linear velocity proportionally to obstacle distance
                reduction_factor = min_front_distance / self.obstacle_threshold
                cmd_vel.linear.x *= reduction_factor

                # If very close, consider stopping
                if min_front_distance < 0.2:
                    cmd_vel.linear.x = 0.0

        return cmd_vel

    def apply_humanoid_constraints(self, cmd_vel):
        """Apply humanoid-specific motion constraints"""
        # Limit velocities for humanoid stability
        cmd_vel.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, cmd_vel.linear.x))
        cmd_vel.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, cmd_vel.angular.z))

        # Ensure minimum velocities for stable walking
        if abs(cmd_vel.linear.x) > 0 and abs(cmd_vel.linear.x) < self.min_linear_vel:
            cmd_vel.linear.x = self.min_linear_vel if cmd_vel.linear.x > 0 else -self.min_linear_vel

        if abs(cmd_vel.angular.z) > 0 and abs(cmd_vel.angular.z) < self.min_angular_vel:
            cmd_vel.angular.z = self.min_angular_vel if cmd_vel.angular.z > 0 else -self.min_angular_vel

        return cmd_vel

    def has_reached_goal(self):
        """Check if robot has reached the goal"""
        if not self.current_path or not self.current_pose:
            return False

        # Get the last pose in the path (goal)
        goal_pose = self.current_path.poses[-1].pose.position

        # Calculate distance to goal
        dx = goal_pose.x - self.current_pose.position.x
        dy = goal_pose.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        return distance <= self.goal_tolerance

    def stop_robot(self):
        """Stop the robot"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def get_yaw_from_quaternion(self, q):
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    planner = HumanoidLocalPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Local planner stopped')
        # Stop robot on shutdown
        stop_cmd = Twist()
        planner.cmd_vel_pub.publish(stop_cmd)
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Trajectory Generation and Evaluation

#### Local Trajectory Evaluator
```python
# trajectory_evaluator.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

class TrajectoryEvaluator(Node):
    def __init__(self):
        super().__init__('trajectory_evaluator')

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.trajectory_score_pub = self.create_publisher(Float32, '/trajectory/score', 10)
        self.trajectory_viz_pub = self.create_publisher(MarkerArray, '/trajectory/visualization', 10)

        # Internal state
        self.current_path = None
        self.current_pose = None
        self.current_cmd = None

        self.get_logger().info('Trajectory Evaluator initialized')

    def path_callback(self, msg):
        self.current_path = msg

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def cmd_vel_callback(self, msg):
        self.current_cmd = msg

    def evaluate_trajectory(self):
        """Evaluate the quality of the current trajectory"""
        if not self.current_pose or not self.current_cmd:
            return

        # Simulate trajectory based on current command
        simulated_trajectory = self.simulate_trajectory(self.current_pose, self.current_cmd)

        # Evaluate trajectory quality
        score = self.calculate_trajectory_score(simulated_trajectory)

        # Publish score
        score_msg = Float32()
        score_msg.data = float(score)
        self.trajectory_score_pub.publish(score_msg)

        # Visualize trajectory
        self.visualize_trajectory(simulated_trajectory)

        self.get_logger().info(f'Trajectory Score: {score:.3f}')

    def simulate_trajectory(self, start_pose, cmd_vel, duration=1.0, dt=0.1):
        """Simulate trajectory based on current command"""
        trajectory = []
        current_pose = start_pose

        num_steps = int(duration / dt)
        for i in range(num_steps):
            # Create pose for this time step
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose = current_pose

            trajectory.append(pose_stamped)

            # Update pose based on velocity command
            new_pose = self.update_pose(current_pose, cmd_vel, dt)
            current_pose = new_pose

        return trajectory

    def update_pose(self, current_pose, cmd_vel, dt):
        """Update pose based on velocity command"""
        from geometry_msgs.msg import Pose
        new_pose = Pose()

        # Get current orientation
        current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)

        # Calculate new position
        dx = cmd_vel.linear.x * dt * math.cos(current_yaw) - cmd_vel.linear.y * dt * math.sin(current_yaw)
        dy = cmd_vel.linear.x * dt * math.sin(current_yaw) + cmd_vel.linear.y * dt * math.cos(current_yaw)

        new_pose.position.x = current_pose.position.x + dx
        new_pose.position.y = current_pose.position.y + dy
        new_pose.position.z = current_pose.position.z

        # Update orientation
        new_yaw = current_yaw + cmd_vel.angular.z * dt
        new_pose.orientation.z = math.sin(new_yaw / 2.0)
        new_pose.orientation.w = math.cos(new_yaw / 2.0)

        return new_pose

    def calculate_trajectory_score(self, trajectory):
        """Calculate trajectory quality score"""
        if len(trajectory) < 2:
            return 0.0

        # Factors for trajectory scoring
        safety_score = self.evaluate_safety(trajectory)
        efficiency_score = self.evaluate_efficiency(trajectory)
        smoothness_score = self.evaluate_smoothness(trajectory)

        # Weighted combination of factors
        total_score = 0.4 * safety_score + 0.4 * efficiency_score + 0.2 * smoothness_score

        return total_score

    def evaluate_safety(self, trajectory):
        """Evaluate trajectory safety (avoiding obstacles)"""
        # This would typically involve checking against local costmap
        # For this example, we'll return a placeholder value
        return 0.8  # Assume 80% safety

    def evaluate_efficiency(self, trajectory):
        """Evaluate trajectory efficiency (progress toward goal)"""
        if not self.current_path or len(self.current_path.poses) == 0:
            return 0.5  # Neutral score if no path

        # Calculate progress toward goal
        start_pose = trajectory[0].pose
        end_pose = trajectory[-1].pose

        # Find closest points on global path to start and end of trajectory
        start_dist = self.distance_to_path(start_pose.position, self.current_path.poses)
        end_dist = self.distance_to_path(end_pose.position, self.current_path.poses)

        # Efficiency is better if we're moving toward the goal
        if start_dist > end_dist:
            efficiency = min(1.0, (start_dist - end_dist) / 1.0)  # Normalize
        else:
            efficiency = 0.1  # Low efficiency if moving away

        return efficiency

    def evaluate_smoothness(self, trajectory):
        """Evaluate trajectory smoothness"""
        if len(trajectory) < 3:
            return 1.0

        total_curvature = 0.0
        segment_count = 0

        for i in range(1, len(trajectory) - 1):
            p1 = np.array([trajectory[i-1].pose.position.x, trajectory[i-1].pose.position.y])
            p2 = np.array([trajectory[i].pose.position.x, trajectory[i].pose.position.y])
            p3 = np.array([trajectory[i+1].pose.position.x, trajectory[i+1].pose.position.y])

            # Calculate angle at point p2
            v1 = p2 - p1
            v2 = p3 - p2
            dot_product = np.dot(v1, v2)
            norms = np.linalg.norm(v1) * np.linalg.norm(v2)

            if norms > 0:
                cos_angle = dot_product / norms
                # Clamp to valid range to avoid numerical errors
                cos_angle = max(-1, min(1, cos_angle))
                angle = math.acos(cos_angle)
                # Convert to deviation from straight line
                curvature = math.pi - angle
                total_curvature += curvature
                segment_count += 1

        if segment_count > 0:
            avg_curvature = total_curvature / segment_count
            # Convert to smoothness (lower curvature = higher smoothness)
            smoothness = 1.0 / (1.0 + avg_curvature * 10)  # Scale factor for normalization
        else:
            smoothness = 1.0

        return smoothness

    def distance_to_path(self, point, path_poses):
        """Calculate minimum distance from point to any point on path"""
        if len(path_poses) == 0:
            return float('inf')

        min_distance = float('inf')
        for pose in path_poses:
            dx = pose.pose.position.x - point.x
            dy = pose.pose.position.y - point.y
            distance = math.sqrt(dx*dx + dy*dy)
            min_distance = min(min_distance, distance)

        return min_distance

    def visualize_trajectory(self, trajectory):
        """Visualize trajectory in RViz"""
        marker_array = MarkerArray()

        # Create trajectory line marker
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'trajectory'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8

        for pose_stamped in trajectory:
            point = Point()
            point.x = pose_stamped.pose.position.x
            point.y = pose_stamped.pose.position.y
            point.z = pose_stamped.pose.position.z
            line_marker.points.append(point)

        marker_array.markers.append(line_marker)

        # Publish marker array
        self.trajectory_viz_pub.publish(marker_array)

    def get_yaw_from_quaternion(self, q):
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    evaluator = TrajectoryEvaluator()

    # Timer for periodic evaluation
    timer = evaluator.create_timer(0.5, evaluator.evaluate_trajectory)

    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        evaluator.get_logger().info('Trajectory evaluator stopped')
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration and Testing

### 1. Complete Path Planning System

#### Launch File for Complete System
```python
# complete_path_planning_launch.py
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
    path_planning_dir = get_package_share_directory('path_planning_tutorials')

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
                path_planning_dir,
                'config',
                params_file
            ])
        }.items()
    )

    # Path planning evaluation container
    evaluation_container = ComposableNodeContainer(
        name='path_planning_evaluation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Path quality evaluator
            ComposableNode(
                package='path_planning_tutorials',
                plugin='path_planning_tutorials::PathQualityEvaluator',
                name='path_quality_evaluator',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            # Trajectory evaluator
            ComposableNode(
                package='path_planning_tutorials',
                plugin='path_planning_tutorials::TrajectoryEvaluator',
                name='trajectory_evaluator',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        nav2_bringup_launch,
        evaluation_container
    ])
```

### 2. Performance Testing

#### Path Planning Benchmarking
```python
# path_planning_benchmark.py
import rclpy
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import time
import numpy as np

class PathPlanningBenchmark(Node):
    def __init__(self):
        super().__init__('path_planning_benchmark')

        # Action client for path planning
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Publishers
        self.planning_time_pub = self.create_publisher(Float32, '/benchmark/planning_time', 10)
        self.path_length_pub = self.create_publisher(Float32, '/benchmark/path_length', 10)

        # Test parameters
        self.test_poses = [
            # Define test start and goal poses
            ((0.0, 0.0), (5.0, 5.0)),
            ((1.0, 1.0), (10.0, 10.0)),
            ((-2.0, -2.0), (8.0, -5.0)),
        ]

        # Timer for running benchmarks
        self.benchmark_timer = self.create_timer(5.0, self.run_benchmark)

        self.get_logger().info('Path Planning Benchmark initialized')

    def run_benchmark(self):
        """Run path planning benchmark"""
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Path planning server not available')
            return

        # Run each test case
        for i, (start_pos, goal_pos) in enumerate(self.test_poses):
            self.get_logger().info(f'Running benchmark {i+1}/{len(self.test_poses)}')

            # Create goal message
            goal_msg = ComputePathToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = goal_pos[0]
            goal_msg.pose.pose.position.y = goal_pos[1]
            goal_msg.pose.pose.orientation.w = 1.0

            # Send goal and time the operation
            start_time = time.time()
            future = self.action_client.send_goal_async(goal_msg)
            # Note: In a real implementation, you'd wait for result and calculate metrics

            planning_time = time.time() - start_time

            # Publish planning time
            time_msg = Float32()
            time_msg.data = float(planning_time)
            self.planning_time_pub.publish(time_msg)

            self.get_logger().info(f'Planning time: {planning_time:.3f}s')

def main(args=None):
    rclpy.init(args=args)
    benchmark = PathPlanningBenchmark()

    try:
        rclpy.spin(benchmark)
    except KeyboardInterrupt:
        benchmark.get_logger().info('Path planning benchmark stopped')
    finally:
        benchmark.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices and Tips

### 1. Path Planning Optimization

#### Parameter Tuning Guidelines
- **Global Planner**: Focus on finding feasible paths with reasonable optimality
- **Local Planner**: Prioritize safety and obstacle avoidance over optimality
- **Humanoid Constraints**: Always consider balance and step limitations
- **Frequency**: Balance between responsiveness and computational load

#### Common Pitfalls to Avoid
- Setting tolerances too tight for humanoid capabilities
- Ignoring sensor limitations and noise
- Not accounting for robot dynamics
- Overlooking recovery behaviors

### 2. Safety Considerations

#### Path Validation
- Always validate paths for kinematic feasibility
- Check for collisions with updated sensor data
- Implement proper timeout mechanisms
- Monitor for oscillation and stuck conditions

This comprehensive tutorial provides the foundation for implementing both global and local path planning systems suitable for humanoid robots, with practical examples and evaluation techniques.