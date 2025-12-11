# Navigation Testing Procedures in Simulated Environments

## Overview

This tutorial provides comprehensive procedures for testing navigation systems in simulated environments, specifically focusing on Navigation2 (Nav2) stack with humanoid robots. The procedures cover various testing scenarios, evaluation metrics, and validation techniques to ensure robust navigation performance before real-world deployment.

## Testing Environment Setup

### 1. Simulation Platform Selection

#### Isaac Sim for High-Fidelity Testing
Isaac Sim provides photorealistic simulation capabilities ideal for navigation testing:

```python
# isaac_sim_navigation_test.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.carb import set_carb_setting
from omni.isaac.navigation import NavigationWorld
import carb

def setup_navigation_test_environment():
    """Setup Isaac Sim environment for navigation testing"""
    # Configure Isaac Sim settings
    set_carb_setting("/persistent/isaac/asset_root/default", get_assets_root_path())
    set_carb_setting("/app/player/play_sim_on_startup", True)
    set_carb_setting("/app/window/drawMouse", True)
    set_carb_setting("/app/window/hideSafezone", True)
    set_carb_setting("/persistent/app/window/resizable", True)
    set_carb_setting("/app/window/width", 1920)
    set_carb_setting("/app/window/height", 1080)

    # Initialize world
    world = World(stage_units_in_meters=1.0)

    # Add navigation testing environment
    assets_root_path = get_assets_root_path()
    if assets_root_path:
        # Add a complex indoor environment for navigation testing
        env_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        add_reference_to_stage(usd_path=env_path, prim_path="/World/NavigationEnv")

    return world

def add_humanoid_robot_to_simulation(world):
    """Add humanoid robot to the simulation environment"""
    # Add humanoid robot (example using a simple humanoid model)
    robot_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid.usd"
    add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")

    # Configure robot for navigation testing
    robot = world.scene.add_default_ground_plane()
    return robot

def main():
    # Setup navigation test environment
    world = setup_navigation_test_environment()
    robot = add_humanoid_robot_to_simulation(world)

    # Reset and run simulation
    world.reset()

    # Run for specified number of steps
    for i in range(10000):
        world.step(render=True)

        if i % 100 == 0:
            print(f"Simulation step: {i}")

if __name__ == "__main__":
    main()
```

#### Gazebo for Standard Testing
For standard ROS-based testing, Gazebo provides a reliable simulation environment:

```bash
# Gazebo navigation test launch
# navigation_test_world.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='maze_world.world')

    # Launch Gazebo with navigation test world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': PathJoinSubstitution([
                get_package_share_directory('navigation_test_pkg'),
                'worlds',
                world_file
            ])
        }.items()
    )

    return LaunchDescription([
        gazebo_launch
    ])
```

### 2. Test World Design

#### Navigation Test World Features
```xml
<!-- navigation_test_world.sdf -->
<sdf version="1.7">
  <world name="navigation_test_world">
    <!-- Include common assets -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Navigation test environment elements -->
    <!-- Corridors with varying widths -->
    <model name="corridor_narrow">
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.8 2</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.8 2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Doorways of different sizes -->
    <model name="doorway_standard">
      <pose>5 0 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.8 2</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.8 2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Obstacles of various shapes and sizes -->
    <model name="obstacle_cylinder">
      <pose>2 2 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Dynamic obstacles -->
    <model name="moving_obstacle">
      <pose>0 3 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <!-- Add simple controller for movement -->
      </link>
    </model>
  </world>
</sdf>
```

## Test Case Development

### 1. Basic Navigation Tests

#### Straight Line Navigation Test
```python
# straight_line_navigation_test.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import TransformListener, Buffer
import math
import time

class StraightLineNavigationTest(Node):
    def __init__(self):
        super().__init__('straight_line_navigation_test')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF listener for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Test parameters
        self.test_distance = 5.0  # meters
        self.test_tolerance = 0.5  # meters

        self.get_logger().info('Straight Line Navigation Test initialized')

    def run_test(self):
        """Execute straight line navigation test"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False

        # Calculate goal position (5 meters forward)
        goal_pose = self.create_goal_pose(5.0, 0.0, 0.0)  # 5m in x direction

        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending navigation goal...')
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        status = get_result_future.result().status

        if status == 3:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
            return self.verify_final_position(5.0, 0.0, self.test_tolerance)
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            return False

    def create_goal_pose(self, x, y, theta):
        """Create a goal pose message"""
        from geometry_msgs.msg import PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # Convert theta to quaternion
        from math import sin, cos
        goal_pose.pose.orientation.z = sin(theta / 2.0)
        goal_pose.pose.orientation.w = cos(theta / 2.0)

        return goal_pose

    def verify_final_position(self, expected_x, expected_y, tolerance):
        """Verify that robot reached expected position"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            actual_x = transform.transform.translation.x
            actual_y = transform.transform.translation.y

            distance_error = math.sqrt(
                (actual_x - expected_x)**2 + (actual_y - expected_y)**2)

            if distance_error <= tolerance:
                self.get_logger().info(
                    f'Position verification passed: '
                    f'Expected ({expected_x}, {expected_y}), '
                    f'Actual ({actual_x:.2f}, {actual_y:.2f}), '
                    f'Error: {distance_error:.2f}m')
                return True
            else:
                self.get_logger().error(
                    f'Position verification failed: '
                    f'Error {distance_error:.2f}m > tolerance {tolerance}m')
                return False
        except Exception as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    test = StraightLineNavigationTest()

    success = test.run_test()
    if success:
        test.get_logger().info('Straight line navigation test PASSED')
    else:
        test.get_logger().error('Straight line navigation test FAILED')

    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Obstacle Avoidance Test
```python
# obstacle_avoidance_test.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

class ObstacleAvoidanceTest(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_test')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publishers for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        # Internal state
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.obstacle_angle = 0.0

        self.get_logger().info('Obstacle Avoidance Test initialized')

    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Find closest obstacle in front of robot
        front_scan_start = len(msg.ranges) // 2 - 30  # 30 indices before center
        front_scan_end = len(msg.ranges) // 2 + 30    # 30 indices after center

        if 0 <= front_scan_start < len(msg.ranges) and 0 <= front_scan_end < len(msg.ranges):
            front_ranges = msg.ranges[front_scan_start:front_scan_end]

            # Remove invalid readings
            valid_ranges = [r for r in front_ranges if not (math.isnan(r) or math.isinf(r))]

            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)
                closest_idx = front_ranges.index(self.obstacle_distance)
                self.obstacle_angle = msg.angle_min + (closest_idx + front_scan_start) * msg.angle_increment
                self.obstacle_detected = self.obstacle_distance < 2.0  # Obstacle within 2m

    def run_test(self):
        """Execute obstacle avoidance test"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False

        # Set goal that requires obstacle avoidance
        goal_pose = self.create_goal_pose(8.0, 2.0, 0.0)  # Goal that requires path around obstacle

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending navigation goal with obstacle avoidance...')
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        status = get_result_future.result().status

        if status == 3:  # SUCCEEDED
            self.get_logger().info('Obstacle avoidance navigation succeeded!')
            return True
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            return False

    def create_goal_pose(self, x, y, theta):
        """Create a goal pose message"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        from math import sin, cos
        goal_pose.pose.orientation.z = sin(theta / 2.0)
        goal_pose.pose.orientation.w = cos(theta / 2.0)

        return goal_pose

def main(args=None):
    rclpy.init(args=args)
    test = ObstacleAvoidanceTest()

    success = test.run_test()
    if success:
        test.get_logger().info('Obstacle avoidance test PASSED')
    else:
        test.get_logger().error('Obstacle avoidance test FAILED')

    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Complex Navigation Tests

#### Maze Navigation Test
```python
# maze_navigation_test.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Float32
import math

class MazeNavigationTest(Node):
    def __init__(self):
        super().__init__('maze_navigation_test')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers for metrics
        self.path_efficiency_pub = self.create_publisher(Float32, '/test/path_efficiency', 10)
        self.success_rate_pub = self.create_publisher(Float32, '/test/success_rate', 10)

        # Test parameters
        self.maze_goals = [
            (5.0, 0.0, 0.0),    # Goal 1
            (10.0, 5.0, 1.57),  # Goal 2
            (2.0, 8.0, 3.14),   # Goal 3
            (0.0, 0.0, 0.0),    # Return to start
        ]

        self.get_logger().info('Maze Navigation Test initialized')

    def run_maze_test(self):
        """Execute complete maze navigation test"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False

        success_count = 0
        total_goals = len(self.maze_goals)

        for i, (x, y, theta) in enumerate(self.maze_goals):
            self.get_logger().info(f'Navigating to goal {i+1}/{total_goals}: ({x}, {y})')

            goal_pose = self.create_goal_pose(x, y, theta)
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose

            future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(f'Goal {i+1} rejected')
                continue

            # Wait for result
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)

            result = get_result_future.result().result
            status = get_result_future.result().status

            if status == 3:  # SUCCEEDED
                self.get_logger().info(f'Goal {i+1} reached successfully')
                success_count += 1
            else:
                self.get_logger().error(f'Goal {i+1} failed with status: {status}')

        success_rate = success_count / total_goals
        self.get_logger().info(f'Maze test completed: {success_count}/{total_goals} goals reached')

        # Publish success rate
        rate_msg = Float32()
        rate_msg.data = float(success_rate)
        self.success_rate_pub.publish(rate_msg)

        return success_rate >= 0.8  # Require 80% success rate

    def create_goal_pose(self, x, y, theta):
        """Create a goal pose message"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        from math import sin, cos
        goal_pose.pose.orientation.z = sin(theta / 2.0)
        goal_pose.pose.orientation.w = cos(theta / 2.0)

        return goal_pose

def main(args=None):
    rclpy.init(args=args)
    test = MazeNavigationTest()

    success = test.run_maze_test()
    if success:
        test.get_logger().info('Maze navigation test PASSED')
    else:
        test.get_logger().error('Maze navigation test FAILED')

    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Evaluation Metrics

### 1. Navigation Metrics Collection

#### Navigation Performance Metrics Node
```python
# navigation_metrics_collector.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32
from tf2_ros import TransformListener, Buffer
import numpy as np
import math
import time

class NavigationMetricsCollector(Node):
    def __init__(self):
        super().__init__('navigation_metrics_collector')

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers for metrics
        self.path_length_pub = self.create_publisher(Float32, '/metrics/path_length', 10)
        self.executed_distance_pub = self.create_publisher(Float32, '/metrics/executed_distance', 10)
        self.path_efficiency_pub = self.create_publisher(Float32, '/metrics/path_efficiency', 10)
        self.navigation_time_pub = self.create_publisher(Float32, '/metrics/navigation_time', 10)
        self.success_pub = self.create_publisher(Int32, '/metrics/navigation_success', 10)

        # Internal state
        self.planned_path = []
        self.executed_trajectory = []
        self.start_time = None
        self.goal_pose = None
        self.current_pose = None
        self.navigation_active = False

        self.get_logger().info('Navigation Metrics Collector initialized')

    def path_callback(self, msg):
        """Store planned path for metrics calculation"""
        self.planned_path = msg.poses

    def odom_callback(self, msg):
        """Track robot's executed trajectory"""
        self.current_pose = msg.pose.pose

        if self.navigation_active:
            # Add current position to executed trajectory
            current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.executed_trajectory.append(current_pos)

    def goal_callback(self, msg):
        """Start metrics collection when new goal is received"""
        self.goal_pose = msg.pose
        self.start_time = time.time()
        self.navigation_active = True
        self.executed_trajectory = []
        self.get_logger().info('Started collecting navigation metrics')

    def check_navigation_completion(self):
        """Check if navigation has been completed and calculate metrics"""
        if not self.navigation_active or not self.current_pose or not self.goal_pose:
            return

        # Check if robot is close to goal
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        distance_to_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

        if distance_to_goal < 0.5:  # 50cm tolerance
            self.calculate_and_publish_metrics()
            self.navigation_active = False

    def calculate_and_publish_metrics(self):
        """Calculate and publish navigation metrics"""
        if not self.planned_path or not self.executed_trajectory:
            return

        # Calculate planned path length
        planned_length = self.calculate_path_length(self.planned_path)

        # Calculate executed trajectory length
        executed_length = self.calculate_trajectory_length(self.executed_trajectory)

        # Calculate path efficiency (direct distance / planned path length)
        start_pos = (self.planned_path[0].pose.position.x, self.planned_path[0].pose.position.y)
        goal_pos = (self.planned_path[-1].pose.position.x, self.planned_path[-1].pose.position.y)
        direct_distance = math.sqrt((goal_pos[0] - start_pos[0])**2 + (goal_pos[1] - start_pos[1])**2)

        path_efficiency = direct_distance / planned_length if planned_length > 0 else 0

        # Calculate navigation time
        navigation_time = time.time() - self.start_time if self.start_time else 0

        # Calculate success (1 for success, 0 for failure)
        success = 1 if len(self.executed_trajectory) > 10 else 0  # Arbitrary threshold

        # Publish metrics
        planned_len_msg = Float32()
        planned_len_msg.data = float(planned_length)
        self.path_length_pub.publish(planned_len_msg)

        executed_len_msg = Float32()
        executed_len_msg.data = float(executed_length)
        self.executed_distance_pub.publish(executed_len_msg)

        efficiency_msg = Float32()
        efficiency_msg.data = float(path_efficiency)
        self.path_efficiency_pub.publish(efficiency_msg)

        time_msg = Float32()
        time_msg.data = float(navigation_time)
        self.navigation_time_pub.publish(time_msg)

        success_msg = Int32()
        success_msg.data = success
        self.success_pub.publish(success_msg)

        self.get_logger().info(
            f'Navigation Metrics - Planned: {planned_length:.2f}m, '
            f'Executed: {executed_length:.2f}m, '
            f'Efficiency: {path_efficiency:.3f}, '
            f'Time: {navigation_time:.2f}s'
        )

    def calculate_path_length(self, poses):
        """Calculate total length of a path"""
        if len(poses) < 2:
            return 0.0

        total_length = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i-1].pose.position.x
            dy = poses[i].pose.position.y - poses[i-1].pose.position.y
            segment_length = math.sqrt(dx*dx + dy*dy)
            total_length += segment_length

        return total_length

    def calculate_trajectory_length(self, positions):
        """Calculate total length of executed trajectory"""
        if len(positions) < 2:
            return 0.0

        total_length = 0.0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            segment_length = math.sqrt(dx*dx + dy*dy)
            total_length += segment_length

        return total_length

def main(args=None):
    rclpy.init(args=args)
    metrics_collector = NavigationMetricsCollector()

    # Timer for checking navigation completion
    timer = metrics_collector.create_timer(1.0, metrics_collector.check_navigation_completion)

    try:
        rclpy.spin(metrics_collector)
    except KeyboardInterrupt:
        metrics_collector.get_logger().info('Metrics collector stopped')
    finally:
        metrics_collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Stress Testing Procedures

#### Navigation Stress Test
```python
# navigation_stress_test.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32
import random
import time

class NavigationStressTest(Node):
    def __init__(self):
        super().__init__('navigation_stress_test')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers for stress test metrics
        self.test_count_pub = self.create_publisher(Int32, '/stress_test/completed_tests', 10)
        self.success_rate_pub = self.create_publisher(Float32, '/stress_test/success_rate', 10)

        # Test parameters
        self.test_count = 0
        self.success_count = 0
        self.max_tests = 50  # Run 50 navigation tests

        self.get_logger().info('Navigation Stress Test initialized')

    def run_stress_test(self):
        """Run multiple navigation tests to stress test the system"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False

        for i in range(self.max_tests):
            self.get_logger().info(f'Running stress test {i+1}/{self.max_tests}')

            # Generate random goal within test area
            goal_x = random.uniform(-10, 10)  # Random x between -10 and 10
            goal_y = random.uniform(-10, 10)  # Random y between -10 and 10
            goal_theta = random.uniform(-3.14, 3.14)  # Random orientation

            success = self.execute_single_test(goal_x, goal_y, goal_theta)

            if success:
                self.success_count += 1

            self.test_count += 1

            # Publish metrics
            count_msg = Int32()
            count_msg.data = self.test_count
            self.test_count_pub.publish(count_msg)

            success_rate_msg = Float32()
            success_rate_msg.data = float(self.success_count) / float(self.test_count) if self.test_count > 0 else 0.0
            self.success_rate_pub.publish(success_rate_msg)

            # Brief pause between tests
            time.sleep(1.0)

        final_success_rate = float(self.success_count) / float(self.max_tests)
        self.get_logger().info(
            f'Stress test completed: {self.success_count}/{self.max_tests} '
            f'successful ({final_success_rate:.2%})')

        return final_success_rate >= 0.8  # Require 80% success rate

    def execute_single_test(self, goal_x, goal_y, goal_theta):
        """Execute a single navigation test"""
        goal_pose = self.create_goal_pose(goal_x, goal_y, goal_theta)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)  # 60 second timeout

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Wait for result with timeout
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=120.0)  # 2 minute timeout

        if get_result_future.done():
            result = get_result_future.result().result
            status = get_result_future.result().status

            if status == 3:  # SUCCEEDED
                self.get_logger().info(f'Navigation to ({goal_x:.2f}, {goal_y:.2f}) succeeded')
                return True
            else:
                self.get_logger().error(f'Navigation failed with status: {status}')
                return False
        else:
            self.get_logger().error('Navigation timed out')
            return False

    def create_goal_pose(self, x, y, theta):
        """Create a goal pose message"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        from math import sin, cos
        goal_pose.pose.orientation.z = sin(theta / 2.0)
        goal_pose.pose.orientation.w = cos(theta / 2.0)

        return goal_pose

def main(args=None):
    rclpy.init(args=args)
    stress_test = NavigationStressTest()

    success = stress_test.run_stress_test()
    if success:
        stress_test.get_logger().info('Navigation stress test PASSED')
    else:
        stress_test.get_logger().error('Navigation stress test FAILED')

    stress_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Automated Testing Framework

### 1. Test Suite Configuration

#### Navigation Test Suite
```python
# navigation_test_suite.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import subprocess
import time
import yaml

class NavigationTestSuite(Node):
    def __init__(self):
        super().__init__('navigation_test_suite')

        # Publishers
        self.test_status_pub = self.create_publisher(String, '/test_suite/status', 10)
        self.test_results_pub = self.create_publisher(String, '/test_suite/results', 10)
        self.test_count_pub = self.create_publisher(Int32, '/test_suite/test_count', 10)

        # Test configuration
        self.tests = [
            {
                'name': 'straight_line_navigation',
                'script': 'straight_line_navigation_test.py',
                'timeout': 30,
                'required': True
            },
            {
                'name': 'obstacle_avoidance',
                'script': 'obstacle_avoidance_test.py',
                'timeout': 60,
                'required': True
            },
            {
                'name': 'maze_navigation',
                'script': 'maze_navigation_test.py',
                'timeout': 120,
                'required': False
            },
            {
                'name': 'stress_test',
                'script': 'navigation_stress_test.py',
                'timeout': 600,  # 10 minutes
                'required': False
            }
        ]

        self.test_results = {}
        self.completed_tests = 0
        self.passed_tests = 0

        self.get_logger().info('Navigation Test Suite initialized')

    def run_all_tests(self):
        """Run all navigation tests in sequence"""
        self.get_logger().info('Starting navigation test suite...')

        status_msg = String()
        status_msg.data = 'RUNNING'
        self.test_status_pub.publish(status_msg)

        for test in self.tests:
            self.get_logger().info(f'Running test: {test["name"]}')

            # Run the test
            success = self.run_single_test(test)

            # Store result
            self.test_results[test['name']] = success

            if success:
                self.passed_tests += 1
                self.get_logger().info(f'Test {test["name"]} PASSED')
            else:
                self.get_logger().error(f'Test {test["name"]} FAILED')
                # For required tests, we might want to stop the suite
                if test['required']:
                    self.get_logger().error(f'Required test {test["name"]} failed, stopping suite')
                    break

            self.completed_tests += 1

            # Publish test count
            count_msg = Int32()
            count_msg.data = self.completed_tests
            self.test_count_pub.publish(count_msg)

        # Calculate and publish final results
        self.publish_final_results()

    def run_single_test(self, test_config):
        """Run a single test script"""
        try:
            # Construct command to run the test
            cmd = ['python3', test_config['script']]

            # Run the test with timeout
            result = subprocess.run(
                cmd,
                timeout=test_config['timeout'],
                capture_output=True,
                text=True,
                cwd='/path/to/test/scripts'  # Update with actual path
            )

            # Check if test passed based on return code
            success = result.returncode == 0

            # Log test output
            if result.stdout:
                self.get_logger().info(f'Test {test_config["name"]} output: {result.stdout}')
            if result.stderr:
                self.get_logger().error(f'Test {test_config["name"]} error: {result.stderr}')

            return success

        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Test {test_config["name"]} timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Test {test_config["name"]} failed with exception: {e}')
            return False

    def publish_final_results(self):
        """Publish final test suite results"""
        # Calculate overall success rate
        total_required = sum(1 for test in self.tests if test['required'])
        passed_required = sum(1 for test in self.tests
                             if test['required'] and self.test_results.get(test['name'], False))

        overall_success_rate = passed_required / total_required if total_required > 0 else 0

        # Create results summary
        results_summary = {
            'total_tests': len(self.tests),
            'completed_tests': self.completed_tests,
            'passed_tests': self.passed_tests,
            'overall_success_rate': overall_success_rate,
            'test_results': self.test_results
        }

        # Publish results as JSON string
        import json
        results_msg = String()
        results_msg.data = json.dumps(results_summary, indent=2)
        self.test_results_pub.publish(results_msg)

        # Publish final status
        status_msg = String()
        if overall_success_rate >= 1.0:  # All required tests passed
            status_msg.data = 'PASSED'
            self.get_logger().info('Navigation test suite PASSED')
        else:
            status_msg.data = 'FAILED'
            self.get_logger().error('Navigation test suite FAILED')

        self.test_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    test_suite = NavigationTestSuite()

    test_suite.run_all_tests()

    test_suite.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Continuous Integration Testing

#### CI Test Configuration
```yaml
# navigation_ci_tests.yaml
test_suite:
  name: "Navigation CI Tests"
  description: "Continuous integration tests for Nav2 navigation stack"

  tests:
    - name: "Basic Navigation"
      script: "basic_navigation_test.py"
      timeout: 60
      required: true
      parameters:
        - start_pose: [0, 0, 0]
        - goal_pose: [5, 0, 0]
        - tolerance: 0.5

    - name: "Obstacle Avoidance"
      script: "obstacle_avoidance_test.py"
      timeout: 120
      required: true
      parameters:
        - start_pose: [0, 0, 0]
        - goal_pose: [8, 2, 0]
        - obstacle_positions: [[3, 0], [5, 1], [6, -1]]

    - name: "Recovery Behaviors"
      script: "recovery_behavior_test.py"
      timeout: 180
      required: true
      parameters:
        - start_pose: [0, 0, 0]
        - goal_pose: [10, 0, 0]
        - trap_positions: [[5, 0]]  # Position where robot gets trapped

    - name: "Multi-Goal Navigation"
      script: "multi_goal_test.py"
      timeout: 300
      required: false
      parameters:
        - goals: [[2, 0], [4, 2], [6, -1], [8, 0]]
        - tolerance: 0.5

  metrics:
    - name: "Success Rate"
      threshold: 0.95
      weight: 0.4
    - name: "Path Efficiency"
      threshold: 0.7
      weight: 0.3
    - name: "Navigation Time"
      threshold: 60.0  # seconds
      weight: 0.2
    - "Obstacle Detection Rate"
      threshold: 0.9
      weight: 0.1

  environment:
    simulation: "isaac_sim"
    world: "navigation_test_world.usd"
    robot_model: "humanoid_robot.urdf"
    sensors:
      - type: "lidar"
        topic: "/scan"
        range: 10.0
      - type: "depth_camera"
        topic: "/camera/depth/points"
        range: 5.0

  success_criteria:
    overall_threshold: 0.85  # Weighted average of all metrics
    required_tests_pass: true
    no_critical_failures: true
```

## Test Result Analysis and Reporting

### 1. Test Result Analysis

#### Test Result Analyzer
```python
# test_result_analyzer.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
import json
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime

class TestResultAnalyzer(Node):
    def __init__(self):
        super().__init__('test_result_analyzer')

        # Subscriptions
        self.results_sub = self.create_subscription(
            String, '/test_suite/results', self.results_callback, 10)

        # Internal storage
        self.test_results_history = []

        self.get_logger().info('Test Result Analyzer initialized')

    def results_callback(self, msg):
        """Process test results and perform analysis"""
        try:
            results = json.loads(msg.data)
            self.test_results_history.append({
                'timestamp': datetime.now(),
                'results': results
            })

            # Perform analysis
            self.analyze_results(results)

            # Generate reports
            self.generate_report(results)

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in test results')
        except Exception as e:
            self.get_logger().error(f'Error processing test results: {e}')

    def analyze_results(self, results):
        """Analyze test results for trends and patterns"""
        overall_rate = results.get('overall_success_rate', 0)
        test_results = results.get('test_results', {})

        self.get_logger().info(f'Overall success rate: {overall_rate:.2%}')

        # Analyze individual test performance
        for test_name, success in test_results.items():
            status = "PASSED" if success else "FAILED"
            self.get_logger().info(f'Test {test_name}: {status}')

    def generate_report(self, results):
        """Generate a comprehensive test report"""
        report = f"""
Navigation Test Report - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

SUMMARY:
- Total Tests: {results.get('total_tests', 0)}
- Completed: {results.get('completed_tests', 0)}
- Passed: {results.get('passed_tests', 0)}
- Success Rate: {results.get('overall_success_rate', 0):.2%}

INDIVIDUAL TEST RESULTS:
"""
        for test_name, success in results.get('test_results', {}).items():
            status = "PASS" if success else "FAIL"
            report += f"- {test_name}: {status}\n"

        # Save report to file
        filename = f"navigation_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        with open(filename, 'w') as f:
            f.write(report)

        self.get_logger().info(f'Test report saved to {filename}')

    def generate_visualizations(self):
        """Generate visualizations of test results"""
        if not self.test_results_history:
            return

        # Create success rate over time plot
        timestamps = [r['timestamp'] for r in self.test_results_history]
        success_rates = [r['results'].get('overall_success_rate', 0) for r in self.test_results_history]

        plt.figure(figsize=(12, 6))
        plt.plot(timestamps, success_rates, marker='o')
        plt.title('Navigation Test Success Rate Over Time')
        plt.xlabel('Time')
        plt.ylabel('Success Rate')
        plt.grid(True)
        plt.xticks(rotation=45)
        plt.tight_layout()

        plot_filename = f"success_rate_trend_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        plt.savefig(plot_filename)
        plt.close()

        self.get_logger().info(f'Success rate trend plot saved to {plot_filename}')

def main(args=None):
    rclpy.init(args=args)
    analyzer = TestResultAnalyzer()

    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info('Test result analyzer stopped')
        analyzer.generate_visualizations()
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This comprehensive testing procedures guide provides detailed methodologies for testing navigation systems in simulated environments, covering everything from basic functionality tests to complex stress testing and automated test suites.