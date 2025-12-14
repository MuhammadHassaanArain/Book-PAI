# Real-World Testing Procedures for Perception and Navigation

## Overview

This document provides comprehensive procedures for testing perception and navigation systems in real-world environments, ensuring safe and effective sim-to-real transfer validation.

## Pre-Testing Safety Checks

### 1. System Readiness Verification

#### Hardware Verification
```bash
# Check all sensors are publishing data
ros2 topic list | grep -E "(scan|image|imu|camera|depth)"

# Verify sensor data quality
ros2 topic echo /scan --field ranges | head -10
ros2 topic echo /camera/image_raw --field width
ros2 topic echo /imu/data --field angular_velocity
```

#### Emergency Procedures
```bash
# Test emergency stop system
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Verify emergency stop publisher
ros2 topic pub /emergency_stop std_msgs/msg/Bool '{data: true}'
```

### 2. Environment Setup

#### Testing Area Requirements
- **Size**: Minimum 10m x 10m clear area
- **Obstacles**: Various types (static and dynamic)
- **Lighting**: Both indoor and outdoor conditions
- **Safety**: Clear escape routes and safety observer

## Perception Testing Procedures

### 1. Object Detection Validation

#### Test Setup
```bash
# Launch perception pipeline
ros2 launch perception_pipeline.launch.py

# Start test object placement
# Place objects of known dimensions and types
```

#### Validation Process
```python
# perception_validation.py
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np

class PerceptionValidator(Node):
    def __init__(self):
        super().__init__('perception_validator')

        # Subscriptions
        self.detections_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detections_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.accuracy_pub = self.create_publisher(Float32, '/perception/accuracy', 10)
        self.precision_pub = self.create_publisher(Float32, '/perception/precision', 10)
        self.recall_pub = self.create_publisher(Float32, '/perception/recall', 10)

        # Ground truth data
        self.ground_truth = []  # Populated with known objects
        self.detection_history = []

        self.get_logger().info('Perception Validator initialized')

    def detections_callback(self, msg):
        """Process detection results"""
        # Compare detections with ground truth
        self.compare_with_ground_truth(msg.detections)

    def compare_with_ground_truth(self, detections):
        """Compare detections with known ground truth"""
        if not self.ground_truth:
            return

        # Calculate metrics
        tp = 0  # True positives
        fp = 0  # False positives
        fn = 0  # False negatives

        # Simple IoU-based matching
        for det in detections:
            matched = False
            for gt in self.ground_truth:
                if self.calculate_iou(det, gt) > 0.5:  # 50% overlap threshold
                    tp += 1
                    matched = True
                    break
            if not matched:
                fp += 1

        fn = len(self.ground_truth) - tp

        # Calculate precision and recall
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0

        # Publish metrics
        precision_msg = Float32()
        precision_msg.data = float(precision)
        self.precision_pub.publish(precision_msg)

        recall_msg = Float32()
        recall_msg.data = float(recall)
        self.recall_pub.publish(recall_msg)

        self.get_logger().info(f'Perception - Precision: {precision:.3f}, Recall: {recall:.3f}')

    def calculate_iou(self, det, gt):
        """Calculate Intersection over Union between detection and ground truth"""
        # Implementation for IoU calculation
        return 0.0  # Placeholder

def main(args=None):
    rclpy.init(args=args)
    validator = PerceptionValidator()
    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Depth Perception Testing

#### Stereo Vision Validation
```bash
# Test depth accuracy
ros2 run image_view stereo_view stereo:=/camera depth:=/camera/depth
```

#### Depth Accuracy Test
```python
# depth_validator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32
from sensor_msgs_py import point_cloud2
import numpy as np

class DepthValidator(Node):
    def __init__(self):
        super().__init__('depth_validator')

        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10)

        self.depth_accuracy_pub = self.create_publisher(Float32, '/depth/accuracy', 10)

        self.get_logger().info('Depth Validator initialized')

    def pointcloud_callback(self, msg):
        """Validate depth accuracy from point cloud"""
        try:
            points = list(point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            ))

            if len(points) > 0:
                # Calculate depth accuracy statistics
                depths = [p[2] for p in points]  # Z values are depths
                accuracy = self.estimate_depth_accuracy(depths)

                accuracy_msg = Float32()
                accuracy_msg.data = float(accuracy)
                self.depth_accuracy_pub.publish(accuracy_msg)

                self.get_logger().info(f'Depth accuracy: {accuracy:.3f}m')

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def estimate_depth_accuracy(self, depths):
        """Estimate depth accuracy based on point cloud statistics"""
        if len(depths) < 10:
            return 1.0  # Default accuracy if insufficient data

        # Calculate statistics
        mean_depth = np.mean(depths)
        std_depth = np.std(depths)

        # Accuracy metric (lower std indicates better accuracy)
        accuracy = max(0.01, 1.0 / (1.0 + std_depth))  # Inverse relationship
        return accuracy

def main(args=None):
    rclpy.init(args=args)
    validator = DepthValidator()
    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Navigation Testing Procedures

### 1. Path Planning Validation

#### Navigation Test Script
```python
# navigation_tester.py
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool
import time

class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')

        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.success_rate_pub = self.create_publisher(Float32, '/navigation/success_rate', 10)
        self.execution_time_pub = self.create_publisher(Float32, '/navigation/execution_time', 10)
        self.path_efficiency_pub = self.create_publisher(Float32, '/navigation/path_efficiency', 10)

        # Test parameters
        self.test_goals = [
            (2.0, 0.0, 0.0),   # Goal 1: 2m forward
            (2.0, 2.0, 1.57),  # Goal 2: Turn and move
            (0.0, 2.0, 3.14),  # Goal 3: Return
        ]

        self.test_results = []
        self.current_test = 0

        self.get_logger().info('Navigation Tester initialized')

    def run_navigation_tests(self):
        """Run comprehensive navigation tests"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False

        success_count = 0
        total_tests = len(self.test_goals)

        for i, (x, y, theta) in enumerate(self.test_goals):
            self.get_logger().info(f'Running navigation test {i+1}/{total_tests}: ({x}, {y})')

            start_time = time.time()

            # Execute navigation
            success = self.execute_navigation_test(x, y, theta)

            execution_time = time.time() - start_time

            if success:
                success_count += 1
                self.get_logger().info(f'Test {i+1} PASSED')
            else:
                self.get_logger().error(f'Test {i+1} FAILED')

            # Record results
            self.test_results.append({
                'goal': (x, y, theta),
                'success': success,
                'time': execution_time
            })

            # Brief pause between tests
            time.sleep(2.0)

        # Calculate and publish overall metrics
        success_rate = success_count / total_tests if total_tests > 0 else 0
        avg_time = sum([result['time'] for result in self.test_results]) / len(self.test_results) if self.test_results else 0

        rate_msg = Float32()
        rate_msg.data = float(success_rate)
        self.success_rate_pub.publish(rate_msg)

        time_msg = Float32()
        time_msg.data = float(avg_time)
        self.execution_time_pub.publish(time_msg)

        self.get_logger().info(f'Navigation tests completed: {success_rate:.2%} success rate')

        return success_rate >= 0.8  # Require 80% success rate

    def execute_navigation_test(self, x, y, theta):
        """Execute a single navigation test"""
        # Create goal pose
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

        # Send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=120.0)

        if get_result_future.done():
            result = get_result_future.result().result
            status = get_result_future.result().status

            return status == 3  # SUCCEEDED
        else:
            self.get_logger().error('Navigation timed out')
            return False

def main(args=None):
    rclpy.init(args=args)
    tester = NavigationTester()

    success = tester.run_navigation_tests()
    if success:
        tester.get_logger().info('Navigation testing PASSED')
    else:
        tester.get_logger().error('Navigation testing FAILED')

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Obstacle Avoidance Testing

#### Dynamic Obstacle Test
```python
# obstacle_avoidance_tester.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import threading
import time

class ObstacleAvoidanceTester(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_tester')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Test parameters
        self.min_obstacle_distance = 0.5  # meters
        self.safety_active = True

        self.get_logger().info('Obstacle Avoidance Tester initialized')

    def run_obstacle_test(self):
        """Run obstacle avoidance test"""
        # Move forward to approach obstacle
        self.move_forward()

        # Wait for obstacle detection
        while self.safety_active and rclpy.ok():
            time.sleep(0.1)

    def move_forward(self):
        """Move robot forward to test obstacle detection"""
        cmd = Twist()
        cmd.linear.x = 0.2  # Move forward slowly

        while rclpy.ok():
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)

    def scan_callback(self, msg):
        """Monitor for obstacles"""
        if not msg.ranges:
            return

        # Find minimum distance in forward sector
        forward_sector = len(msg.ranges) // 2 - 30, len(msg.ranges) // 2 + 30
        start_idx, end_idx = forward_sector

        if 0 <= start_idx < len(msg.ranges) and 0 <= end_idx < len(msg.ranges):
            forward_ranges = msg.ranges[start_idx:end_idx]
            valid_ranges = [r for r in forward_ranges if not (r != r or r > msg.range_max)]

            if valid_ranges:
                min_dist = min(valid_ranges)

                if min_dist < self.min_obstacle_distance:
                    self.get_logger().warn(f'Obstacle detected: {min_dist:.2f}m < {self.min_obstacle_distance:.2f}m')

                    # Stop robot
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                    self.safety_active = False

                    # Resume after delay
                    time.sleep(2.0)
                    self.safety_active = True

def main(args=None):
    rclpy.init(args=args)
    tester = ObstacleAvoidanceTester()

    # Run test in separate thread to allow callbacks
    test_thread = threading.Thread(target=tester.run_obstacle_test)
    test_thread.start()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Obstacle avoidance test stopped')
    finally:
        # Stop robot on shutdown
        stop_cmd = Twist()
        tester.cmd_vel_pub.publish(stop_cmd)
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Metrics Collection

### 1. Real-Time Performance Monitoring

#### Performance Monitor
```python
# performance_monitor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
from collections import deque

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.fps_pub = self.create_publisher(Float32, '/performance/fps', 10)
        self.cpu_usage_pub = self.create_publisher(Float32, '/performance/cpu', 10)
        self.memory_usage_pub = self.create_publisher(Float32, '/performance/memory', 10)

        # Performance tracking
        self.frame_times = deque(maxlen=30)  # Last 30 frames
        self.processing_times = deque(maxlen=30)

        # Timer for performance reporting
        self.perf_timer = self.create_timer(1.0, self.report_performance)

        self.get_logger().info('Performance Monitor initialized')

    def image_callback(self, msg):
        """Track image processing performance"""
        current_time = time.time()

        if hasattr(self, 'last_frame_time'):
            frame_time = current_time - self.last_frame_time
            self.frame_times.append(frame_time)

        self.last_frame_time = current_time

    def scan_callback(self, msg):
        """Track LiDAR processing performance"""
        # Similar tracking for LiDAR data
        pass

    def cmd_vel_callback(self, msg):
        """Track command processing performance"""
        # Track command processing time
        pass

    def report_performance(self):
        """Report current performance metrics"""
        if self.frame_times:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0

            fps_msg = Float32()
            fps_msg.data = float(fps)
            self.fps_pub.publish(fps_msg)

            self.get_logger().info(f'Performance - FPS: {fps:.2f}')

def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This testing procedures guide provides comprehensive validation methods for real-world perception and navigation systems.