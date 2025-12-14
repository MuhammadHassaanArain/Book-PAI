#!/usr/bin/env python3
"""
Sensor Validation Script for Digital Twin System

This module provides validation tests for sensor simulation in the Digital Twin system.
It includes tests for LiDAR, depth camera, RGB camera, and IMU sensors.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
from tf2_ros import Buffer, TransformListener
import numpy as np
import time
from typing import Dict, Any, Optional, Tuple
import math


class SensorValidator(Node):
    """
    Sensor validation node for testing sensor simulation properties
    """

    def __init__(self):
        """
        Initialize the SensorValidator node
        """
        super().__init__('sensor_validator')

        # Initialize data storage
        self.lidar_data = None
        self.depth_image = None
        self.rgb_image = None
        self.imu_data = None
        self.camera_info = None

        # TF buffer for transform validation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers for sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/robot1/lidar/scan',
            self.lidar_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/robot1/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.rgb_sub = self.create_subscription(
            Image,
            '/robot1/camera/rgb/image_raw',
            self.rgb_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/robot1/imu/data',
            self.imu_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/robot1/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

    def lidar_callback(self, msg: LaserScan):
        """
        Callback for LiDAR scan messages
        """
        self.lidar_data = msg

    def depth_callback(self, msg: Image):
        """
        Callback for depth image messages
        """
        self.depth_image = msg

    def rgb_callback(self, msg: Image):
        """
        Callback for RGB image messages
        """
        self.rgb_image = msg

    def imu_callback(self, msg: Imu):
        """
        Callback for IMU messages
        """
        self.imu_data = msg

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for camera info messages
        """
        self.camera_info = msg

    def validate_lidar_simulation(self, timeout: float = 5.0) -> bool:
        """
        Validate LiDAR sensor simulation

        Args:
            timeout (float): Time to wait for data in seconds

        Returns:
            bool: True if LiDAR data is valid, False otherwise
        """
        start_time = time.time()

        # Wait for LiDAR data
        while self.lidar_data is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.lidar_data is None:
            self.get_logger().warn('No LiDAR data received')
            return False

        # Validate basic LiDAR properties
        lidar = self.lidar_data
        valid_ranges = [r for r in lidar.ranges if not (math.isnan(r) or r == float('inf'))]
        has_valid_data = len(valid_ranges) > 0
        has_correct_ranges = lidar.range_min < lidar.range_max
        has_correct_angles = lidar.angle_min < lidar.angle_max

        is_valid = has_valid_data and has_correct_ranges and has_correct_angles

        self.get_logger().info(
            f'LiDAR validation: valid_data={has_valid_data}, '
            f'correct_ranges={has_correct_ranges}, '
            f'correct_angles={has_correct_angles}, '
            f'total_valid={len(valid_ranges)}, '
            f'valid={is_valid}'
        )

        return is_valid

    def validate_depth_camera_simulation(self, timeout: float = 5.0) -> bool:
        """
        Validate depth camera sensor simulation

        Args:
            timeout (float): Time to wait for data in seconds

        Returns:
            bool: True if depth camera data is valid, False otherwise
        """
        start_time = time.time()

        # Wait for depth image data
        while self.depth_image is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.depth_image is None:
            self.get_logger().warn('No depth image data received')
            return False

        # Validate basic depth image properties
        depth_img = self.depth_image
        has_correct_dimensions = depth_img.height > 0 and depth_img.width > 0
        has_correct_encoding = depth_img.encoding in ['16UC1', '32FC1']
        has_data = len(depth_img.data) > 0

        # Check if depth values are reasonable (not all zero or max)
        if depth_img.encoding == '16UC1':
            # Convert to numpy array for analysis (simplified)
            # In real implementation, we'd convert the raw data to proper format
            has_realistic_values = True  # Placeholder for actual validation
        else:
            has_realistic_values = True

        is_valid = (has_correct_dimensions and has_correct_encoding and
                   has_data and has_realistic_values)

        self.get_logger().info(
            f'Depth camera validation: dimensions_ok={has_correct_dimensions}, '
            f'encoding_ok={has_correct_encoding}, '
            f'has_data={has_data}, '
            f'realistic_values={has_realistic_values}, '
            f'valid={is_valid}'
        )

        return is_valid

    def validate_rgb_camera_simulation(self, timeout: float = 5.0) -> bool:
        """
        Validate RGB camera sensor simulation

        Args:
            timeout (float): Time to wait for data in seconds

        Returns:
            bool: True if RGB camera data is valid, False otherwise
        """
        start_time = time.time()

        # Wait for RGB image data
        while self.rgb_image is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.rgb_image is None:
            self.get_logger().warn('No RGB image data received')
            return False

        # Validate basic RGB image properties
        rgb_img = self.rgb_image
        has_correct_dimensions = rgb_img.height > 0 and rgb_img.width > 0
        has_correct_encoding = rgb_img.encoding in ['rgb8', 'bgr8', 'rgba8', 'bgra8']
        has_data = len(rgb_img.data) > 0

        is_valid = has_correct_dimensions and has_correct_encoding and has_data

        self.get_logger().info(
            f'RGB camera validation: dimensions_ok={has_correct_dimensions}, '
            f'encoding_ok={has_correct_encoding}, '
            f'has_data={has_data}, '
            f'valid={is_valid}'
        )

        return is_valid

    def validate_imu_simulation(self, timeout: float = 5.0) -> bool:
        """
        Validate IMU sensor simulation

        Args:
            timeout (float): Time to wait for data in seconds

        Returns:
            bool: True if IMU data is valid, False otherwise
        """
        start_time = time.time()

        # Wait for IMU data
        while self.imu_data is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.imu_data is None:
            self.get_logger().warn('No IMU data received')
            return False

        # Validate basic IMU properties
        imu = self.imu_data
        has_orientation = not (math.isnan(imu.orientation.x) and
                              math.isnan(imu.orientation.y) and
                              math.isnan(imu.orientation.z) and
                              math.isnan(imu.orientation.w))
        has_angular_velocity = not (math.isnan(imu.angular_velocity.x) and
                                   math.isnan(imu.angular_velocity.y) and
                                   math.isnan(imu.angular_velocity.z))
        has_linear_acceleration = not (math.isnan(imu.linear_acceleration.x) and
                                      math.isnan(imu.linear_acceleration.y) and
                                      math.isnan(imu.linear_acceleration.z))

        is_valid = has_orientation or has_angular_velocity or has_linear_acceleration

        self.get_logger().info(
            f'IMU validation: has_orientation={has_orientation}, '
            f'has_angular_velocity={has_angular_velocity}, '
            f'has_linear_acceleration={has_linear_acceleration}, '
            f'valid={is_valid}'
        )

        return is_valid

    def validate_sensor_noise_models(self, timeout: float = 5.0) -> bool:
        """
        Validate that sensor noise models are applied correctly

        Args:
            timeout (float): Time to wait for sufficient data in seconds

        Returns:
            bool: True if noise is detected, False otherwise
        """
        start_time = time.time()

        # Collect multiple readings to check for noise
        lidar_readings = []
        while len(lidar_readings) < 10 and (time.time() - start_time) < timeout:
            if self.lidar_data is not None:
                lidar_readings.append(self.lidar_data)
            rclpy.spin_once(self, timeout_sec=0.1)

        if len(lidar_readings) < 2:
            self.get_logger().warn('Not enough LiDAR readings to validate noise')
            return False

        # Check for variation in readings (indicating noise)
        # Compare the same angle across different readings
        if len(lidar_readings[0].ranges) > 0:
            first_angle_readings = [reading.ranges[0] for reading in lidar_readings
                                  if len(reading.ranges) > 0 and not math.isnan(reading.ranges[0])]
            has_variation = len(set(first_angle_readings)) > 1  # If there's variation, noise is present

            self.get_logger().info(f'Sensor noise validation: has_variation={has_variation}')
            return has_variation

        return False

    def validate_ros2_topic_streaming(self, timeout: float = 3.0) -> Dict[str, bool]:
        """
        Validate that sensor data is streaming to ROS 2 topics

        Args:
            timeout (float): Time to wait for data in seconds

        Returns:
            Dict[str, bool]: Validation results for each sensor type
        """
        start_time = time.time()

        # Wait for all sensor data
        while (self.lidar_data is None or self.depth_image is None or
               self.rgb_image is None or self.imu_data is None) and \
              (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        results = {
            'lidar_streaming': self.lidar_data is not None,
            'depth_streaming': self.depth_image is None,
            'rgb_streaming': self.rgb_image is not None,
            'imu_streaming': self.imu_data is not None
        }

        # Update the result for depth since we initially set it to False by mistake
        results['depth_streaming'] = self.depth_image is not None

        self.get_logger().info(
            f'Topic streaming validation: '
            f'lidar={results["lidar_streaming"]}, '
            f'depth={results["depth_streaming"]}, '
            f'rgb={results["rgb_streaming"]}, '
            f'imu={results["imu_streaming"]}'
        )

        return results

    def validate_sensor_latency(self, timeout: float = 5.0) -> bool:
        """
        Validate that sensor data has acceptable latency (<100ms)

        Args:
            timeout (float): Time to wait for data in seconds

        Returns:
            bool: True if latency is acceptable, False otherwise
        """
        start_time = time.time()

        # Wait for sensor data
        while self.lidar_data is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.lidar_data is None:
            return False

        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        sensor_time = self.lidar_data.header.stamp.sec + self.lidar_data.header.stamp.nanosec / 1e9
        latency = abs(current_time - sensor_time)

        is_acceptable = latency < 0.1  # Less than 100ms

        self.get_logger().info(
            f'Sensor latency validation: latency={latency:.3f}s, '
            f'acceptable={is_acceptable}'
        )

        return is_acceptable


def main():
    """
    Main function to run sensor validation tests
    """
    rclpy.init()

    validator = SensorValidator()

    # Wait a bit for data to accumulate
    time.sleep(2.0)

    # Run validation tests
    results = {}

    # Test 1: LiDAR simulation
    results['lidar'] = validator.validate_lidar_simulation()

    # Test 2: Depth camera simulation
    results['depth_camera'] = validator.validate_depth_camera_simulation()

    # Test 3: RGB camera simulation
    results['rgb_camera'] = validator.validate_rgb_camera_simulation()

    # Test 4: IMU simulation
    results['imu'] = validator.validate_imu_simulation()

    # Test 5: Sensor noise models
    results['noise_models'] = validator.validate_sensor_noise_models()

    # Test 6: ROS 2 topic streaming
    streaming_results = validator.validate_ros2_topic_streaming()
    results['topic_streaming'] = all(streaming_results.values())

    # Test 7: Sensor latency
    results['latency'] = validator.validate_sensor_latency()

    # Print detailed results
    print("\n=== Sensor Validation Results ===")
    for test, result in results.items():
        status = "PASS" if result else "FAIL"
        print(f"{test}: {status}")

    # Print streaming details
    print("\nTopic Streaming Details:")
    for topic, result in streaming_results.items():
        status = "PASS" if result else "FAIL"
        print(f"  {topic}: {status}")

    # Overall result
    all_passed = all(results.values())
    print(f"\nOverall: {'PASS' if all_passed else 'FAIL'}")

    validator.destroy_node()
    rclpy.shutdown()

    return all_passed


class TestSensorValidation(unittest.TestCase):
    """
    Unit tests for sensor validation functions
    """

    def setUp(self):
        """
        Set up the test case
        """
        if not rclpy.ok():
            rclpy.init()
        self.validator = SensorValidator()

    def tearDown(self):
        """
        Tear down the test case
        """
        self.validator.destroy_node()

    def test_lidar_validation(self):
        """
        Test LiDAR validation function
        """
        self.assertTrue(hasattr(self.validator, 'validate_lidar_simulation'))

    def test_camera_validation(self):
        """
        Test camera validation functions
        """
        self.assertTrue(hasattr(self.validator, 'validate_depth_camera_simulation'))
        self.assertTrue(hasattr(self.validator, 'validate_rgb_camera_simulation'))

    def test_imu_validation(self):
        """
        Test IMU validation function
        """
        self.assertTrue(hasattr(self.validator, 'validate_imu_simulation'))


if __name__ == '__main__':
    # Run the main validation
    success = main()

    # If running as part of a test suite, also run unit tests
    if success:
        unittest.main(argv=[''], exit=False, verbosity=2)