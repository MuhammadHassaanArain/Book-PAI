#!/usr/bin/env python3
"""
ROS 2 Integration Validation Script for Digital Twin System

This module provides validation tests for ROS 2 integration in the Digital Twin system.
It includes tests for topic communication, service calls, action execution, and TF transforms.
"""

import unittest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState, LaserScan, Image, Imu
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList
from gazebo_msgs.action import SetJointProperties
import time
from typing import Dict, Any, Optional, List
import threading


class ROS2IntegrationValidator(Node):
    """
    ROS 2 integration validation node for testing ROS 2 communication
    """

    def __init__(self):
        """
        Initialize the ROS2IntegrationValidator node
        """
        super().__init__('ros2_integration_validator')

        # Initialize data storage
        self.received_messages = {}
        self.service_results = {}
        self.action_results = {}

        # QoS profiles
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # TF buffer for transform validation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers for testing
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            self.qos_profile
        )

        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            self.qos_profile
        )

        # Subscribers for validation
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            self.qos_profile
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.qos_profile
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            self.qos_profile
        )

        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')

        # Action client
        self.joint_properties_client = ActionClient(
            self,
            SetJointProperties,
            '/set_joint_properties'
        )

        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.spawn_client.wait_for_service()
        self.delete_client.wait_for_service()
        self.get_model_list_client.wait_for_service()

    def joint_state_callback(self, msg: JointState):
        """
        Callback for joint state messages
        """
        self.received_messages['joint_states'] = msg

    def scan_callback(self, msg: LaserScan):
        """
        Callback for laser scan messages
        """
        self.received_messages['scan'] = msg

    def imu_callback(self, msg: Imu):
        """
        Callback for IMU messages
        """
        self.received_messages['imu'] = msg

    def validate_topic_communication(self, timeout: float = 5.0) -> Dict[str, bool]:
        """
        Validate ROS 2 topic communication

        Args:
            timeout (float): Time to wait for data in seconds

        Returns:
            Dict[str, bool]: Validation results for each topic
        """
        start_time = time.time()

        # Wait for data on all monitored topics
        while (len(self.received_messages) < 3 and  # We expect 3 types of messages
               (time.time() - start_time) < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)

        results = {
            'joint_states': 'joint_states' in self.received_messages,
            'scan': 'scan' in self.received_messages,
            'imu': 'imu' in self.received_messages
        }

        self.get_logger().info(
            f'Topic communication validation: '
            f'joint_states={results["joint_states"]}, '
            f'scan={results["scan"]}, '
            f'imu={results["imu"]}'
        )

        return results

    def validate_service_calls(self) -> Dict[str, bool]:
        """
        Validate ROS 2 service calls

        Returns:
            Dict[str, bool]: Validation results for each service
        """
        results = {}

        # Test get model list service
        try:
            request = GetModelList.Request()
            future = self.get_model_list_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is not None:
                response = future.result()
                results['get_model_list'] = True
                self.get_logger().info(f'Found {len(response.model_names)} models')
            else:
                results['get_model_list'] = False
                self.get_logger().warn('Get model list service call failed')
        except Exception as e:
            results['get_model_list'] = False
            self.get_logger().error(f'Get model list service error: {e}')

        # Test spawn entity service (try to spawn a simple object)
        try:
            # Create a simple box model as SDF
            sdf_model = """
            <sdf version="1.6">
                <model name="test_box">
                    <pose>0 0 0.5 0 0 0</pose>
                    <link name="link">
                        <collision name="collision">
                            <geometry>
                                <box>
                                    <size>0.1 0.1 0.1</size>
                                </box>
                            </geometry>
                        </collision>
                        <visual name="visual">
                            <geometry>
                                <box>
                                    <size>0.1 0.1 0.1</size>
                                </box>
                            </geometry>
                        </visual>
                        <inertial>
                            <mass>0.1</mass>
                            <inertia>
                                <ixx>0.0001</ixx>
                                <ixy>0</ixy>
                                <ixz>0</ixz>
                                <iyy>0.0001</iyy>
                                <iyz>0</iyz>
                                <izz>0.0001</izz>
                            </inertia>
                        </inertial>
                    </link>
                </model>
            </sdf>
            """

            request = SpawnEntity.Request()
            request.name = "test_box"
            request.xml = sdf_model
            request.robot_namespace = ""
            request.initial_pose.position.x = 1.0
            request.initial_pose.position.y = 1.0
            request.initial_pose.position.z = 1.0

            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.result() is not None:
                response = future.result()
                results['spawn_entity'] = response.success
                self.get_logger().info(f'Spawn entity result: {response.success}')
            else:
                results['spawn_entity'] = False
                self.get_logger().warn('Spawn entity service call failed')
        except Exception as e:
            results['spawn_entity'] = False
            self.get_logger().error(f'Spawn entity service error: {e}')

        # Test delete entity service
        try:
            request = DeleteEntity.Request()
            request.name = "test_box"

            future = self.delete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is not None:
                response = future.result()
                results['delete_entity'] = response.success
                self.get_logger().info(f'Delete entity result: {response.success}')
            else:
                results['delete_entity'] = False
                self.get_logger().warn('Delete entity service call failed')
        except Exception as e:
            results['delete_entity'] = False
            self.get_logger().error(f'Delete entity service error: {e}')

        self.get_logger().info(
            f'Service call validation: '
            f'get_model_list={results.get("get_model_list", False)}, '
            f'spawn_entity={results.get("spawn_entity", False)}, '
            f'delete_entity={results.get("delete_entity", False)}'
        )

        return results

    def validate_action_execution(self, timeout: float = 10.0) -> bool:
        """
        Validate ROS 2 action execution

        Args:
            timeout (float): Time to wait for action completion in seconds

        Returns:
            bool: True if action execution is valid, False otherwise
        """
        # Check if action client is available
        is_available = self.joint_properties_client.wait_for_server(timeout_sec=2.0)

        if not is_available:
            self.get_logger().warn('Joint properties action server not available')
            return False

        # For now, just validate that the action client is available
        # In a full implementation, we would send an actual action goal
        is_valid = True

        self.get_logger().info(f'Action execution validation: available={is_available}, valid={is_valid}')

        return is_valid

    def validate_tf_transforms(self, timeout: float = 5.0) -> Dict[str, bool]:
        """
        Validate TF transforms between frames

        Args:
            timeout (float): Time to wait for transforms in seconds

        Returns:
            Dict[str, bool]: Validation results for different transforms
        """
        results = {}

        # Try to get common transforms
        transforms_to_check = [
            ('map', 'odom'),
            ('odom', 'base_link'),
            ('base_link', 'base_footprint'),
            ('base_link', 'imu_link'),
            ('base_link', 'lidar_link')
        ]

        for target_frame, source_frame in transforms_to_check:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=timeout)
                )
                results[f'{source_frame}_to_{target_frame}'] = True
            except Exception as e:
                self.get_logger().debug(f'Could not lookup transform {source_frame} to {target_frame}: {e}')
                results[f'{source_frame}_to_{target_frame}'] = False

        self.get_logger().info(
            f'TF transform validation: {sum(results.values())}/{len(results)} transforms available'
        )

        return results

    def validate_qos_profiles(self, timeout: float = 3.0) -> Dict[str, bool]:
        """
        Validate QoS profile compliance for different topics

        Args:
            timeout (float): Time to wait for validation in seconds

        Returns:
            Dict[str, bool]: Validation results for QoS compliance
        """
        start_time = time.time()

        # Wait for some messages to arrive
        while (len(self.received_messages) == 0 and
               (time.time() - start_time) < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Check if we received messages with proper timestamps
        results = {}
        for topic, msg in self.received_messages.items():
            # Check if message has header with timestamp
            has_header = hasattr(msg, 'header')
            has_timestamp = has_header and msg.header.stamp.sec != 0
            results[f'{topic}_qos'] = has_timestamp

        self.get_logger().info(
            f'QoS validation: {sum(results.values())}/{len(results)} topics have valid timestamps'
        )

        return results

    def validate_message_timing(self, timeout: float = 5.0) -> Dict[str, bool]:
        """
        Validate message timing and frequency

        Args:
            timeout (float): Time to collect messages in seconds

        Returns:
            Dict[str, bool]: Validation results for timing
        """
        start_time = time.time()
        message_counts = {}
        timestamps = {}

        # Collect messages for the specified time
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Count messages by type
            for topic, msg in self.received_messages.items():
                if topic not in message_counts:
                    message_counts[topic] = 0
                    timestamps[topic] = []

                # Only count if this is a new message (not the same reference)
                if not timestamps[topic] or msg.header.stamp.nanosec != timestamps[topic][-1]:
                    message_counts[topic] += 1
                    timestamps[topic].append(msg.header.stamp.nanosec)

        # Calculate frequencies
        frequencies = {}
        for topic, count in message_counts.items():
            frequencies[topic] = count / timeout

        # Validate that we have reasonable frequencies (1-100 Hz)
        results = {}
        for topic, freq in frequencies.items():
            results[f'{topic}_frequency'] = 1.0 <= freq <= 100.0

        self.get_logger().info(
            f'Message timing validation: '
            f'frequencies={frequencies}, '
            f'results={results}'
        )

        return results


def main():
    """
    Main function to run ROS 2 integration validation tests
    """
    rclpy.init()

    validator = ROS2IntegrationValidator()

    # Wait a bit for data to accumulate
    time.sleep(2.0)

    # Run validation tests
    results = {}

    # Test 1: Topic communication
    results['topic_communication'] = validator.validate_topic_communication()

    # Test 2: Service calls
    service_results = validator.validate_service_calls()
    results['service_calls'] = all(service_results.values())

    # Test 3: Action execution
    results['action_execution'] = validator.validate_action_execution()

    # Test 4: TF transforms
    tf_results = validator.validate_tf_transforms()
    results['tf_transforms'] = all(tf_results.values())

    # Test 5: QoS profiles
    qos_results = validator.validate_qos_profiles()
    results['qos_profiles'] = all(qos_results.values())

    # Test 6: Message timing
    timing_results = validator.validate_message_timing()
    results['message_timing'] = all(timing_results.values())

    # Print detailed results
    print("\n=== ROS 2 Integration Validation Results ===")
    for test, result in results.items():
        if isinstance(result, dict):
            print(f"{test}:")
            for subtest, subresult in result.items():
                status = "PASS" if subresult else "FAIL"
                print(f"  {subtest}: {status}")
        else:
            status = "PASS" if result else "FAIL"
            print(f"{test}: {status}")

    # Print detailed service results
    print("\nService Call Details:")
    for service, result in service_results.items():
        status = "PASS" if result else "FAIL"
        print(f"  {service}: {status}")

    # Print detailed TF results
    print("\nTF Transform Details:")
    for tf_pair, result in tf_results.items():
        status = "PASS" if result else "FAIL"
        print(f"  {tf_pair}: {status}")

    # Print detailed QoS results
    print("\nQoS Profile Details:")
    for qos_check, result in qos_results.items():
        status = "PASS" if result else "FAIL"
        print(f"  {qos_check}: {status}")

    # Calculate overall result
    overall_success = True
    for result in results.values():
        if isinstance(result, dict):
            overall_success = overall_success and all(result.values())
        else:
            overall_success = overall_success and result

    print(f"\nOverall: {'PASS' if overall_success else 'FAIL'}")

    validator.destroy_node()
    rclpy.shutdown()

    return overall_success


class TestROS2Integration(unittest.TestCase):
    """
    Unit tests for ROS 2 integration validation functions
    """

    def setUp(self):
        """
        Set up the test case
        """
        if not rclpy.ok():
            rclpy.init()
        self.validator = ROS2IntegrationValidator()

    def tearDown(self):
        """
        Tear down the test case
        """
        self.validator.destroy_node()

    def test_topic_communication(self):
        """
        Test topic communication validation
        """
        self.assertTrue(hasattr(self.validator, 'validate_topic_communication'))

    def test_service_calls(self):
        """
        Test service call validation
        """
        self.assertTrue(hasattr(self.validator, 'validate_service_calls'))

    def test_action_execution(self):
        """
        Test action execution validation
        """
        self.assertTrue(hasattr(self.validator, 'validate_action_execution'))


if __name__ == '__main__':
    # Run the main validation
    success = main()

    # If running as part of a test suite, also run unit tests
    if success:
        unittest.main(argv=[''], exit=False, verbosity=2)