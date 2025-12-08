#!/usr/bin/env python3
"""
Verification script for stable humanoid contact dynamics in Gazebo simulation

This script tests that the humanoid robot maintains stable contact with the ground
with <5% deviation from expected behavior.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64
import time
import numpy as np
import math
from typing import Dict, List, Tuple


class HumanoidContactVerification(Node):
    """Node for verifying stable humanoid contact dynamics"""

    def __init__(self):
        """Initialize the verification node"""
        super().__init__('humanoid_contact_verification')

        # Service clients
        self.get_physics_client = self.create_client(
            GetPhysicsProperties,
            '/gazebo/get_physics_properties'
        )
        self.set_physics_client = self.create_client(
            SetPhysicsProperties,
            '/gazebo/set_physics_properties'
        )

        # Wait for services
        while not self.get_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_physics_properties service...')
        while not self.set_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_physics_properties service...')

        # Subscribers for monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.contact_state_sub = self.create_subscription(
            ContactsState,
            '/gazebo/contact_states',
            self.contact_state_callback,
            10
        )

        # Storage for data
        self.joint_states_history = []
        self.contact_states_history = []
        self.foot_positions_history = []

        # Robot-specific joint names (from our humanoid model)
        self.humanoid_joints = [
            'right_shoulder_joint', 'right_elbow_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint'
        ]

        # Timer for data collection
        self.timer = self.create_timer(0.1, self.collect_data)  # 10 Hz
        self.start_time = time.time()

    def joint_state_callback(self, msg: JointState):
        """Callback for joint state messages"""
        self.current_joint_state = msg

    def contact_state_callback(self, msg: ContactsState):
        """Callback for contact state messages"""
        self.current_contact_state = msg

    def collect_data(self):
        """Collect data for analysis"""
        current_time = time.time()

        if hasattr(self, 'current_joint_state'):
            self.joint_states_history.append((current_time, self.current_joint_state))

        if hasattr(self, 'current_contact_state'):
            self.contact_states_history.append((current_time, self.current_contact_state))

    def get_current_physics_properties(self):
        """Get current physics properties"""
        request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        return None

    def verify_contact_stability(self, duration: float = 10.0) -> Tuple[bool, Dict]:
        """
        Verify that humanoid maintains stable contact with <5% deviation

        Args:
            duration: Duration to test in seconds

        Returns:
            Tuple of (success, metrics)
        """
        self.get_logger().info(f'Starting contact stability verification for {duration} seconds...')

        start_time = time.time()
        initial_time = start_time

        # Store initial joint positions to compare against
        initial_joint_positions = {}
        position_deviation_history = []

        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

            if hasattr(self, 'current_joint_state'):
                # Calculate deviation from initial positions
                if not initial_joint_positions:
                    # Store initial positions
                    for i, name in enumerate(self.current_joint_state.name):
                        if name in self.humanoid_joints:
                            initial_joint_positions[name] = self.current_joint_state.position[i]

                # Calculate current deviation
                current_deviation = 0.0
                deviation_count = 0

                for i, name in enumerate(self.current_joint_state.name):
                    if name in initial_joint_positions:
                        deviation = abs(self.current_joint_state.position[i] - initial_joint_positions[name])
                        current_deviation += deviation
                        deviation_count += 1

                if deviation_count > 0:
                    avg_deviation = current_deviation / deviation_count
                    position_deviation_history.append(avg_deviation)

        # Calculate statistics
        if position_deviation_history:
            max_deviation = max(position_deviation_history)
            avg_deviation = sum(position_deviation_history) / len(position_deviation_history)
            std_deviation = np.std(position_deviation_history)

            # Calculate percentage deviation (assuming typical range of motion)
            # Using 1.57 radians (90 degrees) as a reference for "typical" joint range
            max_deviation_percentage = (max_deviation / 1.57) * 100
            avg_deviation_percentage = (avg_deviation / 1.57) * 100

            metrics = {
                'max_deviation': max_deviation,
                'avg_deviation': avg_deviation,
                'std_deviation': std_deviation,
                'max_deviation_percentage': max_deviation_percentage,
                'avg_deviation_percentage': avg_deviation_percentage,
                'deviation_samples': len(position_deviation_history)
            }

            # Check if max deviation is less than 5%
            success = max_deviation_percentage < 5.0

            return success, metrics
        else:
            return False, {'error': 'No joint data collected'}

    def verify_ground_contact(self, duration: float = 10.0) -> Tuple[bool, Dict]:
        """
        Verify that the humanoid maintains ground contact

        Args:
            duration: Duration to test in seconds

        Returns:
            Tuple of (success, metrics)
        """
        self.get_logger().info(f'Starting ground contact verification for {duration} seconds...')

        contact_count = 0
        total_samples = 0

        start_time = time.time()

        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

            if hasattr(self, 'current_contact_state'):
                # Check if there are contacts with the ground plane
                ground_contacts = 0
                for contact in self.current_contact_state.states:
                    if 'ground_plane' in contact.collision1_name or 'ground_plane' in contact.collision2_name:
                        ground_contacts += 1

                if ground_contacts > 0:
                    contact_count += 1
                total_samples += 1

        if total_samples > 0:
            contact_ratio = contact_count / total_samples
            metrics = {
                'contact_ratio': contact_ratio,
                'total_samples': total_samples,
                'contact_samples': contact_count
            }

            # Require at least 95% of samples to have ground contact
            success = contact_ratio >= 0.95
            return success, metrics
        else:
            return False, {'error': 'No contact data collected'}


def main():
    """Main function to run humanoid contact verification"""
    rclpy.init()

    verifier = HumanoidContactVerification()

    time.sleep(2.0)  # Allow some time for data to accumulate

    # Run contact stability verification
    stability_success, stability_metrics = verifier.verify_contact_stability(duration=10.0)

    # Run ground contact verification
    contact_success, contact_metrics = verifier.verify_ground_contact(duration=10.0)

    # Print results
    print("\n=== Humanoid Contact Dynamics Verification Results ===")
    print(f"Stability Test: {'PASS' if stability_success else 'FAIL'}")
    print(f"  Max deviation: {stability_metrics.get('max_deviation', 'N/A'):.4f} rad ({stability_metrics.get('max_deviation_percentage', 'N/A'):.2f}%)")
    print(f"  Avg deviation: {stability_metrics.get('avg_deviation', 'N/A'):.4f} rad ({stability_metrics.get('avg_deviation_percentage', 'N/A'):.2f}%)")
    print(f"  Std deviation: {stability_metrics.get('std_deviation', 'N/A'):.4f}")

    print(f"Ground Contact Test: {'PASS' if contact_success else 'FAIL'}")
    print(f"  Contact ratio: {contact_metrics.get('contact_ratio', 'N/A'):.2f}")

    overall_success = stability_success and contact_success
    print(f"\nOverall Verification: {'PASS' if overall_success else 'FAIL'}")

    verifier.destroy_node()
    rclpy.shutdown()

    return overall_success


if __name__ == '__main__':
    success = main()
    exit(0 if success else 1)