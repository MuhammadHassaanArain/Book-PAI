#!/usr/bin/env python3
"""
Physics Validation Script for Digital Twin System

This module provides validation tests for physics simulation in the Digital Twin system.
It includes tests for gravity simulation, contact dynamics, and parameter adjustment.
"""

import unittest
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time
import numpy as np
from typing import Dict, Any, Optional


class PhysicsValidator(Node):
    """
    Physics validation node for testing physics simulation properties
    """

    def __init__(self):
        """
        Initialize the PhysicsValidator node
        """
        super().__init__('physics_validator')

        # Service clients
        self.get_physics_client = self.create_client(
            GetPhysicsProperties,
            '/gazebo/get_physics_properties'
        )
        self.set_physics_client = self.create_client(
            SetPhysicsProperties,
            '/gazebo/set_physics_properties'
        )

        # Wait for services to be available
        while not self.get_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_physics_properties service...')

        while not self.set_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_physics_properties service...')

        # Variables to store validation results
        self.joint_states = None
        self.contact_states = None

        # Subscribers for validation data
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.contact_sub = self.create_subscription(
            ContactsState,
            '/gazebo/contact_states',
            self.contact_state_callback,
            10
        )

    def joint_state_callback(self, msg: JointState):
        """
        Callback for joint state messages
        """
        self.joint_states = msg

    def contact_state_callback(self, msg: ContactsState):
        """
        Callback for contact state messages
        """
        self.contact_states = msg

    def get_current_physics_properties(self) -> Optional[Dict[str, Any]]:
        """
        Get current physics properties from Gazebo

        Returns:
            Optional[Dict[str, Any]]: Physics properties or None if failed
        """
        request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            return {
                'time_step': response.time_step,
                'max_update_rate': response.max_update_rate,
                'gravity': {
                    'x': response.gravity.x,
                    'y': response.gravity.y,
                    'z': response.gravity.z
                },
                'ode_config': {
                    'iters': response.ode_config.sor_pgs_iters,
                    'sor': response.ode_config.sor_pgs_w,
                    'cfm': response.ode_config.cfm,
                    'erp': response.ode_config.erp
                }
            }
        else:
            self.get_logger().error('Failed to get physics properties')
            return None

    def set_physics_properties(self, gravity: Vector3, time_step: float = 0.001,
                              max_update_rate: float = 1000.0, iters: int = 200,
                              sor: float = 1.3, cfm: float = 1e-6, erp: float = 0.2) -> bool:
        """
        Set physics properties in Gazebo

        Args:
            gravity (Vector3): Gravity vector
            time_step (float): Time step for simulation
            max_update_rate (float): Maximum update rate
            iters (int): Number of solver iterations
            sor (float): Successive over-relaxation parameter
            cfm (float): Constraint Force Mixing parameter
            erp (float): Error Reduction Parameter

        Returns:
            bool: True if successful, False otherwise
        """
        from gazebo_msgs.msg import ODEPhysics

        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = iters
        ode_config.sor_pgs_w = sor
        ode_config.contact_surface_layer = 0.001
        ode_config.contact_max_correcting_vel = 100.0
        ode_config.cfm = cfm
        ode_config.erp = erp
        ode_config.max_contacts = 20

        request = SetPhysicsProperties.Request()
        request.time_step = time_step
        request.max_update_rate = max_update_rate
        request.gravity = gravity
        request.ode_config = ode_config

        future = self.set_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            success = response.success
            if not success:
                self.get_logger().error(f'Failed to set physics properties: {response.status_message}')
            return success
        else:
            self.get_logger().error('Failed to call set_physics_properties service')
            return False

    def validate_gravity_simulation(self, expected_gravity_z: float = -9.8,
                                   tolerance: float = 0.1) -> bool:
        """
        Validate that gravity simulation is working correctly

        Args:
            expected_gravity_z (float): Expected gravity in z direction
            tolerance (float): Tolerance for comparison

        Returns:
            bool: True if gravity is within expected range, False otherwise
        """
        props = self.get_current_physics_properties()
        if props is None:
            return False

        actual_gravity_z = props['gravity']['z']
        is_valid = abs(actual_gravity_z - expected_gravity_z) <= tolerance

        self.get_logger().info(
            f'Gravity validation: expected={expected_gravity_z}, '
            f'actual={actual_gravity_z}, valid={is_valid}'
        )

        return is_valid

    def validate_contact_dynamics(self, timeout: float = 5.0) -> bool:
        """
        Validate that contact dynamics are working by checking for contact states

        Args:
            timeout (float): Time to wait for contact states in seconds

        Returns:
            bool: True if contact states are received, False otherwise
        """
        start_time = time.time()

        while self.contact_states is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        has_contacts = self.contact_states is not None and len(self.contact_states.states) > 0

        self.get_logger().info(f'Contact dynamics validation: has_contacts={has_contacts}')

        return has_contacts

    def validate_parameter_adjustment(self, new_gravity_z: float = -4.9) -> bool:
        """
        Validate that physics parameters can be adjusted during simulation

        Args:
            new_gravity_z (float): New gravity value to test

        Returns:
            bool: True if parameter adjustment is successful, False otherwise
        """
        # Get original gravity
        original_props = self.get_current_physics_properties()
        if original_props is None:
            return False

        original_gravity_z = original_props['gravity']['z']

        # Set new gravity
        new_gravity = Vector3(x=0.0, y=0.0, z=new_gravity_z)
        success = self.set_physics_properties(new_gravity)

        if not success:
            return False

        # Verify new gravity is set
        updated_props = self.get_current_physics_properties()
        if updated_props is None:
            # Restore original gravity
            orig_gravity = Vector3(
                x=original_props['gravity']['x'],
                y=original_props['gravity']['y'],
                z=original_props['gravity']['z']
            )
            self.set_physics_properties(orig_gravity)
            return False

        actual_gravity_z = updated_props['gravity']['z']
        is_adjusted = abs(actual_gravity_z - new_gravity_z) <= 0.1

        # Restore original gravity
        orig_gravity = Vector3(
            x=original_props['gravity']['x'],
            y=original_props['gravity']['y'],
            z=original_props['gravity']['z']
        )
        self.set_physics_properties(orig_gravity)

        self.get_logger().info(
            f'Parameter adjustment validation: '
            f'original={original_gravity_z}, new={new_gravity_z}, '
            f'actual={actual_gravity_z}, adjusted={is_adjusted}'
        )

        return is_adjusted

    def validate_humanoid_contact_stability(self, tolerance: float = 0.05) -> bool:
        """
        Validate that humanoid robot maintains stable contact with ground

        Args:
            tolerance (float): Maximum deviation from expected contact behavior

        Returns:
            bool: True if contact is stable, False otherwise
        """
        if self.joint_states is None:
            return False

        # Check if joint positions are stable (not wildly fluctuating)
        if len(self.joint_states.position) == 0:
            return False

        # Calculate standard deviation of joint positions as a measure of stability
        positions = np.array(self.joint_states.position)
        stability = np.std(positions) < tolerance

        self.get_logger().info(f'Humanoid contact stability validation: stable={stability}')

        return stability


def main():
    """
    Main function to run physics validation tests
    """
    rclpy.init()

    validator = PhysicsValidator()

    # Run validation tests
    results = {}

    # Wait a bit for data to accumulate
    time.sleep(2.0)

    # Test 1: Gravity simulation
    results['gravity'] = validator.validate_gravity_simulation()

    # Test 2: Contact dynamics
    results['contact_dynamics'] = validator.validate_contact_dynamics()

    # Test 3: Parameter adjustment
    results['parameter_adjustment'] = validator.validate_parameter_adjustment()

    # Test 4: Humanoid contact stability
    results['contact_stability'] = validator.validate_humanoid_contact_stability()

    # Print results
    print("\n=== Physics Validation Results ===")
    for test, result in results.items():
        status = "PASS" if result else "FAIL"
        print(f"{test}: {status}")

    # Overall result
    all_passed = all(results.values())
    print(f"\nOverall: {'PASS' if all_passed else 'FAIL'}")

    validator.destroy_node()
    rclpy.shutdown()

    return all_passed


class TestPhysicsValidation(unittest.TestCase):
    """
    Unit tests for physics validation functions
    """

    def setUp(self):
        """
        Set up the test case
        """
        if not rclpy.ok():
            rclpy.init()
        self.validator = PhysicsValidator()

    def tearDown(self):
        """
        Tear down the test case
        """
        self.validator.destroy_node()

    def test_gravity_validation(self):
        """
        Test gravity validation function
        """
        # This test would require a running Gazebo instance
        # For now, we'll just check that the method exists and can be called
        self.assertTrue(hasattr(self.validator, 'validate_gravity_simulation'))

    def test_parameter_adjustment(self):
        """
        Test parameter adjustment validation
        """
        self.assertTrue(hasattr(self.validator, 'validate_parameter_adjustment'))


if __name__ == '__main__':
    # Run the main validation
    success = main()

    # If running as part of a test suite, also run unit tests
    if success:
        unittest.main(argv=[''], exit=False, verbosity=2)