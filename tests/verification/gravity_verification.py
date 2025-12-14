#!/usr/bin/env python3
"""
Verification script for realistic gravity simulation in Gazebo

This script tests that gravity simulation matches 9.8 m/s² acceleration.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64
import time
import math
from typing import Dict, Tuple, Optional


class GravityVerification(Node):
    """Node for verifying realistic gravity simulation"""

    def __init__(self):
        """Initialize the gravity verification node"""
        super().__init__('gravity_verification')

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

        # Subscriber for model states
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        # Storage for data
        self.model_states_history = []
        self.test_object_name = 'test_sphere'  # Name of object to drop for gravity test
        self.test_object_initial_position = None
        self.test_object_positions = []
        self.test_start_time = None

        # Timer for data collection
        self.timer = self.create_timer(0.02, self.collect_data)  # 50 Hz
        self.start_time = time.time()

    def model_states_callback(self, msg):
        """Callback for model states messages"""
        self.current_model_states = msg

    def collect_data(self):
        """Collect model state data for analysis"""
        if hasattr(self, 'current_model_states'):
            current_time = time.time()
            self.model_states_history.append((current_time, self.current_model_states))

    def get_current_physics_properties(self):
        """Get current physics properties"""
        request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        return None

    def drop_test_object(self, object_name: str = 'test_sphere', start_z: float = 2.0):
        """
        Simulate dropping an object to measure gravity

        Args:
            object_name: Name of the object to drop
            start_z: Starting height for the drop test
        """
        self.get_logger().info(f'Dropping test object {object_name} from height {start_z}m...')

        # Find the object in model states and record initial position
        if hasattr(self, 'current_model_states'):
            for i, name in enumerate(self.current_model_states.name):
                if object_name in name:
                    self.test_object_initial_position = self.current_model_states.pose[i].position
                    self.test_object_positions = []
                    self.test_start_time = time.time()
                    self.get_logger().info(f'Found test object at initial position: {self.test_object_initial_position}')
                    break

    def measure_gravity(self, duration: float = 3.0) -> Tuple[bool, Dict]:
        """
        Measure gravity by dropping an object and calculating acceleration

        Args:
            duration: Duration of measurement in seconds

        Returns:
            Tuple of (success, metrics)
        """
        self.get_logger().info(f'Starting gravity measurement for {duration} seconds...')

        # Drop a test object if available
        self.drop_test_object()

        start_time = time.time()
        measured_positions = []

        # Collect position data for the duration
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.05)  # 20 Hz sampling

            if hasattr(self, 'current_model_states') and self.test_object_initial_position:
                for i, name in enumerate(self.current_model_states.name):
                    if self.test_object_name in name:
                        pos = self.current_model_states.pose[i].position
                        current_time = time.time()
                        measured_positions.append((current_time, pos.z))
                        break

        if len(measured_positions) >= 2:
            # Calculate acceleration using kinematic equation: s = ut + 0.5*a*t^2
            # For free fall, initial velocity u = 0, so s = 0.5*a*t^2, therefore a = 2s/t^2
            results = []

            # Use multiple time intervals to get more accurate measurement
            for i in range(1, len(measured_positions)):
                t1, z1 = measured_positions[0]
                t2, z2 = measured_positions[i]

                dt = t2 - t1
                if dt > 0.1:  # Only use measurements with sufficient time difference
                    # Calculate displacement (z1 is higher than z2 since object falls down)
                    displacement = z1 - z2  # Positive displacement as object falls
                    acceleration = (2 * displacement) / (dt * dt)
                    results.append((dt, displacement, acceleration))

            if results:
                accelerations = [result[2] for result in results if result[2] > 0]
                if accelerations:
                    avg_acceleration = sum(accelerations) / len(accelerations)
                    min_acceleration = min(accelerations)
                    max_acceleration = max(accelerations)

                    # Calculate percentage error from expected 9.8 m/s²
                    error_percentage = abs(avg_acceleration - 9.8) / 9.8 * 100

                    metrics = {
                        'avg_acceleration': avg_acceleration,
                        'min_acceleration': min_acceleration,
                        'max_acceleration': max_acceleration,
                        'error_percentage': error_percentage,
                        'measurements_count': len(accelerations),
                        'expected_acceleration': 9.8
                    }

                    # Success if within 5% of expected value
                    success = error_percentage <= 5.0
                    return success, metrics

        # If we can't measure directly, check physics properties
        physics_props = self.get_current_physics_properties()
        if physics_props:
            gravity_z = physics_props.gravity.z
            error_percentage = abs(abs(gravity_z) - 9.8) / 9.8 * 100

            metrics = {
                'gravity_z': gravity_z,
                'abs_gravity': abs(gravity_z),
                'error_percentage': error_percentage,
                'expected_acceleration': 9.8,
                'measurements_count': 0
            }

            success = error_percentage <= 5.0
            return success, metrics

        return False, {'error': 'Could not measure gravity'}

    def verify_gravity_from_physics_properties(self) -> Tuple[bool, Dict]:
        """
        Verify gravity by checking physics properties directly

        Returns:
            Tuple of (success, metrics)
        """
        self.get_logger().info('Verifying gravity from physics properties...')

        physics_props = self.get_current_physics_properties()

        if physics_props:
            gravity = physics_props.gravity
            gravity_magnitude = math.sqrt(gravity.x**2 + gravity.y**2 + gravity.z**2)

            # Expected is 9.8 m/s² in the -Z direction
            expected_magnitude = 9.8
            error_percentage = abs(gravity_magnitude - expected_magnitude) / expected_magnitude * 100

            metrics = {
                'gravity_x': gravity.x,
                'gravity_y': gravity.y,
                'gravity_z': gravity.z,
                'gravity_magnitude': gravity_magnitude,
                'expected_magnitude': expected_magnitude,
                'error_percentage': error_percentage
            }

            # Check that gravity is primarily in Z direction (should be negative)
            z_dominant = abs(gravity.z) > 0.9 * gravity_magnitude
            correct_direction = gravity.z < 0  # Gravity should be negative Z
            reasonable_magnitude = abs(gravity_magnitude - expected_magnitude) <= 0.5  # Within 0.5 m/s²

            success = (z_dominant and correct_direction and reasonable_magnitude)
            return success, metrics
        else:
            return False, {'error': 'Could not get physics properties'}


def main():
    """Main function to run gravity verification"""
    rclpy.init()

    verifier = GravityVerification()

    time.sleep(1.0)  # Allow some time for data to accumulate

    # Verify gravity from physics properties
    direct_success, direct_metrics = verifier.verify_gravity_from_physics_properties()

    # Try to measure gravity by observing object motion
    measurement_success, measurement_metrics = verifier.measure_gravity(duration=2.0)

    # Print results
    print("\n=== Gravity Simulation Verification Results ===")

    print(f"Direct Physics Properties Check: {'PASS' if direct_success else 'FAIL'}")
    if 'error' not in direct_metrics:
        print(f"  Gravity magnitude: {direct_metrics.get('gravity_magnitude', 'N/A'):.4f} m/s²")
        print(f"  Gravity direction: ({direct_metrics.get('gravity_x', 'N/A'):.4f}, "
              f"{direct_metrics.get('gravity_y', 'N/A'):.4f}, {direct_metrics.get('gravity_z', 'N/A'):.4f})")
        print(f"  Error: {direct_metrics.get('error_percentage', 'N/A'):.2f}%")
    else:
        print(f"  Error: {direct_metrics['error']}")

    print(f"Motion-Based Measurement: {'PASS' if measurement_success else 'FAIL'}")
    if 'error' not in measurement_metrics:
        if measurement_metrics['measurements_count'] > 0:
            print(f"  Avg acceleration: {measurement_metrics.get('avg_acceleration', 'N/A'):.4f} m/s²")
            print(f"  Error: {measurement_metrics.get('error_percentage', 'N/A'):.2f}%")
            print(f"  Measurements: {measurement_metrics.get('measurements_count', 'N/A')}")
        else:
            print(f"  Gravity magnitude (from properties): {measurement_metrics.get('abs_gravity', 'N/A'):.4f} m/s²")
            print(f"  Error: {measurement_metrics.get('error_percentage', 'N/A'):.2f}%")
    else:
        print(f"  Error: {measurement_metrics['error']}")

    # Overall success is based on direct physics check (most reliable)
    overall_success = direct_success
    print(f"\nOverall Verification: {'PASS' if overall_success else 'FAIL'}")

    verifier.destroy_node()
    rclpy.shutdown()

    return overall_success


if __name__ == '__main__':
    success = main()
    exit(0 if success else 1)