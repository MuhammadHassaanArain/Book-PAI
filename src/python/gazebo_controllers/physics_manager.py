#!/usr/bin/env python3
"""
Physics Parameter Manager for Gazebo Simulation

This module provides utilities for managing and adjusting physics parameters in Gazebo simulations.
It includes functionality for changing gravity, adjusting solver parameters, and tuning contact properties.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import time
from typing import Dict, Any, Optional


class PhysicsManager(Node):
    """
    A class to manage and adjust physics parameters in Gazebo simulations
    """

    def __init__(self):
        """
        Initialize the PhysicsManager class
        """
        super().__init__('physics_manager')

        # Initialize service clients
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

        # Store current physics properties
        self.current_properties = self.get_current_physics_properties()

        self.get_logger().info('Physics Manager initialized')

    def get_current_physics_properties(self) -> Optional[Dict[str, Any]]:
        """
        Get current physics properties from Gazebo

        Returns:
            Optional[Dict[str, Any]]: Current physics properties or None if failed
        """
        request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            props = {
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
            self.get_logger().info(f'Current physics properties: {props}')
            return props
        else:
            self.get_logger().error('Failed to get physics properties')
            return None

    def set_gravity(self, x: float = 0.0, y: float = 0.0, z: float = -9.8) -> bool:
        """
        Set gravity vector in the simulation

        Args:
            x (float): Gravity in x direction (m/s^2)
            y (float): Gravity in y direction (m/s^2)
            z (float): Gravity in z direction (m/s^2), typically -9.8

        Returns:
            bool: True if successful, False otherwise
        """
        # Get current physics properties to preserve other settings
        current_props = self.get_current_physics_properties()
        if current_props is None:
            return False

        # Set new gravity
        gravity = Vector3(x=x, y=y, z=z)

        # Create ODE config from current settings
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = current_props['ode_config']['iters']
        ode_config.sor_pgs_w = current_props['ode_config']['sor']
        ode_config.contact_surface_layer = 0.001
        ode_config.contact_max_correcting_vel = 100.0
        ode_config.cfm = current_props['ode_config']['cfm']
        ode_config.erp = current_props['ode_config']['erp']
        ode_config.max_contacts = 20

        # Create the request
        request = SetPhysicsProperties.Request()
        request.time_step = current_props['time_step']
        request.max_update_rate = current_props['max_update_rate']
        request.gravity = gravity
        request.ode_config = ode_config

        future = self.set_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            success = response.success
            if success:
                self.get_logger().info(f'Gravity successfully set to: ({x}, {y}, {z})')
            else:
                self.get_logger().error(f'Failed to set gravity: {response.status_message}')
            return success
        else:
            self.get_logger().error('Failed to call set_physics_properties service')
            return False

    def set_solver_parameters(self, time_step: float = 0.001, max_update_rate: float = 1000.0,
                             iters: int = 100, sor: float = 1.3) -> bool:
        """
        Set physics solver parameters

        Args:
            time_step (float): Simulation time step
            max_update_rate (float): Maximum update rate
            iters (int): Number of solver iterations
            sor (float): Successive over-relaxation parameter

        Returns:
            bool: True if successful, False otherwise
        """
        # Get current physics properties to preserve other settings
        current_props = self.get_current_physics_properties()
        if current_props is None:
            return False

        # Create new ODE config
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = iters
        ode_config.sor_pgs_w = sor
        ode_config.contact_surface_layer = 0.001
        ode_config.contact_max_correcting_vel = 100.0
        ode_config.cfm = current_props['ode_config']['cfm']
        ode_config.erp = current_props['ode_config']['erp']
        ode_config.max_contacts = 20

        # Create the request
        request = SetPhysicsProperties.Request()
        request.time_step = time_step
        request.max_update_rate = max_update_rate
        request.gravity.x = current_props['gravity']['x']
        request.gravity.y = current_props['gravity']['y']
        request.gravity.z = current_props['gravity']['z']
        request.ode_config = ode_config

        future = self.set_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            success = response.success
            if success:
                self.get_logger().info(f'Solver parameters set: time_step={time_step}, iters={iters}, sor={sor}')
            else:
                self.get_logger().error(f'Failed to set solver parameters: {response.status_message}')
            return success
        else:
            self.get_logger().error('Failed to call set_physics_properties service')
            return False

    def set_contact_parameters(self, cfm: float = 1e-6, erp: float = 0.2,
                             contact_surface_layer: float = 0.001,
                             contact_max_correcting_vel: float = 100.0) -> bool:
        """
        Set contact parameters for realistic collision behavior

        Args:
            cfm (float): Constraint Force Mixing parameter
            erp (float): Error Reduction Parameter
            contact_surface_layer (float): Contact surface layer thickness
            contact_max_correcting_vel (float): Maximum correcting velocity

        Returns:
            bool: True if successful, False otherwise
        """
        # Get current physics properties to preserve other settings
        current_props = self.get_current_physics_properties()
        if current_props is None:
            return False

        # Create ODE config with new contact parameters
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = current_props['ode_config']['iters']
        ode_config.sor_pgs_w = current_props['ode_config']['sor']
        ode_config.contact_surface_layer = contact_surface_layer
        ode_config.contact_max_correcting_vel = contact_max_correcting_vel
        ode_config.cfm = cfm
        ode_config.erp = erp
        ode_config.max_contacts = 20

        # Create the request
        request = SetPhysicsProperties.Request()
        request.time_step = current_props['time_step']
        request.max_update_rate = current_props['max_update_rate']
        request.gravity.x = current_props['gravity']['x']
        request.gravity.y = current_props['gravity']['y']
        request.gravity.z = current_props['gravity']['z']
        request.ode_config = ode_config

        future = self.set_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            success = response.success
            if success:
                self.get_logger().info(f'Contact parameters set: cfm={cfm}, erp={erp}')
            else:
                self.get_logger().error(f'Failed to set contact parameters: {response.status_message}')
            return success
        else:
            self.get_logger().error('Failed to call set_physics_properties service')
            return False

    def set_all_physics_parameters(self, gravity: tuple = (0, 0, -9.8),
                                  time_step: float = 0.001, max_update_rate: float = 1000.0,
                                  solver_iters: int = 100, solver_sor: float = 1.3,
                                  contact_cfm: float = 1e-6, contact_erp: float = 0.2) -> bool:
        """
        Set all physics parameters at once

        Args:
            gravity (tuple): Gravity vector (x, y, z)
            time_step (float): Simulation time step
            max_update_rate (float): Maximum update rate
            solver_iters (int): Solver iterations
            solver_sor (float): Solver successive over-relaxation parameter
            contact_cfm (float): Contact constraint force mixing
            contact_erp (float): Contact error reduction parameter

        Returns:
            bool: True if successful, False otherwise
        """
        # Create ODE config
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = solver_iters
        ode_config.sor_pgs_w = solver_sor
        ode_config.contact_surface_layer = 0.001
        ode_config.contact_max_correcting_vel = 100.0
        ode_config.cfm = contact_cfm
        ode_config.erp = contact_erp
        ode_config.max_contacts = 20

        # Create gravity vector
        grav_vec = Vector3(x=gravity[0], y=gravity[1], z=gravity[2])

        # Create the request
        request = SetPhysicsProperties.Request()
        request.time_step = time_step
        request.max_update_rate = max_update_rate
        request.gravity = grav_vec
        request.ode_config = ode_config

        future = self.set_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            success = response.success
            if success:
                self.get_logger().info(f'All physics parameters set: g={gravity}, dt={time_step}, iters={solver_iters}')
            else:
                self.get_logger().error(f'Failed to set physics parameters: {response.status_message}')
            return success
        else:
            self.get_logger().error('Failed to call set_physics_properties service')
            return False

    def get_parameter_presets(self) -> Dict[str, Dict[str, Any]]:
        """
        Get predefined parameter presets for different scenarios

        Returns:
            Dict[str, Dict[str, Any]]: Dictionary of parameter presets
        """
        return {
            'default': {
                'gravity': (0, 0, -9.8),
                'time_step': 0.001,
                'max_update_rate': 1000.0,
                'solver_iters': 100,
                'solver_sor': 1.3,
                'contact_cfm': 1e-6,
                'contact_erp': 0.2
            },
            'stable': {
                'gravity': (0, 0, -9.8),
                'time_step': 0.001,
                'max_update_rate': 1000.0,
                'solver_iters': 200,
                'solver_sor': 1.3,
                'contact_cfm': 1e-7,
                'contact_erp': 0.8
            },
            'fast': {
                'gravity': (0, 0, -9.8),
                'time_step': 0.002,
                'max_update_rate': 500.0,
                'solver_iters': 50,
                'solver_sor': 1.2,
                'contact_cfm': 1e-5,
                'contact_erp': 0.1
            },
            'moon_gravity': {
                'gravity': (0, 0, -1.62),
                'time_step': 0.001,
                'max_update_rate': 1000.0,
                'solver_iters': 100,
                'solver_sor': 1.3,
                'contact_cfm': 1e-6,
                'contact_erp': 0.2
            },
            'zero_gravity': {
                'gravity': (0, 0, 0),
                'time_step': 0.001,
                'max_update_rate': 1000.0,
                'solver_iters': 100,
                'solver_sor': 1.3,
                'contact_cfm': 1e-6,
                'contact_erp': 0.2
            }
        }

    def apply_preset(self, preset_name: str) -> bool:
        """
        Apply a predefined parameter preset

        Args:
            preset_name (str): Name of the preset to apply

        Returns:
            bool: True if successful, False otherwise
        """
        presets = self.get_parameter_presets()

        if preset_name not in presets:
            self.get_logger().error(f'Unknown preset: {preset_name}')
            return False

        preset = presets[preset_name]
        return self.set_all_physics_parameters(
            gravity=preset['gravity'],
            time_step=preset['time_step'],
            max_update_rate=preset['max_update_rate'],
            solver_iters=preset['solver_iters'],
            solver_sor=preset['solver_sor'],
            contact_cfm=preset['contact_cfm'],
            contact_erp=preset['contact_erp']
        )


def main():
    """
    Main function for testing physics manager functionality
    """
    rclpy.init()

    physics_manager = PhysicsManager()

    # Example: Apply different presets
    print("Applying stable preset...")
    success = physics_manager.apply_preset('stable')
    if success:
        print("Stable preset applied successfully")
    else:
        print("Failed to apply stable preset")

    # Wait a bit
    time.sleep(2.0)

    # Example: Set custom gravity
    print("Setting custom moon gravity...")
    success = physics_manager.set_gravity(x=0.0, y=0.0, z=-1.62)
    if success:
        print("Moon gravity set successfully")
    else:
        print("Failed to set moon gravity")

    # Wait a bit
    time.sleep(2.0)

    # Example: Adjust solver parameters for performance
    print("Setting fast solver parameters...")
    success = physics_manager.set_solver_parameters(
        time_step=0.002,
        max_update_rate=500.0,
        iters=50,
        sor=1.2
    )
    if success:
        print("Fast solver parameters set successfully")
    else:
        print("Failed to set fast solver parameters")

    # Keep the node running
    try:
        rclpy.spin(physics_manager)
    except KeyboardInterrupt:
        physics_manager.get_logger().info('Shutting down Physics Manager')
    finally:
        physics_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()