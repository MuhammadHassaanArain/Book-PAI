#!/usr/bin/env python3
"""
Physics Configuration Utility for Gazebo Simulation

This module provides utilities for configuring physics parameters in Gazebo simulations.
It includes functions for setting gravity, solver parameters, and contact properties.
"""

import rospy
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3


class PhysicsConfig:
    """
    A class to handle physics configuration in Gazebo simulations
    """

    def __init__(self):
        """
        Initialize the PhysicsConfig class
        """
        rospy.wait_for_service('/gazebo/set_physics_properties')
        self.set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_properties = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)

    def set_gravity(self, x=0.0, y=0.0, z=-9.8):
        """
        Set gravity vector in the simulation

        Args:
            x (float): Gravity in x direction (m/s^2)
            y (float): Gravity in y direction (m/s^2)
            z (float): Gravity in z direction (m/s^2), typically -9.8
        """
        try:
            # Get current physics properties to preserve other settings
            current_props = self.get_physics_properties()

            # Set new gravity
            gravity = Vector3(x=x, y=y, z=z)

            # Call the service to set physics properties
            response = self.set_physics_properties(
                time_step=current_props.time_step,
                max_update_rate=current_props.max_update_rate,
                gravity=gravity,
                ode_config=current_props.ode_config
            )

            if response.success:
                rospy.loginfo(f"Gravity successfully set to: ({x}, {y}, {z})")
            else:
                rospy.logerr(f"Failed to set gravity: {response.status_message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_solver_parameters(self, time_step=0.001, max_update_rate=1000.0,
                             solver_type="ode", iters=100, sor=1.3):
        """
        Set physics solver parameters

        Args:
            time_step (float): Simulation time step
            max_update_rate (float): Maximum update rate
            solver_type (str): Type of physics solver ("ode", "bullet", etc.)
            iters (int): Number of solver iterations
            sor (float): Successive over-relaxation parameter
        """
        try:
            # Get current physics properties to preserve other settings
            current_props = self.get_physics_properties()

            # Create ODE config
            ode_config = current_props.ode_config
            ode_config.auto_disable_bodies = False
            ode_config.sor_pgs_precon_iters = 0
            ode_config.sor_pgs_iters = iters
            ode_config.sor_pgs_w = sor
            ode_config.contact_surface_layer = 0.001
            ode_config.contact_max_correcting_vel = 100.0
            ode_config.cfm = 0.0
            ode_config.erp = 0.2
            ode_config.max_contacts = 20

            # Call the service to set physics properties
            response = self.set_physics_properties(
                time_step=time_step,
                max_update_rate=max_update_rate,
                gravity=current_props.gravity,
                ode_config=ode_config
            )

            if response.success:
                rospy.loginfo(f"Solver parameters successfully set: time_step={time_step}, iters={iters}")
            else:
                rospy.logerr(f"Failed to set solver parameters: {response.status_message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_contact_parameters(self, cfm=0.0, erp=0.2, max_vel=100.0, min_depth=0.001):
        """
        Set contact parameters for realistic collision behavior

        Args:
            cfm (float): Constraint Force Mixing parameter
            erp (float): Error Reduction Parameter
            max_vel (float): Maximum correcting velocity
            min_depth (float): Minimum contact surface layer
        """
        try:
            # Get current physics properties to preserve other settings
            current_props = self.get_physics_properties()

            # Modify ODE config for contact parameters
            ode_config = current_props.ode_config
            ode_config.contact_surface_layer = min_depth
            ode_config.contact_max_correcting_vel = max_vel
            ode_config.cfm = cfm
            ode_config.erp = erp

            # Call the service to set physics properties
            response = self.set_physics_properties(
                time_step=current_props.time_step,
                max_update_rate=current_props.max_update_rate,
                gravity=current_props.gravity,
                ode_config=ode_config
            )

            if response.success:
                rospy.loginfo(f"Contact parameters successfully set: cfm={cfm}, erp={erp}")
            else:
                rospy.logerr(f"Failed to set contact parameters: {response.status_message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def get_current_physics_properties(self):
        """
        Get current physics properties from the simulation

        Returns:
            dict: Dictionary containing current physics properties
        """
        try:
            props = self.get_physics_properties()
            return {
                'time_step': props.time_step,
                'max_update_rate': props.max_update_rate,
                'gravity': {
                    'x': props.gravity.x,
                    'y': props.gravity.y,
                    'z': props.gravity.z
                },
                'ode_config': {
                    'iters': props.ode_config.sor_pgs_iters,
                    'sor': props.ode_config.sor_pgs_w,
                    'cfm': props.ode_config.cfm,
                    'erp': props.ode_config.erp
                }
            }
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None


def main():
    """
    Main function for testing physics configuration
    """
    rospy.init_node('physics_config_node', anonymous=True)

    # Create physics configuration object
    physics_config = PhysicsConfig()

    # Example: Set custom gravity
    physics_config.set_gravity(x=0.0, y=0.0, z=-9.8)

    # Example: Set solver parameters for humanoid stability
    physics_config.set_solver_parameters(
        time_step=0.001,
        max_update_rate=1000.0,
        iters=200,  # Higher iterations for better stability
        sor=1.3
    )

    # Example: Set contact parameters for realistic interaction
    physics_config.set_contact_parameters(
        cfm=1e-5,  # Very low CFM for stiff contacts
        erp=0.8,   # Higher ERP for stronger contact constraints
        max_vel=100.0,
        min_depth=0.001
    )

    rospy.loginfo("Physics configuration completed")

    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    main()