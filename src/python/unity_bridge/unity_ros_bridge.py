#!/usr/bin/env python3
"""
Unity-ROS Bridge for Digital Twin System

This module provides the bridge between Unity visualization and ROS 2 simulation.
It synchronizes physics states, transforms, and sensor data between the two systems.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState, LaserScan, Image, Imu
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from builtin_interfaces.msg import Time
import socket
import json
import threading
import time
from typing import Dict, Any, Optional, List
import yaml
import numpy as np


class UnityROSBridge(Node):
    """
    Unity-ROS Bridge node for synchronizing between Gazebo simulation and Unity visualization
    """

    def __init__(self):
        """
        Initialize the Unity-ROS Bridge
        """
        super().__init__('unity_ros_bridge')

        # Load configuration
        self.config = self.load_configuration()

        # Initialize connection variables
        self.unity_socket = None
        self.is_connected = False

        # Initialize data storage
        self.joint_states = None
        self.transforms = {}
        self.sensor_data = {}

        # TF broadcaster for Unity transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # QoS profiles
        self.qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.qos_control = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers for Gazebo data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            self.qos_control
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers for Unity data
        self.unity_joint_pub = self.create_publisher(
            JointState,
            '/unity_joint_states',
            self.qos_control
        )

        # Setup connection to Unity
        self.setup_unity_connection()

        # Start synchronization timer
        self.sync_timer = self.create_timer(
            1.0 / self.config['synchronization']['visualization_sync']['frequency'],
            self.synchronize_data
        )

        self.get_logger().info('Unity-ROS Bridge initialized')

    def load_configuration(self) -> Dict[str, Any]:
        """
        Load configuration from YAML file

        Returns:
            Dict[str, Any]: Configuration dictionary
        """
        config_path = "simulation-assets/ros2/config/unity_bridge_config.yaml"

        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                return config
        except FileNotFoundError:
            self.get_logger().warn(f'Config file not found at {config_path}, using defaults')
            # Return default configuration
            return {
                'network': {
                    'unity_ip': '127.0.0.1',
                    'unity_port': 10000,
                    'connection_timeout': 10.0,
                    'auto_reconnect': True,
                    'reconnect_interval': 1.0
                },
                'synchronization': {
                    'visualization_sync': {
                        'frequency': 30.0
                    }
                }
            }

    def setup_unity_connection(self):
        """
        Setup connection to Unity application
        """
        try:
            self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.unity_socket.settimeout(self.config['network']['connection_timeout'])

            unity_ip = self.config['network']['unity_ip']
            unity_port = self.config['network']['unity_port']

            self.unity_socket.connect((unity_ip, unity_port))
            self.is_connected = True
            self.get_logger().info(f'Connected to Unity at {unity_ip}:{unity_port}')

            # Start receiving thread
            self.receive_thread = threading.Thread(target=self.receive_from_unity, daemon=True)
            self.receive_thread.start()

        except Exception as e:
            self.get_logger().error(f'Failed to connect to Unity: {e}')
            self.is_connected = False

            # If auto-reconnect is enabled, try again after interval
            if self.config['network']['auto_reconnect']:
                self.get_logger().info('Attempting to reconnect to Unity...')
                time.sleep(self.config['network']['reconnect_interval'])
                self.setup_unity_connection()

    def joint_state_callback(self, msg: JointState):
        """
        Callback for joint state messages from Gazebo
        """
        self.joint_states = msg
        # Forward to Unity if connected
        if self.is_connected:
            self.send_joint_states_to_unity(msg)

    def receive_from_unity(self):
        """
        Receive data from Unity in a separate thread
        """
        while rclpy.ok() and self.is_connected:
            try:
                data = self.unity_socket.recv(4096)
                if data:
                    # Process received data
                    self.process_unity_data(data)
            except socket.timeout:
                continue  # Just continue if timeout
            except Exception as e:
                self.get_logger().error(f'Error receiving from Unity: {e}')
                self.is_connected = False
                break

    def process_unity_data(self, data: bytes):
        """
        Process data received from Unity

        Args:
            data (bytes): Raw data from Unity
        """
        try:
            # Decode JSON data
            decoded_data = json.loads(data.decode('utf-8'))

            # Process different types of data
            if 'type' in decoded_data:
                msg_type = decoded_data['type']

                if msg_type == 'transform':
                    self.handle_unity_transform(decoded_data)
                elif msg_type == 'control':
                    self.handle_unity_control(decoded_data)
                elif msg_type == 'sensor':
                    self.handle_unity_sensor(decoded_data)

        except json.JSONDecodeError:
            self.get_logger().warn('Received invalid JSON from Unity')
        except Exception as e:
            self.get_logger().error(f'Error processing Unity data: {e}')

    def handle_unity_transform(self, data: Dict[str, Any]):
        """
        Handle transform data from Unity

        Args:
            data (Dict[str, Any]): Transform data from Unity
        """
        try:
            transform_data = data['data']

            # Create and broadcast transform
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = transform_data.get('parent_frame', 'map')
            t.child_frame_id = transform_data.get('child_frame', 'unity_object')

            t.transform.translation.x = transform_data.get('translation', {}).get('x', 0.0)
            t.transform.translation.y = transform_data.get('translation', {}).get('y', 0.0)
            t.transform.translation.z = transform_data.get('translation', {}).get('z', 0.0)

            t.transform.rotation.x = transform_data.get('rotation', {}).get('x', 0.0)
            t.transform.rotation.y = transform_data.get('rotation', {}).get('y', 0.0)
            t.transform.rotation.z = transform_data.get('rotation', {}).get('z', 0.0)
            t.transform.rotation.w = transform_data.get('rotation', {}).get('w', 1.0)

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f'Error handling Unity transform: {e}')

    def handle_unity_control(self, data: Dict[str, Any]):
        """
        Handle control commands from Unity

        Args:
            data (Dict[str, Any]): Control data from Unity
        """
        # In a full implementation, this would publish control commands to ROS topics
        self.get_logger().debug(f'Received control command from Unity: {data}')

    def handle_unity_sensor(self, data: Dict[str, Any]):
        """
        Handle sensor data from Unity

        Args:
            data (Dict[str, Any]): Sensor data from Unity
        """
        # Store sensor data received from Unity
        sensor_type = data.get('sensor_type', 'unknown')
        self.sensor_data[sensor_type] = data.get('data', {})

    def send_joint_states_to_unity(self, joint_state: JointState):
        """
        Send joint states to Unity

        Args:
            joint_state (JointState): Joint state message to send
        """
        if not self.is_connected:
            return

        try:
            # Convert joint state to dictionary format
            joint_data = {
                'type': 'joint_state',
                'timestamp': time.time(),
                'data': {
                    'name': list(joint_state.name),
                    'position': list(joint_state.position),
                    'velocity': list(joint_state.velocity),
                    'effort': list(joint_state.effort)
                }
            }

            # Send to Unity
            json_data = json.dumps(joint_data)
            self.unity_socket.send(json_data.encode('utf-8'))

        except Exception as e:
            self.get_logger().error(f'Error sending joint states to Unity: {e}')
            self.is_connected = False

    def synchronize_data(self):
        """
        Synchronize data between ROS and Unity at regular intervals
        """
        if not self.is_connected:
            # Try to reconnect if connection was lost
            if self.config['network']['auto_reconnect']:
                self.setup_unity_connection()
            return

        try:
            # Send current joint states to Unity
            if self.joint_states is not None:
                self.send_joint_states_to_unity(self.joint_states)

            # Send synchronization message
            sync_data = {
                'type': 'synchronization',
                'timestamp': time.time(),
                'data': {
                    'connected': True,
                    'ros_time': self.get_clock().now().nanoseconds / 1e9
                }
            }

            json_data = json.dumps(sync_data)
            self.unity_socket.send(json_data.encode('utf-8'))

        except Exception as e:
            self.get_logger().error(f'Error in data synchronization: {e}')
            self.is_connected = False

    def destroy_node(self):
        """
        Clean up resources when node is destroyed
        """
        if self.unity_socket:
            self.unity_socket.close()
        super().destroy_node()


def main():
    """
    Main function to run the Unity-ROS Bridge
    """
    rclpy.init()

    bridge = UnityROSBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('Shutting down Unity-ROS Bridge')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()