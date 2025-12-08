#!/usr/bin/env python3
"""
Common ROS 2 Interface Utilities

This module provides common utilities for ROS 2 interfaces used in the Digital Twin system.
It includes utilities for topic management, service calls, and TF transformations.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image, Imu, JointState
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from typing import Optional, Tuple, Dict, Any
import time


class CommonInterfaceUtils(Node):
    """
    Common utilities for ROS 2 interfaces in the Digital Twin system
    """

    def __init__(self, node_name: str = "common_interface_utils"):
        """
        Initialize the CommonInterfaceUtils node

        Args:
            node_name (str): Name of the ROS 2 node
        """
        super().__init__(node_name)

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # QoS profiles for different message types
        self.qos_profile_sensor_data = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.qos_profile_control = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.qos_profile_state = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

    def lookup_transform(self, target_frame: str, source_frame: str,
                        timeout: float = 1.0) -> Optional[TransformStamped]:
        """
        Lookup a transform between two frames

        Args:
            target_frame (str): Target frame
            source_frame (str): Source frame
            timeout (float): Timeout in seconds

        Returns:
            Optional[TransformStamped]: Transform if found, None otherwise
        """
        try:
            # Wait for transform to become available
            self.tf_buffer.wait_for_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout)
            )

            # Lookup the transform
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            return transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not lookup transform {source_frame} to {target_frame}: {e}")
            return None

    def broadcast_transform(self, translation: Tuple[float, float, float],
                          rotation: Tuple[float, float, float, float],
                          parent_frame: str, child_frame: str):
        """
        Broadcast a transform

        Args:
            translation (Tuple[float, float, float]): (x, y, z) translation
            rotation (Tuple[float, float, float, float]): (x, y, z, w) quaternion
            parent_frame (str): Parent frame ID
            child_frame (str): Child frame ID
        """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(t)

    def create_qos_profile(self, reliability: str = "reliable",
                          durability: str = "volatile", depth: int = 10) -> QoSProfile:
        """
        Create a QoS profile with specified parameters

        Args:
            reliability (str): "reliable" or "best_effort"
            durability (str): "volatile" or "transient_local"
            depth (int): Queue depth

        Returns:
            QoSProfile: Configured QoS profile
        """
        reliability_policy = (ReliabilityPolicy.RELIABLE if reliability == "reliable"
                             else ReliabilityPolicy.BEST_EFFORT)
        durability_policy = (DurabilityPolicy.TRANSIENT_LOCAL if durability == "transient_local"
                            else DurabilityPolicy.VOLATILE)

        return QoSProfile(
            depth=depth,
            reliability=reliability_policy,
            durability=durability_policy
        )

    def validate_sensor_data(self, msg: Any, sensor_type: str) -> bool:
        """
        Validate sensor data based on type

        Args:
            msg (Any): Sensor message
            sensor_type (str): Type of sensor ("lidar", "camera", "imu", "joint_state")

        Returns:
            bool: True if data is valid, False otherwise
        """
        if sensor_type == "lidar":
            if isinstance(msg, LaserScan):
                # Check for valid range values
                valid_ranges = [r for r in msg.ranges if not np.isnan(r) and r > 0]
                return len(valid_ranges) > 0
        elif sensor_type == "camera":
            if isinstance(msg, Image):
                # Check for valid image dimensions
                return msg.height > 0 and msg.width > 0
        elif sensor_type == "imu":
            if isinstance(msg, Imu):
                # Check for valid orientation or angular velocity
                has_orientation = not (np.isnan(msg.orientation.x) and
                                     np.isnan(msg.orientation.y) and
                                     np.isnan(msg.orientation.z) and
                                     np.isnan(msg.orientation.w))
                has_angular_velocity = not (np.isnan(msg.angular_velocity.x) and
                                          np.isnan(msg.angular_velocity.y) and
                                          np.isnan(msg.angular_velocity.z))
                return has_orientation or has_angular_velocity
        elif sensor_type == "joint_state":
            if isinstance(msg, JointState):
                # Check for valid joint positions
                valid_positions = [p for p in msg.position if not np.isnan(p)]
                return len(valid_positions) > 0

        return False

    def calculate_tf_error(self, transform1: TransformStamped,
                          transform2: TransformStamped) -> float:
        """
        Calculate the positional error between two transforms

        Args:
            transform1 (TransformStamped): First transform
            transform2 (TransformStamped): Second transform

        Returns:
            float: Positional error in meters
        """
        pos1 = transform1.transform.translation
        pos2 = transform2.transform.translation

        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z

        return np.sqrt(dx*dx + dy*dy + dz*dz)

    def get_timestamp_diff(self, msg1: Any, msg2: Any) -> float:
        """
        Get the time difference between two messages

        Args:
            msg1 (Any): First message with header
            msg2 (Any): Second message with header

        Returns:
            float: Time difference in seconds
        """
        try:
            time1 = msg1.header.stamp.sec + msg1.header.stamp.nanosec * 1e-9
            time2 = msg2.header.stamp.sec + msg2.header.stamp.nanosec * 1e-9
            return abs(time1 - time2)
        except AttributeError:
            # If messages don't have headers, return 0
            return 0.0


class TopicMonitor:
    """
    A utility class to monitor ROS 2 topics
    """

    def __init__(self, node: Node):
        """
        Initialize the TopicMonitor

        Args:
            node (Node): ROS 2 node instance
        """
        self.node = node
        self.message_counts = {}
        self.last_messages = {}
        self.message_times = {}

    def create_monitoring_subscriber(self, topic_name: str, msg_type: Any,
                                   callback=None, qos_profile=None):
        """
        Create a subscriber that monitors message statistics

        Args:
            topic_name (str): Name of the topic to monitor
            msg_type (Any): Message type
            callback (callable): Custom callback function
            qos_profile: QoS profile for the subscriber

        Returns:
            Subscription: The created subscription
        """
        if qos_profile is None:
            qos_profile = self.node.qos_profile_sensor_data

        def default_callback(msg):
            self.message_counts[topic_name] = self.message_counts.get(topic_name, 0) + 1
            self.last_messages[topic_name] = msg
            self.message_times[topic_name] = time.time()

            # Call custom callback if provided
            if callback:
                callback(msg)

        return self.node.create_subscription(
            msg_type,
            topic_name,
            default_callback,
            qos_profile
        )

    def get_topic_stats(self, topic_name: str) -> Dict[str, Any]:
        """
        Get statistics for a monitored topic

        Args:
            topic_name (str): Name of the topic

        Returns:
            Dict[str, Any]: Statistics dictionary
        """
        count = self.message_counts.get(topic_name, 0)
        last_time = self.message_times.get(topic_name, 0)
        current_time = time.time()

        return {
            'message_count': count,
            'last_message_time': last_time,
            'time_since_last': current_time - last_time if last_time > 0 else float('inf'),
            'has_data': topic_name in self.last_messages
        }


def main():
    """
    Main function for testing common interface utilities
    """
    rclpy.init()

    node = CommonInterfaceUtils("common_interface_test")

    # Example: Create a topic monitor
    monitor = TopicMonitor(node)

    # Example: Create a monitoring subscriber for joint states
    joint_sub = monitor.create_monitoring_subscriber(
        '/joint_states',
        JointState
    )

    # Example: Broadcast a test transform
    node.broadcast_transform(
        translation=(1.0, 2.0, 0.0),
        rotation=(0.0, 0.0, 0.0, 1.0),  # identity quaternion
        parent_frame='map',
        child_frame='test_frame'
    )

    # Example: Validate some sensor data (would need actual messages in practice)
    # valid = node.validate_sensor_data(joint_state_msg, "joint_state")

    node.get_logger().info("Common interface utilities initialized")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down common interface utilities")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()