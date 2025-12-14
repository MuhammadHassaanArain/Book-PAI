# Lab 3: Simulate LiDAR, Depth Camera, and IMU

## Objective

In this lab, you will learn to simulate various sensors including LiDAR, depth cameras, and IMUs in Gazebo. You will configure sensor parameters, publish data to ROS 2 topics, and validate sensor accuracy with noise modeling.

## Prerequisites

- Basic understanding of ROS 2 and sensor messages
- Completed Lab 1-2
- Working Gazebo installation with ROS 2 integration
- Basic knowledge of sensor physics

## Lab Tasks

### Task 1: Configure LiDAR Sensor

Create a LiDAR sensor configuration in your world file:

```xml
<model name="sensor_platform">
  <pose>0 0 0.5 0 0 0</pose>
  <link name="base_link">
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.166</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.166</iyy>
        <iyz>0</iyz>
        <izz>0.166</izz>
      </inertia>
    </inertial>
  </link>

  <!-- LiDAR Sensor -->
  <sensor name="lidar_2d" type="ray">
    <pose>0.2 0 0.1 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>

    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>

    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>robot1</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>robot1/lidar_link</frame_name>
    </plugin>
  </sensor>
</model>
```

### Task 2: Configure Depth Camera Sensor

Add a depth camera sensor to your platform:

```xml
<!-- Depth Camera Sensor -->
<sensor name="depth_camera" type="depth">
  <pose>0.2 0 0.2 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>30</update_rate>

  <camera>
    <horizontal_fov>1.0472</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>

  <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <ros>
      <namespace>robot1</namespace>
      <remapping>~/depth/image_raw:=depth/image_raw</remapping>
      <remapping>~/rgb/image_raw:=rgb/image_raw</remapping>
      <remapping>~/depth/camera_info:=depth/camera_info</remapping>
    </ros>
    <frame_name>robot1/depth_camera_optical_frame</frame_name>
    <baseline>0.1</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
    <point_cloud_cutoff>0.1</point_cloud_cutoff>
    <point_cloud_cutoff_max>3.0</point_cloud_cutoff_max>
    <Cx>320.0</Cx>
    <Cy>240.0</Cy>
    <focal_length>525.0</focal_length>
  </plugin>
</sensor>
```

### Task 3: Configure IMU Sensor

Add an IMU sensor to your platform:

```xml
<!-- IMU Sensor -->
<sensor name="imu_sensor" type="imu">
  <pose>0.0 0 0.1 0 0 0</pose>
  <update_rate>100</update_rate>

  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </z>
    </angular_velocity>

    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.01</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>

  <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>robot1</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <frame_name>robot1/imu_link</frame_name>
    <body_name>sensor_platform</body_name>
    <update_rate>100</update_rate>
    <gaussian_noise>0.017</gaussian_noise>
    <topic_name>imu/data</topic_name>
  </plugin>
</sensor>
```

### Task 4: Validate Sensor Data

Create a ROS 2 node to validate sensor data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np
import math

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Create subscribers for each sensor type
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.lidar_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/robot1/depth/image_raw',
            self.depth_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/robot1/imu/data',
            self.imu_callback,
            10
        )

        self.cv_bridge = CvBridge()

        # Statistics trackers
        self.lidar_stats = {'count': 0, 'min_range': float('inf'), 'max_range': 0}
        self.depth_stats = {'count': 0, 'min_depth': float('inf'), 'max_depth': 0}
        self.imu_stats = {'count': 0, 'acc_mag': [], 'gyro_mag': []}

        self.get_logger().info('Sensor Validator initialized')

    def lidar_callback(self, msg):
        self.lidar_stats['count'] += 1

        # Calculate valid range statistics
        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and r > 0]
        if valid_ranges:
            self.lidar_stats['min_range'] = min(self.lidar_stats['min_range'], min(valid_ranges))
            self.lidar_stats['max_range'] = max(self.lidar_stats['max_range'], max(valid_ranges))

        if self.lidar_stats['count'] % 100 == 0:
            self.get_logger().info(
                f'Lidar Stats - Count: {self.lidar_stats["count"]}, '
                f'Min Range: {self.lidar_stats["min_range"]:.2f}, '
                f'Max Range: {self.lidar_stats["max_range"]:.2f}'
            )

    def depth_callback(self, msg):
        self.depth_stats['count'] += 1

        # Convert to numpy array for analysis
        depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Calculate depth statistics
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            self.depth_stats['min_depth'] = min(self.depth_stats['min_depth'], np.min(valid_depths))
            self.depth_stats['max_depth'] = max(self.depth_stats['max_depth'], np.max(valid_depths))

        if self.depth_stats['count'] % 100 == 0:
            self.get_logger().info(
                f'Depth Stats - Count: {self.depth_stats["count"]}, '
                f'Min Depth: {self.depth_stats["min_depth"]:.2f}, '
                f'Max Depth: {self.depth_stats["max_depth"]:.2f}'
            )

    def imu_callback(self, msg):
        self.imu_stats['count'] += 1

        # Calculate magnitude of acceleration and angular velocity
        acc_mag = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        gyro_mag = math.sqrt(
            msg.angular_velocity.x**2 +
            msg.angular_velocity.y**2 +
            msg.angular_velocity.z**2
        )

        self.imu_stats['acc_mag'].append(acc_mag)
        self.imu_stats['gyro_mag'].append(gyro_mag)

        if self.imu_stats['count'] % 100 == 0:
            avg_acc = np.mean(self.imu_stats['acc_mag'][-100:]) if len(self.imu_stats['acc_mag']) >= 100 else np.mean(self.imu_stats['acc_mag'])
            avg_gyro = np.mean(self.imu_stats['gyro_mag'][-100:]) if len(self.imu_stats['gyro_mag']) >= 100 else np.mean(self.imu_stats['gyro_mag'])

            self.get_logger().info(
                f'IMU Stats - Count: {self.imu_stats["count"]}, '
                f'Avg Acc: {avg_acc:.3f}, Avg Gyro: {avg_gyro:.3f}'
            )

def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 5: Sensor Fusion Node

Create a basic sensor fusion node to combine sensor data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribers for sensor data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.lidar_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/robot1/imu/data',
            self.imu_callback,
            10
        )

        # Publisher for fused data
        self.odom_publisher = self.create_publisher(Twist, '/robot1/odom_fused', 10)

        # Initialize state variables
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Timer for publishing fused data
        self.timer = self.create_timer(0.05, self.publish_fused_data)  # 20 Hz

        self.get_logger().info('Sensor Fusion Node initialized')

    def lidar_callback(self, msg):
        # Process LiDAR data for obstacle detection
        # This is a simplified example
        min_distance = float('inf')
        for i, range_val in enumerate(msg.ranges):
            if not math.isnan(range_val) and range_val > 0 and range_val < min_distance:
                min_distance = range_val

        # Simple obstacle avoidance based on LiDAR
        if min_distance < 1.0:  # Obstacle closer than 1m
            self.linear_velocity *= 0.5  # Slow down
            self.angular_velocity = 0.5  # Turn away from obstacle

    def imu_callback(self, msg):
        # Integrate IMU data for position estimation
        dt = 0.05  # Time step (from timer)

        # Update angular velocity from IMU
        self.angular_velocity = msg.angular_velocity.z

        # Update linear acceleration from IMU
        linear_acc = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2
        )

        # Update linear velocity (simplified integration)
        self.linear_velocity += linear_acc * dt

        # Update orientation
        self.orientation_z += self.angular_velocity * dt

        # Update position (simplified odometry)
        self.position_x += self.linear_velocity * math.cos(self.orientation_z) * dt
        self.position_y += self.linear_velocity * math.sin(self.orientation_z) * dt

    def publish_fused_data(self):
        # Create and publish fused odometry data
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.angular_velocity

        self.odom_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab Exercises

1. **LiDAR Range Validation**: Test LiDAR detection at various distances and validate range accuracy
2. **Camera Calibration**: Calibrate the depth camera using checkerboard patterns
3. **IMU Drift Analysis**: Monitor IMU drift over extended periods and implement bias correction
4. **Sensor Fusion**: Combine multiple sensors for improved localization accuracy
5. **Noise Modeling**: Validate sensor noise models against theoretical specifications

## Validation Steps

1. Launch the sensor simulation:
   ```bash
   gazebo your_world_with_sensors.world
   ```

2. Monitor sensor topics:
   ```bash
   ro2 topic echo /robot1/scan
   ros2 topic echo /robot1/depth/image_raw
   ros2 topic echo /robot1/imu/data
   ```

3. Run the validation node:
   ```bash
   ros2 run your_package sensor_validator
   ```

4. Verify data quality and accuracy

## Expected Outcomes

- Working LiDAR, depth camera, and IMU sensors
- Proper ROS 2 topic publishing
- Validated sensor data quality
- Basic sensor fusion capabilities

## References

1. ROS 2 Documentation. (2023). Sensor Integration. https://docs.ros.org/en/humble/p/sensor_msgs/
2. Open Source Robotics Foundation. (2023). Gazebo Sensor Tutorial. http://gazebosim.org/tutorials/?tut=ros_gzplugins_sensors
3. Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics. Springer.