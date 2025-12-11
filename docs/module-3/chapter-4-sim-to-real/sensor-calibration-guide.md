# Real-World Sensor Calibration Against Simulation

## Overview

This guide provides procedures for calibrating real-world sensors to match simulation parameters, ensuring consistent perception and navigation performance across sim-to-real transfer.

## Camera Calibration

### 1. Intrinsic Calibration

#### Calibration Process
```bash
# Install camera calibration tools
sudo apt install ros-humble-camera-calibration

# Launch calibration with checkerboard pattern
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.108 \
    image:=/camera/image_raw \
    camera:=/camera
```

#### Calibration File Format
```yaml
# camera_calibration.yaml
camera_name: realsense_camera
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [615.179443, 0.0, 320.5, 0.0, 615.179443, 240.5, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [615.179443, 0.0, 320.5, 0.0, 0.0, 615.179443, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
```

### 2. Extrinsic Calibration

#### Camera to Robot Calibration
```bash
# Use robot_state_publisher to define camera extrinsics
# robot.urdf
<link name="camera_link">
  <visual>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.3" rpy="0 0.1 0"/>  # Position relative to base
</joint>
```

## LiDAR Calibration

### 1. LiDAR Intrinsic Calibration

#### Point Cloud Alignment
```python
# lidar_calibration.py
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

def align_point_clouds(source_pcd, target_pcd):
    """Align source point cloud to target using ICP"""
    # Downsample point clouds
    source_down = source_pcd.voxel_down_sample(voxel_size=0.05)
    target_down = target_pcd.voxel_down_sample(voxel_size=0.05)

    # Initial alignment (identity transformation)
    current_transformation = np.eye(4)

    # Apply ICP
    result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, max_correspondence_distance=0.2,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )

    return result.transformation

def calibrate_lidar_extrinsics(sim_pcd_path, real_pcd_path):
    """Calibrate LiDAR extrinsics by aligning simulated and real point clouds"""
    # Load point clouds
    sim_pcd = o3d.io.read_point_cloud(sim_pcd_path)
    real_pcd = o3d.io.read_point_cloud(real_pcd_path)

    # Align point clouds
    transformation = align_point_clouds(sim_pcd, real_pcd)

    print("Calibration transformation matrix:")
    print(transformation)

    # Save calibration
    np.savetxt('lidar_extrinsics_calibration.txt', transformation)
    return transformation
```

## IMU Calibration

### 1. Bias and Scale Factor Calibration

#### IMU Calibration Process
```bash
# Install IMU calibration tools
sudo apt install ros-humble-imu-tools

# Collect stationary data for bias calibration
ros2 run imu_calibration imu_calibrator
```

#### Calibration Node
```python
# imu_calibrator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.accel_samples = []
        self.gyro_samples = []
        self.sample_count = 0
        self.max_samples = 1000

        self.get_logger().info('IMU Calibrator initialized - Keep robot stationary')

    def imu_callback(self, msg):
        # Collect stationary IMU readings
        self.accel_samples.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        self.gyro_samples.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        self.sample_count += 1

        if self.sample_count >= self.max_samples:
            self.compute_calibration()
            self.destroy_node()

    def compute_calibration(self):
        # Calculate bias values
        accel_bias = np.mean(self.accel_samples, axis=0)
        gyro_bias = np.mean(self.gyro_samples, axis=0)

        # Expected gravity on Z-axis
        accel_bias[2] -= 9.81

        self.get_logger().info(f'Acceleration bias: {accel_bias}')
        self.get_logger().info(f'Gyroscope bias: {gyro_bias}')

        # Save calibration to file
        calibration_data = {
            'accel_bias': accel_bias.tolist(),
            'gyro_bias': gyro_bias.tolist()
        }

        import json
        with open('imu_calibration.json', 'w') as f:
            json.dump(calibration_data, f)

        self.get_logger().info('IMU calibration saved to imu_calibration.json')

def main(args=None):
    rclpy.init(args=args)
    calibrator = IMUCalibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation-to-Reality Mapping

### 1. Sensor Parameter Alignment

#### Parameter Configuration
```yaml
# sensor_alignment_params.yaml
camera_alignment:
  intrinsic_matching:
    # Match focal lengths and optical centers
    focal_length_ratio: 1.0  # Adjust if needed
    optical_center_offset_x: 0.0  # Pixels
    optical_center_offset_y: 0.0  # Pixels

  distortion_matching:
    # Adjust distortion coefficients to match simulation
    k1_ratio: 1.0  # Radial distortion coefficient ratio
    k2_ratio: 1.0  # Radial distortion coefficient ratio
    p1_ratio: 1.0  # Tangential distortion coefficient ratio
    p2_ratio: 1.0  # Tangential distortion coefficient ratio

lidar_alignment:
  # Point cloud alignment parameters
  resolution_matching: 0.05  # Resolution to match simulation
  range_matching: 10.0       # Range to match simulation
  noise_model:
    # Add noise to match simulation characteristics
    range_noise_std: 0.01    # Range measurement noise
    angular_noise_std: 0.001 # Angular measurement noise

imu_alignment:
  # IMU parameter matching
  bias_compensation:
    accel_bias_x: 0.01
    accel_bias_y: -0.02
    accel_bias_z: 0.05
    gyro_bias_x: 0.001
    gyro_bias_y: -0.002
    gyro_bias_z: 0.003

  noise_matching:
    accel_noise_density: 0.001  # m/s^2/sqrt(Hz)
    gyro_noise_density: 0.0001  # rad/s/sqrt(Hz)
```

### 2. Validation Procedures

#### Calibration Validation
```python
# calibration_validator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import Float32
import numpy as np

class CalibrationValidator(Node):
    def __init__(self):
        super().__init__('calibration_validator')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publishers
        self.calibration_score_pub = self.create_publisher(Float32, '/calibration_score', 10)

        self.get_logger().info('Calibration Validator initialized')

    def image_callback(self, msg):
        """Validate camera calibration"""
        # Check for calibration pattern detection
        # Calculate reprojection error
        pass

    def scan_callback(self, msg):
        """Validate LiDAR calibration"""
        # Check for consistent range measurements
        # Validate angular resolution
        pass

    def imu_callback(self, msg):
        """Validate IMU calibration"""
        # Check for bias removal
        # Validate noise characteristics
        pass

    def calculate_calibration_score(self):
        """Calculate overall calibration quality score"""
        # Implementation for calculating calibration quality
        score = 0.9  # Placeholder
        score_msg = Float32()
        score_msg.data = float(score)
        self.calibration_score_pub.publish(score_msg)

def main(args=None):
    rclpy.init(args=args)
    validator = CalibrationValidator()
    timer = validator.create_timer(1.0, validator.calculate_calibration_score)
    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This guide provides the essential procedures for calibrating real-world sensors to match simulation parameters for effective sim-to-real transfer.