# Sensor Pipeline Calibration Tutorial

## Overview

This tutorial provides a comprehensive guide to calibrating sensor pipelines in Isaac ROS, covering camera, IMU, and multi-sensor fusion calibration. Proper calibration is essential for accurate perception and navigation in robotics applications.

## Prerequisites

### Hardware Requirements
- Camera (monocular, stereo, or RGB-D)
- IMU sensor (internal or external)
- Calibration target (checkerboard pattern)
- Stable mounting platform
- Computer with ROS 2 Humble and Isaac ROS

### Software Requirements
- ROS 2 Humble Hawksbill
- Isaac ROS packages
- Camera calibration tools
- Image processing libraries

## Camera Calibration

### 1. Understanding Camera Calibration

#### Pinhole Camera Model
The pinhole camera model describes the mathematical relationship between 3D world coordinates and 2D image coordinates. Calibration determines the intrinsic and extrinsic parameters:

- **Intrinsic parameters**: Internal camera properties (focal length, principal point, distortion coefficients)
- **Extrinsic parameters**: Camera position and orientation relative to world coordinates

#### Distortion Models
Common distortion models include:
- **Radial distortion**: Barrel and pincushion effects
- **Tangential distortion**: Due to lens and sensor misalignment

### 2. Checkerboard Pattern Setup

#### Pattern Creation
```bash
# Create a checkerboard pattern for calibration
# Recommended: 8x6 inner corners, 108mm square size

# You can print a checkerboard pattern or use a digital display
# Pattern should have high contrast and precise geometry
```

#### Image Acquisition Guidelines
- Capture images from different angles and distances
- Ensure good lighting conditions
- Cover the entire field of view
- Avoid motion blur
- Use tripod for consistent positioning

### 3. Camera Calibration Process

#### Using ROS Camera Calibration Tool
```bash
# Install camera calibration package
sudo apt install ros-humble-camera-calibration

# Launch calibration tool
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera
```

#### Calibration Parameters
```yaml
# Example calibration file: camera_info.yaml
camera_name: calibrated_camera
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [615.179443, 0.000000, 320.500000, 0.000000, 615.179443, 240.500000, 0.000000, 0.000000, 1.000000]
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
  data: [615.179443, 0.000000, 320.500000, 0.000000, 0.000000, 615.179443, 240.500000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```

#### Isaac ROS Camera Calibration Node
```python
# camera_calibration_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class IsaacROSCameraCalibrator(Node):
    def __init__(self):
        super().__init__('isaac_ros_camera_calibrator')

        # Parameters
        self.declare_parameter('pattern_size', [8, 6])
        self.declare_parameter('square_size', 0.108)  # meters
        self.declare_parameter('calibration_flags', 0)

        self.pattern_size = self.get_parameter('pattern_size').get_parameter_value().integer_array_value
        self.square_size = self.get_parameter('square_size').get_parameter_value().double_value
        self.flags = self.get_parameter('calibration_flags').get_parameter_value().integer_value

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.calibrated_pub = self.create_publisher(Image, '/camera/image_rect', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info_rect', 10)

        # Internal variables
        self.bridge = CvBridge()
        self.object_points = []  # 3D points in real world space
        self.image_points = []   # 2D points in image plane
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_complete = False

        self.get_logger().info('Isaac ROS Camera Calibrator initialized')

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not self.calibration_complete:
            # Perform calibration pattern detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, tuple(self.pattern_size), None)

            if ret:
                # Refine corner positions
                refined_corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), self.criteria)

                # Store calibration points
                self.store_calibration_points(gray, refined_corners)

                # Draw and display the corners
                cv2.drawChessboardCorners(cv_image, tuple(self.pattern_size), refined_corners, ret)
                cv2.imshow('Calibration', cv_image)
                cv2.waitKey(1)

                self.get_logger().info('Calibration pattern detected')
            else:
                cv2.imshow('Calibration', cv_image)
                cv2.waitKey(1)
        else:
            # Apply rectification if calibration is complete
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                h, w = cv_image.shape[:2]
                new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(
                    self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))

                # Undistort image
                dst = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs, None, new_camera_mtx)

                # Crop the image
                x, y, w, h = roi
                dst = dst[y:y+h, x:x+w]

                # Publish rectified image
                rectified_msg = self.bridge.cv2_to_imgmsg(dst, encoding='bgr8')
                rectified_msg.header = msg.header
                self.calibrated_pub.publish(rectified_msg)

    def store_calibration_points(self, gray_image, corners):
        """Store calibration points for later processing"""
        # Create 3D points (real world coordinates)
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size  # Scale to real world size

        self.object_points.append(objp)
        self.image_points.append(corners.reshape(-1, 2))

    def perform_calibration(self):
        """Perform camera calibration using collected points"""
        if len(self.object_points) < 10:
            self.get_logger().warn('Need at least 10 calibration images')
            return False

        h, w = cv2.imread('sample_image.jpg').shape[:2] if cv2.imread('sample_image.jpg') is not None else (480, 640)

        ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.object_points, self.image_points, (w, h), None, None)

        if ret:
            self.get_logger().info('Camera calibration successful')
            self.get_logger().info(f'Camera Matrix:\n{self.camera_matrix}')
            self.get_logger().info(f'Distortion Coefficients:\n{self.dist_coeffs}')

            # Calculate reprojection error
            total_error = 0
            for i in range(len(self.object_points)):
                imgpoints2, _ = cv2.projectPoints(
                    self.object_points[i], rvecs[i], tvecs[i],
                    self.camera_matrix, self.dist_coeffs)
                error = cv2.norm(self.image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                total_error += error

            avg_error = total_error / len(self.object_points)
            self.get_logger().info(f'Average reprojection error: {avg_error:.4f} pixels')

            self.calibration_complete = True
            cv2.destroyAllWindows()
            return True
        else:
            self.get_logger().error('Camera calibration failed')
            return False

def main(args=None):
    rclpy.init(args=args)
    calibrator = IsaacROSCameraCalibrator()

    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        # Perform final calibration when interrupted
        if not calibrator.calibration_complete:
            calibrator.perform_calibration()
        calibrator.get_logger().info('Camera calibration process stopped')
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Stereo Camera Calibration

### 1. Stereo Calibration Process

#### Stereo Calibration Setup
```bash
# For stereo cameras, use stereo calibration tool
ros2 run camera_calibration stereo_calibrator --size 8x6 --square 0.108 left:=/camera/left/image_raw right:=/camera/right/image_raw left_camera:=/camera/left right_camera:=/camera/right
```

#### Stereo Calibration Parameters
```yaml
# stereo_calibration.yaml
left:
  camera_name: left_camera
  # ... left camera calibration parameters ...
right:
  camera_name: right_camera
  # ... right camera calibration parameters ...
stereo:
  # Extrinsics - relationship between cameras
  R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Rotation matrix
  T: [-0.12, 0.0, 0.0]  # Translation vector (baseline)
  # Stereo rectification parameters
  R1: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Left rectification
  R2: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Right rectification
  P1: [615.179443, 0.0, 320.5, 0.0, 0.0, 615.179443, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]  # Left projection
  P2: [615.179443, 0.0, 320.5, -74.43671265, 0.0, 615.179443, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]  # Right projection
```

## IMU Calibration

### 1. Understanding IMU Calibration

IMU calibration addresses several types of errors:
- **Bias**: Constant offset in measurements
- **Scale factor**: Scaling errors in measurement units
- **Non-orthogonality**: Misalignment between sensor axes
- **Cross-coupling**: Interference between different measurement axes

### 2. IMU Calibration Process

#### Static Calibration
```python
# imu_static_calibration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from scipy.optimize import minimize

class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Calibration parameters
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.accel_scale = np.ones(3)
        self.gyro_scale = np.ones(3)

        # Data collection
        self.accel_data = []
        self.gyro_data = []
        self.calibration_samples = 1000
        self.stationary_samples = 0

        self.get_logger().info('IMU Calibrator initialized. Keep IMU stationary for calibration.')

    def imu_callback(self, msg):
        # Collect stationary IMU data for bias estimation
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Check if IMU is stationary (low angular velocity)
        if np.linalg.norm(gyro) < 0.05:  # Threshold for stationary
            self.accel_data.append(accel)
            self.gyro_data.append(gyro)
            self.stationary_samples += 1

            if self.stationary_samples >= self.calibration_samples:
                self.compute_calibration()
                self.stationary_samples = 0  # Reset for continuous calibration

    def compute_calibration(self):
        """Compute IMU calibration parameters"""
        if len(self.accel_data) == 0 or len(self.gyro_data) == 0:
            return

        # Compute bias values
        accel_avg = np.mean(self.accel_data, axis=0)
        gyro_avg = np.mean(self.gyro_data, axis=0)

        # Expected gravity on Z-axis (assuming IMU is upright)
        expected_gravity = np.array([0.0, 0.0, 9.81])
        self.accel_bias = accel_avg - expected_gravity
        self.gyro_bias = gyro_avg  # Gyro should average to 0 when stationary

        self.get_logger().info(f'Computed biases - Accel: {self.accel_bias}, Gyro: {self.gyro_bias}')

        # Clear data for next calibration cycle
        self.accel_data.clear()
        self.gyro_data.clear()

    def calibrate_with_multiple_orientations(self):
        """Advanced calibration using multiple orientations"""
        # This would involve collecting data in multiple known orientations
        # and solving for scale factors and alignment matrices
        pass

def main(args=None):
    rclpy.init(args=args)
    calibrator = IMUCalibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multi-Sensor Calibration

### 1. Camera-IMU Calibration

#### Understanding Camera-IMU Calibration
Camera-IMU calibration determines the spatial and temporal relationship between visual and inertial sensors, which is crucial for:
- Visual-inertial odometry
- Sensor fusion algorithms
- Motion compensation

#### Calibration Process
```python
# camera_imu_calibrator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import cv2

class CameraIMUCalibrator(Node):
    def __init__(self):
        super().__init__('camera_imu_calibrator')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # TF broadcaster for calibration result
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Calibration variables
        self.image_buffer = []
        self.imu_buffer = []
        self.calibration_complete = False

        self.get_logger().info('Camera-IMU Calibrator initialized')

    def image_callback(self, msg):
        if not self.calibration_complete:
            # Store image with timestamp for synchronization
            self.image_buffer.append(msg)
            # Keep only recent images
            if len(self.image_buffer) > 100:
                self.image_buffer.pop(0)

    def imu_callback(self, msg):
        if not self.calibration_complete:
            # Store IMU data with timestamp
            self.imu_buffer.append(msg)
            # Keep only recent data
            if len(self.imu_buffer) > 1000:
                self.imu_buffer.pop(0)

    def perform_camera_imu_calibration(self):
        """Perform camera-IMU extrinsic calibration"""
        # This is a simplified example - actual implementation would use
        # specialized calibration tools like Kalibr or VINS-Calib

        # The process typically involves:
        # 1. Collecting synchronized camera and IMU data
        # 2. Detecting calibration patterns in images
        # 3. Estimating relative pose between sensors
        # 4. Optimizing extrinsic parameters

        # For Isaac ROS, you would typically use external tools
        # like the Kalibr toolbox or Isaac ROS calibration packages

        # Example result (in practice, this would come from actual calibration)
        translation = [0.05, 0.02, 0.01]  # meters
        rotation_quaternion = [0.0, 0.0, 0.0, 1.0]  # identity rotation

        self.publish_calibration_transform(translation, rotation_quaternion)

    def publish_calibration_transform(self, translation, rotation):
        """Publish the calibrated transform"""
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'imu_link'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    calibrator = CameraIMUCalibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Isaac ROS Calibration Nodes

### 1. Isaac ROS Calibration Packages

#### Installing Calibration Packages
```bash
# Install Isaac ROS calibration packages
sudo apt install ros-humble-isaac-ros-camera-calibration
sudo apt install ros-humble-isaac-ros-stereo-image-pipeline
```

#### Using Isaac ROS Calibration Nodes
```python
# isaac_ros_calibration_launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Container for calibration nodes
    calibration_container = ComposableNodeContainer(
        name='calibration_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Camera rectification node (uses calibration parameters)
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect'),
                ]
            ),
            # Stereo rectification (if using stereo camera)
            ComposableNode(
                package='isaac_ros_stereo_image_proc',
                plugin='nvidia::isaac_ros::stereo_image_proc::StereoRectifyNode',
                name='stereo_rectify_node',
                remappings=[
                    ('left/image_raw', '/camera/left/image_raw'),
                    ('right/image_raw', '/camera/right/image_raw'),
                    ('left/camera_info', '/camera/left/camera_info'),
                    ('right/camera_info', '/camera/right/camera_info'),
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([calibration_container])
```

## Calibration Validation

### 1. Validation Techniques

#### Reprojection Error Validation
```python
# validation_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CalibrationValidator(Node):
    def __init__(self):
        super().__init__('calibration_validator')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_rect', self.image_callback, 10)

        # Internal variables
        self.bridge = CvBridge()
        self.pattern_size = (8, 6)
        self.square_size = 0.108  # meters

        self.get_logger().info('Calibration Validator initialized')

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect calibration pattern to validate calibration
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray, self.pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret:
            # Pattern detected - calibration is working
            cv2.drawChessboardCorners(cv_image, self.pattern_size, corners, ret)
            cv2.putText(cv_image, 'CALIBRATION VALID', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            # Pattern not detected properly
            cv2.putText(cv_image, 'CALIBRATION ISSUE', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Display validation result
        cv2.imshow('Calibration Validation', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    validator = CalibrationValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Calibration validation stopped')
    finally:
        cv2.destroyAllWindows()
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Performance Monitoring

#### Calibration Quality Metrics
```python
# calibration_metrics.py
import numpy as np

class CalibrationMetrics:
    def __init__(self):
        self.metrics = {}

    def calculate_reprojection_error(self, object_points, image_points, camera_matrix, dist_coeffs, rvecs, tvecs):
        """Calculate reprojection error for camera calibration"""
        total_error = 0
        for i in range(len(object_points)):
            imgpoints2, _ = cv2.projectPoints(
                object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
            error = cv2.norm(image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            total_error += error
        return total_error / len(object_points)

    def calculate_calibration_quality(self, reprojection_error):
        """Rate calibration quality based on reprojection error"""
        if reprojection_error < 0.5:
            return "Excellent"
        elif reprojection_error < 1.0:
            return "Good"
        elif reprojection_error < 2.0:
            return "Acceptable"
        else:
            return "Poor - Recalibration recommended"

    def validate_stereo_calibration(self, E, F, left_points, right_points):
        """Validate stereo calibration using epipolar constraint"""
        # Check epipolar constraint: x'^T * F * x = 0
        errors = []
        for i in range(len(left_points)):
            x1 = np.array([left_points[i][0], left_points[i][1], 1])
            x2 = np.array([right_points[i][0], right_points[i][1], 1])
            error = x2.T @ F @ x1
            errors.append(abs(error))

        mean_error = np.mean(errors)
        return mean_error < 0.01  # Threshold for good stereo calibration
```

## Practical Calibration Workflow

### 1. Complete Calibration Process

#### Step-by-Step Calibration Guide
```bash
# Step 1: Camera Calibration
# 1. Print or display calibration pattern
# 2. Capture 20-30 images from different angles
# 3. Run calibration tool
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera

# Step 2: Save calibration file
# After successful calibration, save the YAML file to:
# /path/to/camera_info/camera_name.yaml

# Step 3: IMU Calibration (if applicable)
# Keep IMU stationary for bias estimation
# Use IMU calibration tools or custom scripts

# Step 4: Multi-sensor calibration (if applicable)
# Use tools like Kalibr for camera-IMU calibration

# Step 5: Validation
# Test calibration with validation patterns
# Check for proper rectification
```

### 2. Integration with Isaac ROS Pipelines

#### Launch File with Calibration
```python
# calibrated_pipeline_launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare launch arguments
    calibration_file = DeclareLaunchArgument(
        'camera_calibration_file',
        default_value='/path/to/calibration.yaml',
        description='Path to camera calibration file'
    )

    # Container with calibrated nodes
    calibrated_container = ComposableNodeContainer(
        name='calibrated_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Rectification using calibration
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect'),
                ]
            ),
            # Perception pipeline using rectified images
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'enable_rectified_pose': True,
                    'map_frame': 'map',
                    'odometry_frame': 'odom',
                    'base_frame': 'base_link',
                }],
                remappings=[
                    ('/visual_slam/image_raw', '/camera/image_rect'),
                    ('/visual_slam/camera_info', '/camera/camera_info_rect'),
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        calibration_file,
        calibrated_container
    ])
```

## Troubleshooting Calibration Issues

### 1. Common Problems and Solutions

#### Poor Calibration Quality
**Issue**: High reprojection error (>2 pixels)
**Solutions**:
- Capture more diverse calibration images
- Ensure good lighting conditions
- Use higher resolution images
- Verify pattern dimensions are accurate

#### Pattern Detection Failure
**Issue**: Calibration pattern not detected
**Solutions**:
- Improve lighting and contrast
- Use larger pattern or closer distance
- Clean camera lens
- Check pattern quality and flatness

#### Stereo Calibration Issues
**Issue**: Poor stereo rectification
**Solutions**:
- Ensure synchronized image capture
- Use precise stereo calibration target
- Check baseline distance is appropriate
- Verify cameras are rigidly mounted

### 2. Quality Assurance Checks

#### Pre-Deployment Validation
```bash
# Validate calibration before deployment
# 1. Check reprojection error
# 2. Verify rectification quality
# 3. Test with live data
# 4. Confirm TF transforms are published correctly
```

This comprehensive calibration tutorial provides the knowledge and tools needed to properly calibrate sensor pipelines in Isaac ROS, ensuring accurate perception and navigation for robotics applications.