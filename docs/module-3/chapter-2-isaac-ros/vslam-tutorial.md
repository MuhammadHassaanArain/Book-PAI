# VSLAM Tutorial: Mapping and Localization with Isaac ROS

## Overview

This tutorial provides a comprehensive guide to implementing Visual Simultaneous Localization and Mapping (VSLAM) using NVIDIA Isaac ROS on Jetson platforms. The tutorial covers both simulation and real-world deployment scenarios, focusing on achieving real-time performance with high accuracy.

## Prerequisites

### Hardware Requirements
- NVIDIA Jetson Orin Nano/NX or higher
- RGB-D camera (Intel RealSense, ZED, or equivalent)
- 8GB+ RAM (16GB recommended)
- Sufficient storage for map data

### Software Requirements
- ROS 2 Humble Hawksbill
- Isaac ROS Visual SLAM package
- Isaac ROS Common packages
- CUDA and TensorRT properly configured

## Understanding VSLAM Concepts

### Simultaneous Localization and Mapping (SLAM)
SLAM is a technique that allows a robot to simultaneously build a map of an unknown environment while tracking its own location within that map. Visual SLAM (VSLAM) uses visual sensors (cameras) as the primary input for this process.

### Key VSLAM Components
1. **Feature Detection**: Identifying distinctive points in images
2. **Feature Tracking**: Following features across image sequences
3. **Pose Estimation**: Calculating camera/robot position and orientation
4. **Mapping**: Building a representation of the environment
5. **Loop Closure**: Recognizing previously visited locations

## Isaac ROS Visual SLAM Architecture

### Core Components
- **VisualSlamNode**: Main VSLAM processing node
- **Feature Detection**: Extracts visual features from images
- **Feature Tracking**: Tracks features across frames
- **Pose Estimation**: Calculates camera pose relative to map
- **Mapping**: Maintains and updates the map of the environment

### Data Flow
```
Camera Input → Feature Detection → Feature Tracking → Pose Estimation → Mapping → Output
```

## Installation and Setup

### Install Isaac ROS VSLAM Package
```bash
# Update package lists
sudo apt update

# Install Isaac ROS Visual SLAM package
sudo apt install -y ros-humble-isaac-ros-visual-slam

# Install additional dependencies
sudo apt install -y ros-humble-isaac-ros-common ros-humble-isaac-ros-nitros
```

### Verify Installation
```bash
# Check if package is installed
dpkg -l | grep isaac-ros-visual-slam

# Check available launch files
find /opt/ros/humble/share -name "*visual_slam*" -type d
```

## Basic VSLAM Configuration

### Create Configuration File
```yaml
# vslam_config.yaml
visual_slam:
  ros__parameters:
    # Enable rectified pose output
    enable_rectified_pose: true

    # Frame IDs
    map_frame: "map"
    odometry_frame: "odom"
    base_frame: "base_link"

    # Visualization options
    enable_observations_view: true
    enable_slam_visualization: true
    enable_landmarks_view: true
    enable_intra_proc: true

    # VSLAM parameters
    max_num_landmarks: 1000
    min_num_images_per_keyframe: 3
    min_num_images_per_graph_edge: 3
    max_num_features: 1000
    min_num_keyframes_for_local_ba: 5
    max_num_keyframes_in_local_ba: 15
    max_num_keyframes_in_global_ba: 50

    # Localization parameters
    enable_localization: true
    enable_mapping: true
```

## Launch VSLAM in Simulation

### Isaac Sim Integration

#### Create VSLAM Scene Setup Script
```python
# setup_vslam_scene.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera
from omni.isaac.range_sensor import RotatingLidarSensor
import numpy as np

def setup_vslam_scene():
    # Initialize world
    world = World(stage_units_in_meters=1.0)

    # Add a simple environment
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets path")
        return None

    # Add a simple room environment
    room_path = assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
    add_reference_to_stage(usd_path=room_path, prim_path="/World/Room")

    # Add a robot platform
    robot_path = assets_root_path + "/Isaac/Robots/Carter/carter_vda5.usd"
    add_reference_to_stage(usd_path=robot_path, prim_path="/World/Robot")

    # Add RGB camera for VSLAM
    camera = Camera(
        prim_path="/World/Robot/chassis/camera",
        name="vslam_camera",
        translation=np.array([0.2, 0.0, 0.5]),  # Position on robot
        orientation=np.array([0, 0, 0, 1])
    )

    # Set camera properties for VSLAM
    camera.set_focal_length(24.0)
    camera.set_resolution((640, 480))

    return world

def main():
    world = setup_vslam_scene()
    if world is None:
        return

    # Reset and step the world
    world.reset()

    # Run simulation for VSLAM testing
    for i in range(1000):
        world.step(render=True)

        if i % 100 == 0:
            print(f"Simulation step: {i}")

if __name__ == "__main__":
    main()
```

### Launch Isaac Sim with ROS Bridge
```bash
# Terminal 1: Launch Isaac Sim
cd ~/.local/share/ov/pkg/isaac_sim-*
./isaac-sim.sh

# Terminal 2: Run the VSLAM scene setup
source /opt/ros/humble/setup.bash
python3 setup_vslam_scene.py
```

## Isaac ROS VSLAM Launch Files

### Basic VSLAM Launch File
```python
# vslam_basic_launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Create container for VSLAM components
    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Visual SLAM node
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'enable_rectified_pose': True,
                    'map_frame': 'map',
                    'odometry_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_observations_view': True,
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_intra_proc': True
                }],
                remappings=[
                    ('/visual_slam/image_raw', '/camera/image_raw'),
                    ('/visual_slam/camera_info', '/camera/camera_info'),
                    ('/visual_slam/imu', '/imu/data')
                ]
            ),
            # Image rectification node
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect')
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([vslam_container])
```

### Advanced VSLAM Launch File with IMU Integration
```python
# vslam_imu_launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Create container for VSLAM with IMU
    vslam_imu_container = ComposableNodeContainer(
        name='vslam_imu_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Visual SLAM node with IMU
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'enable_rectified_pose': True,
                    'map_frame': 'map',
                    'odometry_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_observations_view': True,
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_intra_proc': True
                }],
                remappings=[
                    ('/visual_slam/image_raw', '/camera/image_rect'),
                    ('/visual_slam/camera_info', '/camera/camera_info_rect'),
                    ('/visual_slam/imu', '/imu/data')
                ]
            ),
            # IMU Preprocessor
            ComposableNode(
                package='isaac_ros_imu_bmi088',
                plugin='nvidia::isaac_ros::imu_bmi088::IMUBMI088Node',
                name='imu_preprocessor',
                parameters=[{
                    'use_acceleration': True,
                    'use_gyroscope': True,
                    'frame_id': 'imu_link'
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([vslam_imu_container])
```

## Real-World VSLAM Deployment

### Camera Calibration

#### Calibration Pattern Setup
```bash
# Install camera calibration tools
sudo apt install ros-humble-camera-calibration

# Launch camera calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera
```

#### Calibration File Format
```yaml
# camera_calibration.yaml
camera_name: vslam_camera
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [615.179443, 0.000000, 320.500000, 0.000000, 615.179443, 240.500000, 0.000000, 0.000000, 1.000000]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [615.179443, 0.000000, 320.500000, 0.000000, 0.000000, 615.179443, 240.500000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```

### Real Camera Integration

#### USB Camera Node
```python
# usb_camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')

        # Create publisher for camera images
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize camera
        self.cap = cv2.VideoCapture(0)  # Use first available camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Timer for capturing images
        self.timer = self.create_timer(0.033, self.capture_image)  # ~30 FPS

        # Load camera calibration
        self.load_camera_info()

        self.get_logger().info('USB Camera Node initialized')

    def load_camera_info(self):
        # Load camera calibration parameters
        # In a real implementation, load from file or parameter server
        self.camera_info = CameraInfo()
        self.camera_info.width = 640
        self.camera_info.height = 480
        self.camera_info.k = [615.179443, 0.0, 320.5, 0.0, 615.179443, 240.5, 0.0, 0.0, 1.0]

    def capture_image(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_link'

            # Publish image and camera info
            self.image_pub.publish(ros_image)
            self.camera_info.header = ros_image.header
            self.info_pub.publish(self.camera_info)
        else:
            self.get_logger().warn('Failed to capture image from camera')

def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down USB Camera Node')
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## VSLAM Performance Optimization

### GPU Acceleration Configuration

#### TensorRT Optimization
```python
# tensorrt_vslam_config.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cuda import cuda, cudart

class TensorRTVSLAMNode(Node):
    def __init__(self):
        super().__init__('tensorrt_vslam')

        # Subscribe to camera input
        self.image_sub = self.create_subscription(
            Image, '/camera/image_rect', self.image_callback, 10)

        # Initialize TensorRT engine
        self.initialize_tensorrt()

        self.get_logger().info('TensorRT VSLAM Node initialized')

    def initialize_tensorrt(self):
        # Initialize CUDA
        cuda.cuInit(0)
        self.get_logger().info('CUDA initialized for TensorRT acceleration')

    def image_callback(self, msg):
        # Process image using TensorRT-accelerated VSLAM
        # This is a simplified example - actual implementation would involve
        # loading a TensorRT engine and performing inference
        self.get_logger().info(f'Processing image with shape: {msg.width}x{msg.height}')

def main(args=None):
    rclpy.init(args=args)
    node = TensorRTVSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Memory Management

#### Optimized VSLAM Parameters
```yaml
# optimized_vslam_config.yaml
visual_slam:
  ros__parameters:
    # Memory optimization parameters
    max_num_landmarks: 500      # Reduce landmark count to save memory
    max_num_features: 500       # Reduce feature count
    min_num_keyframes_for_local_ba: 3  # Reduce optimization window

    # Performance parameters
    enable_rectified_pose: true
    enable_observations_view: false  # Disable visualization to save resources
    enable_slam_visualization: false
    enable_landmarks_view: false

    # Frame rate optimization
    max_frame_rate: 15.0        # Limit processing rate for stability
```

## Mapping and Localization Tasks

### Creating a Map

#### Mapping Mode Launch
```python
# mapping_mode_launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Container for mapping mode
    mapping_container = ComposableNodeContainer(
        name='vslam_mapping_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam_mapper',
                parameters=[{
                    'enable_rectified_pose': True,
                    'map_frame': 'map',
                    'odometry_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_observations_view': True,
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_intra_proc': True,
                    'enable_mapping': True,        # Enable mapping
                    'enable_localization': False   # Disable localization for mapping
                }],
                remappings=[
                    ('/visual_slam/image_raw', '/camera/image_rect'),
                    ('/visual_slam/camera_info', '/camera/camera_info_rect'),
                    ('/visual_slam/imu', '/imu/data')
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([mapping_container])
```

### Localizing in an Existing Map

#### Localization Mode Launch
```python
# localization_mode_launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Container for localization mode
    localization_container = ComposableNodeContainer(
        name='vslam_localization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam_localizer',
                parameters=[{
                    'enable_rectified_pose': True,
                    'map_frame': 'map',
                    'odometry_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_observations_view': False,
                    'enable_slam_visualization': False,
                    'enable_landmarks_view': False,
                    'enable_intra_proc': True,
                    'enable_mapping': False,       # Disable mapping
                    'enable_localization': True    # Enable localization
                }],
                remappings=[
                    ('/visual_slam/image_raw', '/camera/image_rect'),
                    ('/visual_slam/camera_info', '/camera/camera_info_rect'),
                    ('/visual_slam/imu', '/imu/data')
                ]
            )
        ],
        output='screen'
    )

    return LaunchDescription([localization_container])
```

## Visualization and Debugging

### RViz2 Configuration for VSLAM

#### VSLAM RViz Configuration
```yaml
# vslam_rviz_config.rviz
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /TF1
        - /Image1
        - /PointCloud21
        - /Path1
        - /PoseArray1
      Splitter Ratio: 0.5833333134651184
    Tree Height: 865
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /camera/image_rect
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud2
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visual_slam/landmarks
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Path
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visual_slam/trajectory
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.7853981852531433
      Target Frame: base_link
      Value: Orbit (rviz)
      Yaw: 0.7853981852531433
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1043
  Hide Left Dock: false
  Hide Right Dock: true
  Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001560000039bfc0200000009fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000039b000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb0000000a0049006d00610067006501000002cd000001760000002800fffffffb0000000a0049006d00610067006501000002cd000001760000000000000000fb0000000a0049006d00610067006501000002cd000001760000000000000000fb0000000a0049006d00610067006501000002cd000001760000000000000000fb0000000a0049006d00610067006501000002cd000001760000000000000000000000010000010f0000039bfc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d0000039b000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000005d30000039b00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: true
  Width: 1853
  X: 67
  Y: 27
```

## Performance Evaluation

### Benchmarking VSLAM Performance

#### Performance Monitoring Node
```python
# vslam_performance_monitor.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import time
import numpy as np

class VSLAMPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('vslam_performance_monitor')

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped, '/visual_slam/pose', self.pose_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers for performance metrics
        self.fps_pub = self.create_publisher(Float32, '/vslam/fps', 10)
        self.trajectory_accuracy_pub = self.create_publisher(Float32, '/vslam/accuracy', 10)

        # Performance tracking
        self.frame_times = []
        self.poses = []
        self.start_time = time.time()

        # Timer for performance reporting
        self.report_timer = self.create_timer(1.0, self.report_performance)

        self.get_logger().info('VSLAM Performance Monitor initialized')

    def image_callback(self, msg):
        current_time = time.time()

        # Track frame intervals
        if hasattr(self, 'last_frame_time'):
            frame_interval = current_time - self.last_frame_time
            self.frame_times.append(frame_interval)

            # Keep only recent measurements
            if len(self.frame_times) > 30:  # Last 30 frames for FPS calculation
                self.frame_times.pop(0)

        self.last_frame_time = current_time

    def pose_callback(self, msg):
        # Store poses for trajectory analysis
        self.poses.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                          msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

        # Keep only recent poses
        if len(self.poses) > 100:
            self.poses.pop(0)

    def report_performance(self):
        # Calculate FPS
        if len(self.frame_times) > 0:
            avg_frame_time = np.mean(self.frame_times)
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0

            fps_msg = Float32()
            fps_msg.data = float(fps)
            self.fps_pub.publish(fps_msg)

            self.get_logger().info(f'VSLAM Performance - FPS: {fps:.2f}')

        # Calculate trajectory metrics
        if len(self.poses) > 1:
            # Calculate trajectory smoothness and other metrics
            total_distance = 0.0
            for i in range(1, len(self.poses)):
                dx = self.poses[i][1] - self.poses[i-1][1]
                dy = self.poses[i][2] - self.poses[i-1][2]
                dz = self.poses[i][3] - self.poses[i-1][3]
                distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                total_distance += distance

            accuracy_msg = Float32()
            accuracy_msg.data = float(total_distance)
            self.trajectory_accuracy_pub.publish(accuracy_msg)

def main(args=None):
    rclpy.init(args=args)
    monitor = VSLAMPerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Performance monitor shutting down')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting Common Issues

### 1. Poor Tracking Performance
**Issue**: VSLAM loses track frequently or has poor pose estimation
**Solutions**:
- Ensure adequate lighting conditions
- Verify camera calibration is accurate
- Check for sufficient visual features in the environment
- Adjust VSLAM parameters for your specific environment

### 2. High CPU/GPU Usage
**Issue**: VSLAM consumes excessive computational resources
**Solutions**:
- Reduce image resolution
- Lower frame rate
- Decrease number of tracked features
- Use optimized parameters for embedded deployment

### 3. Drift in Mapping
**Issue**: Map becomes increasingly inaccurate over time
**Solutions**:
- Enable loop closure detection
- Use IMU data for better pose estimation
- Implement proper optimization strategies
- Regularly relocalize using known landmarks

### 4. Initialization Problems
**Issue**: VSLAM fails to initialize properly
**Solutions**:
- Ensure camera is properly calibrated
- Check that camera topics are being published
- Verify IMU data is available if used
- Confirm all required transforms are published

## Advanced Topics

### Multi-Camera VSLAM
```python
# For stereo camera setups, Isaac ROS provides stereo VSLAM capabilities
# This can improve depth estimation and tracking robustness
```

### Integration with Navigation
```python
# VSLAM can be integrated with Nav2 for localization in mapped environments
# The VSLAM pose can serve as an alternative to AMCL for visual-based localization
```

This comprehensive tutorial provides the foundation for implementing and deploying VSLAM systems using Isaac ROS, covering both simulation and real-world scenarios with performance optimization techniques.