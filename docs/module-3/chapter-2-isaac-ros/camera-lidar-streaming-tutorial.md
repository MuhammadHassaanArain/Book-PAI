# Simulated Camera and LiDAR Data Streaming Tutorial

## Overview

This tutorial demonstrates how to stream camera and LiDAR data from NVIDIA Isaac Sim to Isaac ROS perception pipelines. The integration enables training and testing of perception algorithms using photorealistic simulation data while maintaining compatibility with real-world ROS 2 interfaces.

## Prerequisites

### Software Requirements
- NVIDIA Isaac Sim installed and configured
- Isaac ROS packages installed
- ROS 2 Humble environment sourced
- Basic understanding of ROS 2 concepts

### Hardware Requirements
- NVIDIA GPU with CUDA support (RTX series recommended)
- 32GB+ RAM for complex scenes
- Sufficient VRAM for sensor simulation

## Setting Up Isaac Sim for Sensor Streaming

### 1. Create a New Scene with Sensors

#### Launch Isaac Sim
```bash
# Navigate to Isaac Sim directory
cd ~/.local/share/ov/pkg/isaac_sim-*

# Launch Isaac Sim
./isaac-sim.sh
```

#### Add Camera Sensor
1. In the Stage panel, right-click and select "Create" → "Camera"
2. Position the camera at an appropriate location (e.g., mounted on a robot)
3. In the Property panel, adjust camera parameters:
   - **Resolution**: Set to desired resolution (e.g., 640x480, 1280x720)
   - **Focal Length**: Adjust based on desired field of view
   - **Clipping Range**: Set near and far clipping planes

#### Add LiDAR Sensor
1. Create a new Xform in the Stage panel: "Create" → "Xform" → "Lidar_Xform"
2. Add a Rotator to the Xform: "Add Child" → "Rotator"
3. Add the actual LiDAR sensor: "Add Child" → "Lidar_Simple"
4. Configure LiDAR parameters in the Property panel:
   - **Range**: Maximum detection range
   - **Resolution**: Angular resolution
   - **Channels**: Number of laser channels
   - **Rotation Rate**: How fast the LiDAR rotates

### 2. Configure ROS 2 Bridge

#### Enable ROS 2 Bridge Extension
1. Go to "Window" → "Extensions"
2. Search for "ROS" extensions
3. Enable "ROS2 Bridge" extension

#### Set Up ROS 2 Bridge Configuration
```python
# Create a Python script to configure the ROS 2 bridge
# sensor_bridge_config.py

import omni
from omni.isaac.core import World
from omni.isaac.ros_bridge import RigidContactSensor, RgbdCameraHelper, RotLidarHelper
import carb

# Initialize Isaac Sim
world = World(stage_units_in_meters=1.0)

# Configure camera publisher
camera_helper = RgbdCameraHelper()
camera_helper.set_camera_parameters(
    "/World/Camera",  # Camera prim path
    "/camera",        # ROS topic namespace
    image_width=640,
    image_height=480,
    update_frame_rate=30.0
)

# Configure LiDAR publisher
lidar_helper = RotLidarHelper()
lidar_helper.set_lidar_parameters(
    "/World/Lidar_Xform",  # LiDAR prim path
    "/lidar",              # ROS topic namespace
    update_frame_rate=10.0
)

# Run simulation
world.reset()
for i in range(1000):
    world.step(render=True)
```

## Isaac Sim Sensor Configuration

### Camera Configuration

#### RGB Camera Setup
```python
# camera_setup.py
import omni
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.sensor import Camera
import numpy as np

# Get or create camera
camera = Camera(
    prim_path="/World/Camera",
    name="camera",
    translation=np.array([1.0, 0.0, 1.5]),
    orientation=np.array([0, 0, 0, 1])
)

# Set camera properties
camera.set_focal_length(24.0)
camera.set_horizontal_aperture(20.955)
camera.set_vertical_aperture(15.291)
camera.set_resolution((640, 480))

# Enable different sensor types
camera.add_raw_sensor_data_to_frame("rgb", "rgb")
camera.add_raw_sensor_data_to_frame("depth", "depth")
camera.add_raw_sensor_data_to_frame("semantic", "semantic")
```

#### Camera ROS 2 Publishers
```yaml
# camera_config.yaml
camera:
  ros__parameters:
    # Camera parameters
    image_width: 640
    image_height: 480
    frame_id: "camera_link"

    # Publisher settings
    image_topic_name: "/camera/image_raw"
    camera_info_topic_name: "/camera/camera_info"
    qos: 10

    # Performance settings
    update_rate: 30.0
    compression_format: "png"
```

### LiDAR Configuration

#### Rotating LiDAR Setup
```python
# lidar_setup.py
import omni
from omni.isaac.range_sensor import RotatingLidarSensor
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np

# Create rotating LiDAR sensor
lidar_sensor = RotatingLidarSensor(
    prim_path="/World/Lidar_Xform/Lidar_Simple",
    name="lidar_sensor",
    translation=np.array([0.0, 0.0, 0.5]),
    min_range=0.1,
    max_range=25.0,
    scan_period=0.1,  # 10Hz
    horizontal_resolution=0.25,  # degrees
    vertical_resolution=2.0,     # degrees
    horizontal_lasers=1440,      # 360 degrees / 0.25
    vertical_lasers=16,          # number of channels
    update_period=0.1,
    rotation_frequency=5,        # Hz
    offset_first_channel_angle=True,
    enable_semantics=False
)

# Configure sensor parameters
lidar_sensor.set_max_range(25.0)
lidar_sensor.set_min_range(0.1)
lidar_sensor.set_horizontal_resolution(0.25)
lidar_sensor.set_vertical_resolution(2.0)
```

#### LiDAR ROS 2 Publishers
```yaml
# lidar_config.yaml
lidar:
  ros__parameters:
    # LiDAR parameters
    frame_id: "lidar_link"

    # Publisher settings
    topic_name: "/scan"
    qos: 10

    # Performance settings
    update_rate: 10.0
    range_threshold: 25.0
    intensity_range: [0.0, 1.0]
```

## Streaming Configuration in Isaac Sim

### 1. Robotics Graph Language (RGL) Setup

#### Create RGL Graph for Camera
```python
# rgl_camera_setup.py
import omni
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.synthetic_utils import RGLGraph
import rgl

def setup_camera_rgl():
    # Create RGL graph for camera data
    graph = RGLGraph()

    # Add nodes for different data types
    graph.add_node("rgl_node_points", rgl.pts_field.XYZ)
    graph.add_node("rgl_node_intensity", rgl.pts_field.INTENSITY)
    graph.add_node("rgl_node_label", rgl.pts_field.SEMANTIC_LABEL)

    # Connect nodes
    graph.connect_nodes("rgl_node_points", "rgl_node_intensity")

    # Configure ROS 2 publishing
    graph.set_ros2_publisher("sensor_msgs/msg/LaserScan", "/scan")

    return graph

# Initialize the graph
camera_graph = setup_camera_rgl()
```

#### Create RGL Graph for LiDAR
```python
# rgl_lidar_setup.py
import rgl
from omni.synthetic_utils import RGLGraph

def setup_lidar_rgl():
    # Create RGL graph for LiDAR data
    graph = RGLGraph()

    # Define LiDAR parameters
    lidar_params = {
        "range": 25.0,
        "horizontal_fov": 360.0,
        "vertical_fov": 30.0,
        "horizontal_resolution": 0.25,
        "vertical_resolution": 2.0,
        "rotation_speed": 5.0
    }

    # Add LiDAR node
    graph.add_node("rgl_lidar", rgl.pts_field.XYZ, lidar_params)

    # Add intensity and label fields
    graph.add_node("rgl_intensity", rgl.pts_field.INTENSITY)
    graph.add_node("rgl_label", rgl.pts_field.SEMANTIC_LABEL)

    # Configure ROS 2 publishing
    graph.set_ros2_publisher("sensor_msgs/msg/PointCloud2", "/point_cloud")
    graph.set_ros2_publisher("sensor_msgs/msg/LaserScan", "/scan")

    return graph

# Initialize the graph
lidar_graph = setup_lidar_rgl()
```

### 2. ROS Bridge Configuration

#### Create ROS Bridge Script
```python
# ros_bridge_setup.py
import omni
from omni.isaac.core import World
from omni.isaac.ros_bridge import ROSBridge
import rclpy
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2
from std_msgs.msg import Header
import numpy as np

class SensorStreamer:
    def __init__(self):
        # Initialize Isaac Sim world
        self.world = World(stage_units_in_meters=1.0)

        # Initialize ROS 2
        rclpy.init()
        self.node = rclpy.create_node('sensor_streamer')

        # Create publishers
        self.image_pub = self.node.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.node.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 10)
        self.pc_pub = self.node.create_publisher(PointCloud2, '/point_cloud', 10)

        # Set up camera and LiDAR in Isaac Sim
        self.setup_sensors()

    def setup_sensors(self):
        # Configure camera
        from omni.isaac.sensor import Camera
        self.camera = Camera(
            prim_path="/World/Camera",
            name="camera",
            translation=np.array([1.0, 0.0, 1.5])
        )

        # Configure LiDAR
        from omni.isaac.range_sensor import RotatingLidarSensor
        self.lidar = RotatingLidarSensor(
            prim_path="/World/Lidar_Xform/Lidar_Simple",
            name="lidar_sensor",
            translation=np.array([0.0, 0.0, 0.5])
        )

    def stream_data(self):
        # Step the simulation
        self.world.step(render=True)

        # Get camera data
        rgb_data = self.camera.get_rgb()
        depth_data = self.camera.get_depth()

        # Get LiDAR data
        lidar_data = self.lidar.get_linear_depth_data()

        # Publish data to ROS 2 topics
        self.publish_camera_data(rgb_data, depth_data)
        self.publish_lidar_data(lidar_data)

    def publish_camera_data(self, rgb_data, depth_data):
        # Create and publish image message
        image_msg = Image()
        image_msg.header.stamp = self.node.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera_link"
        image_msg.height = rgb_data.shape[0]
        image_msg.width = rgb_data.shape[1]
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = False
        image_msg.step = rgb_data.shape[1] * 3
        image_msg.data = rgb_data.tobytes()

        self.image_pub.publish(image_msg)

    def publish_lidar_data(self, lidar_data):
        # Create and publish laser scan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.node.get_clock().now().to_msg()
        scan_msg.header.frame_id = "lidar_link"
        scan_msg.angle_min = -np.pi
        scan_msg.angle_max = np.pi
        scan_msg.angle_increment = 0.005
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 25.0
        scan_msg.ranges = lidar_data.flatten().tolist()

        self.scan_pub.publish(scan_msg)

def main():
    streamer = SensorStreamer()

    # Stream data for 1000 simulation steps
    for i in range(1000):
        streamer.stream_data()

    # Cleanup
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## Isaac ROS Perception Pipeline Integration

### 1. Camera Data Pipeline

#### Create Camera Processing Pipeline
```python
# camera_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPipeline(Node):
    def __init__(self):
        super().__init__('camera_pipeline')

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, 10)

        # Create publishers
        self.processed_pub = self.create_publisher(Image, '/camera/processed', 10)

        # Initialize CV bridge
        self.bridge = CvBridge()
        self.camera_info = None

        self.get_logger().info('Camera pipeline initialized')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Process the image (example: edge detection)
        processed_image = self.process_image(cv_image)

        # Convert back to ROS Image
        processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='rgb8')
        processed_msg.header = msg.header

        # Publish processed image
        self.processed_pub.publish(processed_msg)

    def info_callback(self, msg):
        self.camera_info = msg

    def process_image(self, image):
        # Example processing: convert to grayscale and detect edges
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        return cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)

def main(args=None):
    rclpy.init(args=args)
    pipeline = CameraPipeline()
    rclpy.spin(pipeline)
    pipeline.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. LiDAR Data Pipeline

#### Create LiDAR Processing Pipeline
```python
# lidar_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np

class LidarPipeline(Node):
    def __init__(self):
        super().__init__('lidar_pipeline')

        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/point_cloud', self.pc_callback, 10)

        # Create publishers
        self.processed_scan_pub = self.create_publisher(LaserScan, '/scan_processed', 10)
        self.obstacle_pub = self.create_publisher(LaserScan, '/obstacles', 10)

        self.get_logger().info('LiDAR pipeline initialized')

    def scan_callback(self, msg):
        # Process laser scan data
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        # Detect obstacles (ranges < 1.0m)
        obstacle_indices = np.where((ranges > msg.range_min) & (ranges < 1.0))[0]

        if len(obstacle_indices) > 0:
            # Create obstacle scan message
            obstacle_msg = LaserScan()
            obstacle_msg.header = msg.header
            obstacle_msg.angle_min = msg.angle_min
            obstacle_msg.angle_max = msg.angle_max
            obstacle_msg.angle_increment = msg.angle_increment
            obstacle_msg.time_increment = msg.time_increment
            obstacle_msg.scan_time = msg.scan_time
            obstacle_msg.range_min = msg.range_min
            obstacle_msg.range_max = msg.range_max

            # Initialize ranges array
            obstacle_msg.ranges = [float('inf')] * len(ranges)

            # Set obstacle ranges
            for idx in obstacle_indices:
                obstacle_msg.ranges[idx] = ranges[idx]

            self.obstacle_pub.publish(obstacle_msg)

    def pc_callback(self, msg):
        # Process point cloud data
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points_array = np.array(points)

        # Perform point cloud processing (example: ground plane removal)
        processed_points = self.remove_ground_plane(points_array)

        self.get_logger().info(f'Processed point cloud with {len(processed_points)} points')

    def remove_ground_plane(self, points):
        # Simple ground plane removal using height threshold
        ground_threshold = 0.1  # meters above ground
        non_ground_points = points[points[:, 2] > ground_threshold]
        return non_ground_points

def main(args=None):
    rclpy.init(args=args)
    pipeline = LidarPipeline()
    rclpy.spin(pipeline)
    pipeline.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Configuration

### 1. Isaac Sim Launch Script

#### Create Isaac Sim Configuration
```python
# launch_isaac_sim.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.carb import set_carb_setting
import carb

def setup_simulation():
    # Set Isaac Sim settings
    set_carb_setting("/persistent/isaac/asset_root/default", get_assets_root_path())
    set_carb_setting("/app/player/play_sim_on_startup", True)
    set_carb_setting("/app/window/drawMouse", True)
    set_carb_setting("/app/window/hideSafezone", True)
    set_carb_setting("/persistent/app/window/resizable", True)
    set_carb_setting("/app/window/width", 1280)
    set_carb_setting("/app/window/height", 720)

    # Initialize world
    world = World(stage_units_in_meters=1.0)

    # Add a simple scene
    add_reference_to_stage(
        usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Environments/Simple_Room/simple_room.usd",
        prim_path="/World"
    )

    # Add robot with sensors
    add_reference_to_stage(
        usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Robots/Carter/carter_vda5.usd",
        prim_path="/World/Robot"
    )

    # Add sensors to robot
    # (This would be done in the Stage panel or through API)

    return world

def main():
    # Setup simulation
    world = setup_simulation()

    # Reset and run simulation
    world.reset()

    # Run for specified number of steps
    for i in range(10000):
        world.step(render=True)

        if i % 100 == 0:
            print(f"Simulation step: {i}")

if __name__ == "__main__":
    main()
```

### 2. ROS 2 Launch Files

#### Camera Pipeline Launch
```python
# camera_pipeline_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Camera processing node
    camera_node = Node(
        package='your_package',
        executable='camera_pipeline',
        name='camera_pipeline',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/camera/image_raw', '/sim_camera/image_raw'),
            ('/camera/camera_info', '/sim_camera/camera_info'),
        ]
    )

    # Image view node for visualization
    image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='image_viewer',
        remappings=[
            ('image', '/camera/processed')
        ]
    )

    return LaunchDescription([
        camera_node,
        image_view_node
    ])
```

#### LiDAR Pipeline Launch
```python
# lidar_pipeline_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # LiDAR processing node
    lidar_node = Node(
        package='your_package',
        executable='lidar_pipeline',
        name='lidar_pipeline',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/scan', '/sim_lidar/scan'),
            ('/point_cloud', '/sim_lidar/point_cloud'),
        ]
    )

    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'path/to/lidar_config.rviz']
    )

    return LaunchDescription([
        lidar_node,
        rviz_node
    ])
```

## Testing and Validation

### 1. Verify Data Streaming

#### Check Available Topics
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# List available topics
ros2 topic list

# Check camera topics
ros2 topic list | grep camera

# Check LiDAR topics
ros2 topic list | grep scan
```

#### Echo Sensor Data
```bash
# Echo camera image info
ros2 topic echo /camera/image_raw --field header

# Echo LiDAR scan data
ros2 topic echo /scan --field header
```

### 2. Visualize Data

#### Using Image View
```bash
# View camera images
ros2 run image_view image_view image:=/camera/image_raw
```

#### Using RViz2
```bash
# Create RViz2 configuration file
# lidar_config.rviz

Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Image
      Name: Camera Image
      Topic: /camera/image_raw
    - Class: rviz_default_plugins/LaserScan
      Name: LiDAR Scan
      Topic: /scan
      Size (m): 0.1
      Color: 255; 0; 0
```

## Performance Optimization

### 1. Simulation Performance

#### Reduce Render Quality for Better Performance
```bash
# In Isaac Sim, go to Window → Render Settings
# Reduce the following settings for better performance:
# - Render Quality
# - Shadow Quality
# - Reflection Quality
# - Anti-aliasing
```

#### Optimize Sensor Settings
```python
# Reduce sensor resolution for better performance
camera.set_resolution((320, 240))  # Lower resolution
lidar_sensor.set_horizontal_resolution(0.5)  # Lower resolution
lidar_sensor.set_update_period(0.2)  # Lower update rate
```

### 2. ROS 2 Performance

#### QoS Configuration for Real-time Performance
```yaml
# qos_config.yaml
camera_publisher:
  ros__parameters:
    qos_overrides:
      /camera/image_raw:
        publisher:
          history: keep_last
          depth: 1
          reliability: reliable
          durability: volatile
          deadline:
            sec: 0
            nsec: 100000000  # 100ms deadline
```

## Troubleshooting

### 1. Connection Issues
**Issue**: ROS 2 topics not appearing when Isaac Sim is running
**Solution**:
```bash
# Check if ROS 2 bridge is enabled in Isaac Sim
# Go to Window → Extensions → ROS2 Bridge and ensure it's enabled

# Verify ROS 2 domain ID matches
echo $ROS_DOMAIN_ID  # Should match Isaac Sim setting
```

### 2. Performance Issues
**Issue**: Low frame rate or dropped frames
**Solution**:
```bash
# Reduce sensor resolution
# Lower simulation update rate
# Close other applications to free up resources
# Check GPU memory usage: nvidia-smi
```

### 3. Data Quality Issues
**Issue**: Poor quality sensor data
**Solution**:
```bash
# Verify sensor placement and orientation
# Check simulation physics settings
# Adjust sensor parameters (range, resolution, etc.)
# Ensure proper lighting in simulation scene
```

## Integration with Isaac ROS Perception

### 1. Connect to Isaac ROS Nodes

#### Example: Connect Camera to Isaac ROS AprilTag
```python
# apriltag_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

class AprilTagIntegration(Node):
    def __init__(self):
        super().__init__('apriltag_integration')

        # Subscribe to camera data from Isaac Sim
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_rect', self.camera_callback, 10)

        # Subscribe to AprilTag detections from Isaac ROS
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/isaac_ros/apriltag_detections',
            self.detection_callback, 10)

        self.get_logger().info('AprilTag integration node initialized')

    def camera_callback(self, msg):
        # Process camera data
        self.get_logger().info(f'Received camera image: {msg.width}x{msg.height}')

    def detection_callback(self, msg):
        # Process AprilTag detections
        self.get_logger().info(f'Detected {len(msg.detections)} tags')

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagIntegration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This tutorial provides a comprehensive guide to streaming camera and LiDAR data from Isaac Sim to Isaac ROS perception pipelines, enabling effective simulation-to-reality transfer for robotics perception systems.