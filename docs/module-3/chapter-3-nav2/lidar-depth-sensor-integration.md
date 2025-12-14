# LiDAR and Depth Sensor Integration for Obstacle Detection

## Overview

This tutorial provides comprehensive guidance on integrating LiDAR and depth sensors for obstacle detection in the Navigation2 (Nav2) stack. The tutorial covers sensor data processing, costmap integration, and obstacle detection algorithms specifically tailored for humanoid robot navigation.

## Understanding Sensor Types for Navigation

### LiDAR Sensors

#### Types of LiDAR
- **2D LiDAR**: Single-plane laser scanners (e.g., Hokuyo UTM-30LX, SICK TIM571)
- **3D LiDAR**: Multi-plane scanners (e.g., Velodyne VLP-16, Ouster OS1)
- **Solid-state LiDAR**: MEMS-based scanners (e.g., RoboSense, Hesai)

#### Advantages of LiDAR
- High accuracy and precision
- Consistent performance in various lighting conditions
- Reliable distance measurements
- Good for mapping and localization

#### Limitations of LiDAR
- Expensive compared to other sensors
- Limited ability to detect transparent or highly reflective objects
- Can miss thin objects (e.g., chains, wires)
- Performance degradation in adverse weather

### Depth Sensors

#### Types of Depth Sensors
- **Stereo Cameras**: Compute depth from multiple camera views (e.g., Intel RealSense D435)
- **Time-of-Flight (ToF)**: Measure light travel time (e.g., Microsoft Kinect, Intel RealSense L515)
- **Structured Light**: Project patterns for depth calculation (e.g., Microsoft Kinect)

#### Advantages of Depth Sensors
- Rich visual information along with depth
- Lower cost than 3D LiDAR
- Compact form factor
- Ability to detect texture and color information

#### Limitations of Depth Sensors
- Performance affected by lighting conditions
- Limited range compared to LiDAR
- Accuracy decreases with distance
- Sensitive to reflective surfaces

## Sensor Data Processing

### 1. LiDAR Data Processing

#### LaserScan Message Processing
```python
# lidar_processor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from collections import deque

class LiDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publishers
        self.obstacle_pub = self.create_publisher(MarkerArray, '/lidar_obstacles', 10)
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/lidar_pointcloud', 10)

        # Parameters
        self.declare_parameter('min_obstacle_distance', 0.5)
        self.declare_parameter('max_obstacle_distance', 5.0)
        self.declare_parameter('obstacle_threshold', 0.3)
        self.declare_parameter('min_points_for_obstacle', 5)

        self.min_obstacle_dist = self.get_parameter('min_obstacle_distance').get_parameter_value().double_value
        self.max_obstacle_dist = self.get_parameter('max_obstacle_distance').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        self.min_points_for_obstacle = self.get_parameter('min_points_for_obstacle').get_parameter_value().integer_value

        # Obstacle tracking
        self.obstacle_history = deque(maxlen=10)

        self.get_logger().info('LiDAR Processor initialized')

    def scan_callback(self, msg):
        """Process incoming LiDAR scan data"""
        # Filter scan data
        filtered_scan = self.filter_scan_data(msg)

        # Detect obstacles
        obstacles = self.detect_obstacles(filtered_scan)

        # Publish filtered scan
        self.filtered_scan_pub.publish(filtered_scan)

        # Publish obstacles as markers
        obstacle_markers = self.create_obstacle_markers(obstacles, msg.header)
        self.obstacle_pub.publish(obstacle_markers)

        # Publish point cloud representation
        pointcloud = self.scan_to_pointcloud(filtered_scan)
        self.pointcloud_pub.publish(pointcloud)

    def filter_scan_data(self, scan_msg):
        """Filter scan data to remove invalid readings"""
        ranges = np.array(scan_msg.ranges)

        # Replace invalid values (inf, nan) with max range
        ranges[np.isinf(ranges) | np.isnan(ranges)] = scan_msg.range_max
        ranges[ranges < scan_msg.range_min] = scan_msg.range_max
        ranges[ranges > scan_msg.range_max] = scan_msg.range_max

        # Create filtered scan message
        filtered_scan = LaserScan()
        filtered_scan.header = scan_msg.header
        filtered_scan.angle_min = scan_msg.angle_min
        filtered_scan.angle_max = scan_msg.angle_max
        filtered_scan.angle_increment = scan_msg.angle_increment
        filtered_scan.time_increment = scan_msg.time_increment
        filtered_scan.scan_time = scan_msg.scan_time
        filtered_scan.range_min = scan_msg.range_min
        filtered_scan.range_max = scan_msg.range_max
        filtered_scan.ranges = ranges.tolist()
        filtered_scan.intensities = scan_msg.intensities  # Preserve intensities if available

        return filtered_scan

    def detect_obstacles(self, scan_msg):
        """Detect obstacles from scan data using clustering"""
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Convert to Cartesian coordinates
        valid_indices = (ranges >= self.min_obstacle_dist) & (ranges <= self.max_obstacle_dist) & (ranges > 0)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)

        # Cluster points to identify obstacles
        obstacles = []
        if len(x_coords) > 0:
            points = np.column_stack((x_coords, y_coords))
            clusters = self.cluster_points(points)

            for cluster in clusters:
                if len(cluster) >= self.min_points_for_obstacle:
                    # Calculate obstacle center and size
                    center_x = np.mean(cluster[:, 0])
                    center_y = np.mean(cluster[:, 1])
                    size = np.std(cluster) * 2  # Approximate size

                    obstacle = {
                        'center': (center_x, center_y),
                        'size': size,
                        'points': len(cluster),
                        'reliability': min(1.0, len(cluster) / 20.0)  # Scale reliability with number of points
                    }
                    obstacles.append(obstacle)

        return obstacles

    def cluster_points(self, points):
        """Cluster points using DBSCAN-like approach"""
        if len(points) == 0:
            return []

        clusters = []
        visited = [False] * len(points)
        cluster_eps = 0.3  # Maximum distance for points to be in same cluster
        min_cluster_points = 3

        for i, point in enumerate(points):
            if visited[i]:
                continue

            # Find neighbors within cluster_eps distance
            neighbors = []
            for j, other_point in enumerate(points):
                if i != j and not visited[j]:
                    dist = np.sqrt((point[0] - other_point[0])**2 + (point[1] - other_point[1])**2)
                    if dist <= cluster_eps:
                        neighbors.append(j)

            # If enough neighbors, form a cluster
            if len(neighbors) >= min_cluster_points:
                cluster = [points[i]]
                visited[i] = True

                # Add all neighbors to cluster
                for neighbor_idx in neighbors:
                    if not visited[neighbor_idx]:
                        cluster.append(points[neighbor_idx])
                        visited[neighbor_idx] = True

                clusters.append(np.array(cluster))

        return clusters

    def create_obstacle_markers(self, obstacles, header):
        """Create visualization markers for detected obstacles"""
        marker_array = MarkerArray()

        for i, obstacle in enumerate(obstacles):
            # Create sphere marker for obstacle
            marker = Marker()
            marker.header = header
            marker.ns = "lidar_obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = obstacle['center'][0]
            marker.pose.position.y = obstacle['center'][1]
            marker.pose.position.z = 0.5  # Height for humanoid detection
            marker.pose.orientation.w = 1.0

            # Scale based on obstacle size and reliability
            scale_factor = obstacle['reliability']
            marker.scale.x = max(0.2, obstacle['size'] * scale_factor)
            marker.scale.y = max(0.2, obstacle['size'] * scale_factor)
            marker.scale.z = 1.0

            # Color based on reliability (green = high reliability)
            marker.color.r = 1.0 - obstacle['reliability']
            marker.color.g = obstacle['reliability']
            marker.color.b = 0.0
            marker.color.a = 0.7

            marker_array.markers.append(marker)

        return marker_array

    def scan_to_pointcloud(self, scan_msg):
        """Convert LaserScan to PointCloud2 for costmap processing"""
        # Convert scan to point cloud
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Filter valid ranges
        valid_mask = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & (ranges > 0)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Convert to Cartesian coordinates
        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)
        z_coords = np.zeros(len(x_coords))  # Assume ground level

        # Create PointCloud2 message
        points = np.column_stack((x_coords, y_coords, z_coords)).astype(np.float32)

        # Create PointCloud2 message manually
        from std_msgs.msg import Header
        from sensor_msgs.msg import PointCloud2, PointField
        import struct

        # Create header
        header = Header()
        header.stamp = scan_msg.header.stamp
        header.frame_id = scan_msg.header.frame_id

        # Define point fields (x, y, z)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points)
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.is_dense = False
        pointcloud_msg.point_step = 12  # 3 * 4 bytes (float32)
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width

        # Pack points into binary data
        data = []
        for point in points:
            data.append(struct.pack('fff', point[0], point[1], point[2]))

        pointcloud_msg.data = b''.join(data)

        return pointcloud_msg

def main(args=None):
    rclpy.init(args=args)
    processor = LiDARProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Depth Sensor Processing

#### Depth Image to Point Cloud Conversion
```python
# depth_processor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from sensor_msgs_py import point_cloud2
import numpy as np
import cv2
from collections import deque
import struct

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')

        # Subscriptions
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.info_callback, 10)

        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/obstacles_cloud', 10)

        # Internal variables
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None

        # Parameters
        self.declare_parameter('min_obstacle_depth', 0.3)
        self.declare_parameter('max_obstacle_depth', 5.0)
        self.declare_parameter('obstacle_threshold', 0.2)

        self.min_depth = self.get_parameter('min_obstacle_depth').get_parameter_value().double_value
        self.max_depth = self.get_parameter('max_obstacle_depth').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value

        self.get_logger().info('Depth Processor initialized')

    def info_callback(self, msg):
        """Store camera info for depth processing"""
        self.camera_info = msg

    def depth_callback(self, msg):
        """Process incoming depth image"""
        if self.camera_info is None:
            return

        try:
            # Convert ROS Image to OpenCV
            depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
            return

        # Process depth image to point cloud
        pointcloud = self.depth_to_pointcloud(depth_cv, self.camera_info)

        # Detect obstacles in point cloud
        obstacle_cloud = self.detect_obstacles_in_cloud(pointcloud)

        # Publish results
        self.pointcloud_pub.publish(pointcloud)
        if obstacle_cloud:
            self.obstacle_pub.publish(obstacle_cloud)

    def depth_to_pointcloud(self, depth_image, camera_info):
        """Convert depth image to point cloud"""
        # Get camera parameters
        width = camera_info.width
        height = camera_info.height
        ppx = camera_info.p[2]  # Principal point x
        ppy = camera_info.p[6]  # Principal point y
        fx = camera_info.p[0]   # Focal length x
        fy = camera_info.p[5]   # Focal length y

        # Create coordinate grids
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))

        # Calculate 3D coordinates
        z_coords = depth_image  # Depth values are in meters
        x_coords = (u_coords - ppx) * z_coords / fx
        y_coords = (v_coords - ppy) * z_coords / fy

        # Flatten arrays
        x_flat = x_coords.flatten()
        y_flat = y_coords.flatten()
        z_flat = z_coords.flatten()

        # Create valid mask (remove invalid depth values)
        valid_mask = (z_flat > 0) & (z_flat <= self.max_depth) & (z_flat >= self.min_depth)

        # Filter valid points
        valid_x = x_flat[valid_mask]
        valid_y = y_flat[valid_mask]
        valid_z = z_flat[valid_mask]

        # Create PointCloud2 message
        header = self.depth_sub.msg_header  # Use appropriate header
        header.stamp = self.get_clock().now().to_msg()

        # Define point fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create point cloud data
        points = []
        for i in range(len(valid_x)):
            points.append([valid_x[i], valid_y[i], valid_z[i]])

        # Create PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points)
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.is_dense = False
        pointcloud_msg.point_step = 12  # 3 * 4 bytes (float32)
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width

        # Pack points into binary data
        data = []
        for point in points:
            data.append(struct.pack('fff', point[0], point[1], point[2]))

        pointcloud_msg.data = b''.join(data)

        return pointcloud_msg

    def detect_obstacles_in_cloud(self, pointcloud_msg):
        """Detect obstacles in point cloud data"""
        # Convert PointCloud2 to list of points
        points_list = list(point_cloud2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True))

        if len(points_list) == 0:
            return None

        # Convert to numpy array
        points_array = np.array(points_list)

        # Filter points that are potential obstacles (close to robot)
        obstacle_mask = (points_array[:, 0] > 0) & (points_array[:, 0] < 2.0) & \
                       (np.abs(points_array[:, 1]) < 1.0) & \
                       (points_array[:, 2] < 2.0)  # Height threshold for humanoid

        obstacle_points = points_array[obstacle_mask]

        if len(obstacle_points) == 0:
            return None

        # Create obstacle point cloud
        header = pointcloud_msg.header
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create PointCloud2 message for obstacles
        obstacle_cloud = PointCloud2()
        obstacle_cloud.header = header
        obstacle_cloud.height = 1
        obstacle_cloud.width = len(obstacle_points)
        obstacle_cloud.fields = fields
        obstacle_cloud.is_bigendian = False
        obstacle_cloud.is_dense = False
        obstacle_cloud.point_step = 12
        obstacle_cloud.row_step = obstacle_cloud.point_step * obstacle_cloud.width

        # Pack obstacle points
        data = []
        for point in obstacle_points:
            data.append(struct.pack('fff', point[0], point[1], point[2]))

        obstacle_cloud.data = b''.join(data)

        return obstacle_cloud

def main(args=None):
    rclpy.init(args=args)
    processor = DepthProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Costmap Integration

### 1. Costmap Layer Configuration

#### Voxel Layer Configuration for 3D Data
```yaml
# voxel_layer_config.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6.0
      height: 6.0
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2      # Height resolution of voxels
        z_voxels: 10          # Number of vertical layers
        max_obstacle_height: 2.0  # Maximum height to consider
        mark_threshold: 0     # Number of points to mark cell as obstacle
        observation_sources: scan depth_camera

        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0

        depth_camera:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          min_obstacle_height: 0.0
          max_obstacle_height: 2.0
          obstacle_range: 2.5
          raytrace_range: 3.0
```

### 2. Custom Costmap Layer for Multi-Sensor Fusion

#### Multi-Sensor Costmap Layer
```python
# multi_sensor_costmap_layer.py
import rclpy
from rclpy.node import Node
from nav2_costmap_2d import Layer
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
import numpy as np
import math

class MultiSensorLayer(Layer):
    def __init__(self, name):
        super().__init__(name)
        self.layered_costmap_ = None
        self.name_ = name
        self.enabled_ = True

    def onInitialize(self):
        """Initialize the layer"""
        self.create_subscriptions()
        self.get_logger().info(f'{self.name_} layer initialized')

    def create_subscriptions(self):
        """Create subscriptions for different sensor types"""
        # LiDAR subscription
        self.scan_sub_ = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Depth camera subscription
        self.depth_sub_ = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.depth_callback, 10)

    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        # Convert scan to obstacle points
        obstacle_points = self.scan_to_obstacles(msg)

        # Update costmap with LiDAR data
        self.update_costmap_with_points(obstacle_points, sensor_type='lidar')

    def depth_callback(self, msg):
        """Process depth camera point cloud"""
        # Convert point cloud to obstacle points
        obstacle_points = self.pointcloud_to_obstacles(msg)

        # Update costmap with depth data
        self.update_costmap_with_points(obstacle_points, sensor_type='depth')

    def scan_to_obstacles(self, scan_msg):
        """Convert LaserScan to obstacle points"""
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Filter valid ranges
        valid_mask = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & (ranges > 0)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Convert to Cartesian coordinates
        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)

        # Create obstacle points
        obstacle_points = []
        for x, y in zip(x_coords, y_coords):
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0  # Ground level for LiDAR
            obstacle_points.append(point)

        return obstacle_points

    def pointcloud_to_obstacles(self, pointcloud_msg):
        """Convert PointCloud2 to obstacle points"""
        from sensor_msgs_py import point_cloud2

        # Extract points from point cloud
        points = list(point_cloud2.read_points(
            pointcloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        ))

        # Filter points based on height (for humanoid robots)
        filtered_points = []
        for point_data in points:
            x, y, z = point_data[:3]
            # Only consider obstacles within humanoid height range
            if 0.1 < z < 1.8:  # Between 10cm and 180cm height
                point = Point()
                point.x = x
                point.y = y
                point.z = z
                filtered_points.append(point)

        return filtered_points

    def update_costmap_with_points(self, obstacle_points, sensor_type='lidar'):
        """Update costmap with obstacle points"""
        if not self.enabled_:
            return

        # Get costmap dimensions and resolution
        costmap = self.layered_costmap_.getCostmap()
        resolution = costmap.getResolution()
        size_x = costmap.getSizeInCellsX()
        size_y = costmap.getSizeInCellsY()
        origin_x = costmap.getOriginX()
        origin_y = costmap.getOriginY()

        # Update costmap cells with obstacle information
        for point in obstacle_points:
            # Convert world coordinates to costmap cell coordinates
            mx = int((point.x - origin_x) / resolution)
            my = int((point.y - origin_y) / resolution)

            # Check if cell is within costmap bounds
            if 0 <= mx < size_x and 0 <= my < size_y:
                # Mark cell as obstacle with appropriate cost
                if sensor_type == 'lidar':
                    cost = 254  # High cost for LiDAR detected obstacles
                else:  # depth camera
                    cost = 200  # Medium-high cost for depth camera obstacles

                costmap.setCost(mx, my, cost)

    def updateBounds(self, robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y):
        """Update the bounds of the costmap"""
        # This method is called to determine which areas need updating
        # For now, return the current bounds
        pass

    def updateCosts(self, master_grid, min_i, min_j, max_i, max_j):
        """Update the costmap with obstacle information"""
        # This method is called to update the master costmap
        # The actual updating happens in the sensor callbacks
        pass

    def reset(self):
        """Reset the layer"""
        # Clear any accumulated data
        pass
```

## Sensor Fusion for Enhanced Obstacle Detection

### 1. Multi-Sensor Data Fusion Node

#### Sensor Fusion Implementation
```python
# sensor_fusion_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped, PoseArray
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
from scipy.spatial import cKDTree
from collections import deque

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscriptions
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.depth_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.depth_callback, 10)

        # Publishers
        self.fused_obstacles_pub = self.create_publisher(PoseArray, '/fused_obstacles', 10)
        self.confidence_pub = self.create_publisher(Float32MultiArray, '/obstacle_confidence', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, '/fused_obstacles_viz', 10)

        # Parameters
        self.declare_parameter('fusion_distance_threshold', 0.3)
        self.declare_parameter('min_consistency_measurements', 3)
        self.declare_parameter('max_consistency_time', 2.0)

        self.fusion_threshold = self.get_parameter('fusion_distance_threshold').get_parameter_value().double_value
        self.min_consistency = self.get_parameter('min_consistency_measurements').get_parameter_value().integer_value
        self.max_consistency_time = self.get_parameter('max_consistency_time').get_parameter_value().double_value

        # Data storage
        self.lidar_obstacles = deque(maxlen=10)
        self.depth_obstacles = deque(maxlen=10)
        self.fused_obstacles = deque(maxlen=20)

        self.get_logger().info('Sensor Fusion Node initialized')

    def lidar_callback(self, msg):
        """Process LiDAR data and extract obstacles"""
        # Convert LaserScan to obstacle points
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter valid ranges
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & (ranges > 0)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Convert to Cartesian coordinates
        x_coords = valid_ranges * np.cos(valid_angles)
        y_coords = valid_ranges * np.sin(valid_angles)

        # Store LiDAR obstacles with timestamp
        lidar_data = {
            'points': list(zip(x_coords, y_coords)),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

        self.lidar_obstacles.append(lidar_data)

    def depth_callback(self, msg):
        """Process depth camera data and extract obstacles"""
        from sensor_msgs_py import point_cloud2

        # Extract points from point cloud
        points = list(point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        ))

        # Filter points based on height and distance
        filtered_points = []
        for x, y, z in points:
            if 0.1 < z < 1.8 and math.sqrt(x*x + y*y) < 3.0:  # Within humanoid workspace
                filtered_points.append((x, y))

        # Store depth obstacles with timestamp
        depth_data = {
            'points': filtered_points,
            'timestamp': self.get_clock().now().nanoseconds * 1e-9
        }

        self.depth_obstacles.append(depth_data)

    def fuse_sensor_data(self):
        """Fuse LiDAR and depth sensor data"""
        if len(self.lidar_obstacles) == 0 or len(self.depth_obstacles) == 0:
            return []

        # Get the most recent sensor data
        lidar_data = self.lidar_obstacles[-1]
        depth_data = self.depth_obstacles[-1]

        # Create KD trees for efficient nearest neighbor search
        lidar_points = np.array(lidar_data['points'])
        depth_points = np.array(depth_data['points'])

        if len(lidar_points) == 0 or len(depth_points) == 0:
            return []

        # Build KD trees
        lidar_tree = cKDTree(lidar_points)
        depth_tree = cKDTree(depth_points)

        # Find consistent obstacle points between sensors
        fused_obstacles = []

        # Match LiDAR points with depth points
        lidar_distances, lidar_indices = depth_tree.query(lidar_points, distance_upper_bound=self.fusion_threshold)
        valid_matches = lidar_distances < self.fusion_threshold

        for i, is_match in enumerate(valid_matches):
            if is_match:
                # Average the positions of matched points
                lidar_point = lidar_points[i]
                depth_point = depth_points[lidar_indices[i]]

                avg_x = (lidar_point[0] + depth_point[0]) / 2.0
                avg_y = (lidar_point[1] + depth_point[1]) / 2.0

                fused_obstacles.append((avg_x, avg_y))

        # Also check depth points against LiDAR points
        depth_distances, depth_indices = lidar_tree.query(depth_points, distance_upper_bound=self.fusion_threshold)
        valid_matches = depth_distances < self.fusion_threshold

        for i, is_match in enumerate(valid_matches):
            if is_match:
                # Check if this point was already added from LiDAR matching
                depth_point = depth_points[i]
                already_added = False

                for fused_x, fused_y in fused_obstacles:
                    dist = math.sqrt((depth_point[0] - fused_x)**2 + (depth_point[1] - fused_y)**2)
                    if dist < self.fusion_threshold:
                        already_added = True
                        break

                if not already_added:
                    lidar_point = lidar_points[depth_indices[i]]
                    avg_x = (lidar_point[0] + depth_point[0]) / 2.0
                    avg_y = (lidar_point[1] + depth_point[1]) / 2.0
                    fused_obstacles.append((avg_x, avg_y))

        # Add unmatched points with lower confidence
        for i, dist in enumerate(lidar_distances):
            if dist >= self.fusion_threshold:
                fused_obstacles.append(tuple(lidar_points[i]))

        for i, dist in enumerate(depth_distances):
            if dist >= self.fusion_threshold:
                # Only add if not already present in main list
                depth_point = tuple(depth_points[i])
                is_present = any(
                    math.sqrt((dp[0] - depth_point[0])**2 + (dp[1] - depth_point[1])**2) < self.fusion_threshold
                    for dp in fused_obstacles
                )
                if not is_present:
                    fused_obstacles.append(depth_point)

        return fused_obstacles

    def publish_fused_obstacles(self):
        """Publish fused obstacle data"""
        fused_obstacles = self.fuse_sensor_data()

        if len(fused_obstacles) == 0:
            return

        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for x, y in fused_obstacles:
            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        # Publish fused obstacles
        self.fused_obstacles_pub.publish(pose_array)

        # Create and publish confidence scores
        confidence_msg = Float32MultiArray()
        confidence_msg.data = [0.9] * len(fused_obstacles)  # High confidence for fused points
        self.confidence_pub.publish(confidence_msg)

        # Create visualization markers
        marker_array = self.create_visualization_markers(fused_obstacles)
        self.visualization_pub.publish(marker_array)

    def create_visualization_markers(self, obstacles):
        """Create visualization markers for fused obstacles"""
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA

        marker_array = MarkerArray()

        for i, (x, y) in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "fused_obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5  # Half human height
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.3  # Diameter
            marker.scale.y = 0.3
            marker.scale.z = 1.0  # Height

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7

            marker_array.markers.append(marker)

        return marker_array

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()

    # Timer for periodic fusion
    timer = fusion_node.create_timer(0.1, fusion_node.publish_fused_obstacles)

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        fusion_node.get_logger().info('Sensor fusion node stopped')
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Navigation Stack

### 1. Costmap Configuration for Multi-Sensor Setup

#### Complete Multi-Sensor Costmap Configuration
```yaml
# multi_sensor_costmap_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 8.0
      height: 8.0
      resolution: 0.05
      robot_radius: 0.4  # Larger for humanoid safety
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: depth_camera
        depth_camera:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          obstacle_range: 3.0
          raytrace_range: 4.0
          transform_tolerance: 0.2
          marking: True
          clearing: True

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        cost_scaling_factor: 5.0
        inflation_radius: 0.8

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.4
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan_filtered
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 1.0
```

### 2. Launch File for Multi-Sensor Integration

#### Complete Multi-Sensor Launch
```python
# multi_sensor_nav_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default='multi_sensor_costmap_params.yaml')

    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    sensor_integration_dir = get_package_share_directory('sensor_integration_tutorials')

    # Include main Nav2 bringup with custom parameters
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup_dir,
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                sensor_integration_dir,
                'config',
                params_file
            ])
        }.items()
    )

    # Sensor processing container
    sensor_processing_container = ComposableNodeContainer(
        name='sensor_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # LiDAR processor
            ComposableNode(
                package='sensor_integration_tutorials',
                plugin='sensor_integration_tutorials::LiDARProcessor',
                name='lidar_processor',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'min_obstacle_distance': 0.3,
                    'max_obstacle_distance': 5.0,
                    'obstacle_threshold': 0.3
                }]
            ),
            # Depth processor
            ComposableNode(
                package='sensor_integration_tutorials',
                plugin='sensor_integration_tutorials::DepthProcessor',
                name='depth_processor',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'min_obstacle_depth': 0.3,
                    'max_obstacle_depth': 4.0
                }]
            ),
            # Sensor fusion
            ComposableNode(
                package='sensor_integration_tutorials',
                plugin='sensor_integration_tutorials::SensorFusionNode',
                name='sensor_fusion',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fusion_distance_threshold': 0.3
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        nav2_bringup_launch,
        sensor_processing_container
    ])
```

## Testing and Validation

### 1. Obstacle Detection Performance Metrics

#### Detection Performance Evaluator
```python
# detection_performance_evaluator.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import math

class DetectionPerformanceEvaluator(Node):
    def __init__(self):
        super().__init__('detection_performance_evaluator')

        # Subscriptions
        self.fused_obstacles_sub = self.create_subscription(
            PoseArray, '/fused_obstacles', self.fused_obstacles_callback, 10)
        self.lidar_obstacles_sub = self.create_subscription(
            PoseArray, '/lidar_obstacles', self.lidar_obstacles_callback, 10)
        self.ground_truth_sub = self.create_subscription(
            PoseArray, '/ground_truth_obstacles', self.ground_truth_callback, 10)

        # Publishers
        self.detection_rate_pub = self.create_publisher(Float32, '/detection_rate', 10)
        self.false_positive_rate_pub = self.create_publisher(Float32, '/false_positive_rate', 10)
        self.accuracy_pub = self.create_publisher(Float32, '/detection_accuracy', 10)

        # Internal variables
        self.ground_truth_obstacles = []
        self.fused_detections = []
        self.lidar_detections = []

        self.get_logger().info('Detection Performance Evaluator initialized')

    def ground_truth_callback(self, msg):
        """Receive ground truth obstacle positions"""
        self.ground_truth_obstacles = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def fused_obstacles_callback(self, msg):
        """Receive fused obstacle detections"""
        self.fused_detections = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.evaluate_performance()

    def lidar_obstacles_callback(self, msg):
        """Receive LiDAR-only obstacle detections"""
        self.lidar_detections = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def evaluate_performance(self):
        """Evaluate detection performance"""
        if not self.ground_truth_obstacles:
            return

        # Calculate detection metrics
        true_positives, false_positives, false_negatives = self.calculate_detection_metrics(
            self.fused_detections, self.ground_truth_obstacles
        )

        # Calculate metrics
        detection_rate = true_positives / len(self.ground_truth_obstacles) if self.ground_truth_obstacles else 0
        false_positive_rate = false_positives / len(self.fused_detections) if self.fused_detections else 0
        accuracy = true_positives / (true_positives + false_positives + false_negatives) if (true_positives + false_positives + false_negatives) > 0 else 0

        # Publish metrics
        detection_rate_msg = Float32()
        detection_rate_msg.data = float(detection_rate)
        self.detection_rate_pub.publish(detection_rate_msg)

        false_positive_rate_msg = Float32()
        false_positive_rate_msg.data = float(false_positive_rate)
        self.false_positive_rate_pub.publish(false_positive_rate_msg)

        accuracy_msg = Float32()
        accuracy_msg.data = float(accuracy)
        self.accuracy_pub.publish(accuracy_msg)

        self.get_logger().info(
            f'Detection Performance - Rate: {detection_rate:.3f}, '
            f'False Positive Rate: {false_positive_rate:.3f}, '
            f'Accuracy: {accuracy:.3f}'
        )

    def calculate_detection_metrics(self, detections, ground_truth, threshold=0.5):
        """Calculate detection metrics (TP, FP, FN)"""
        true_positives = 0
        false_positives = 0
        false_negatives = 0

        # Track which ground truth obstacles have been detected
        detected_gt = [False] * len(ground_truth)

        for det_x, det_y in detections:
            # Find closest ground truth obstacle
            min_dist = float('inf')
            closest_gt_idx = -1

            for i, (gt_x, gt_y) in enumerate(ground_truth):
                dist = math.sqrt((det_x - gt_x)**2 + (det_y - gt_y)**2)
                if dist < min_dist and dist < threshold:
                    min_dist = dist
                    closest_gt_idx = i

            if closest_gt_idx != -1 and not detected_gt[closest_gt_idx]:
                # True positive: detection matches a ground truth obstacle
                true_positives += 1
                detected_gt[closest_gt_idx] = True
            else:
                # False positive: detection doesn't match any ground truth
                false_positives += 1

        # Count false negatives: ground truth obstacles not detected
        false_negatives = len(ground_truth) - sum(detected_gt)

        return true_positives, false_positives, false_negatives

def main(args=None):
    rclpy.init(args=args)
    evaluator = DetectionPerformanceEvaluator()
    rclpy.spin(evaluator)
    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This comprehensive tutorial provides detailed implementation and configuration for integrating LiDAR and depth sensors for obstacle detection in the Nav2 stack, including sensor data processing, costmap integration, and multi-sensor fusion techniques specifically tailored for humanoid robot navigation.