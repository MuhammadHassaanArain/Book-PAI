---
sidebar_position: 6
---

# Mini Project: Full GPU-Accelerated Perception & Navigation Stack

## Objective

In this mini project, students will integrate all components learned in Module 3 to create a complete GPU-accelerated perception and navigation system. The project combines Isaac Sim simulation, Isaac ROS perception pipelines, Nav2 navigation, and sim-to-real transfer on Jetson hardware.

## Prerequisites

- Completed all previous labs (Lab 1-5)
- Jetson Orin Nano/NX with Ubuntu 22.04
- Isaac ROS packages installed
- Real sensors (camera, IMU, optional LiDAR)
- ROS 2 Humble on Jetson
- Completed calibration procedures

## Estimated Time

8-10 hours (can be completed over multiple sessions)

## Project Overview

This mini project requires students to build an end-to-end system that:
1. Generates synthetic training data in Isaac Sim
2. Trains perception models using synthetic data
3. Deploys perception and navigation on Jetson hardware
4. Executes autonomous navigation with object detection
5. Validates performance against simulation results

## Phase 1: System Design and Planning

### Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Isaac Sim   │───▶│  Training Data   │───▶│  Jetson Target  │
│ (Simulation)  │    │  Generation      │    │  (Deployment)   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        │                       │                       │
        ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ Perception      │    │ Model Training   │    │ Perception +    │
│ Model Training  │───▶│ & Optimization   │───▶│ Navigation      │
│ (Synthetic)     │    │ (TensorRT)       │    │ Integration     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### System Requirements

#### Functional Requirements
- Real-time object detection (30+ FPS) on Jetson
- Autonomous navigation to specified waypoints
- Obstacle avoidance during navigation
- Detection and tracking of specified objects
- Safe operation with emergency stop capabilities

#### Performance Requirements
- Perception pipeline: `<33ms` per frame (30+ FPS)
- Navigation planning: `<100ms` per plan
- Control loop: 20+ Hz
- Localization accuracy: `<0.1m` in known environments

#### Safety Requirements
- Emergency stop capability
- Obstacle detection and avoidance
- Maximum speed limits
- Operational boundaries

## Phase 2: Simulation Environment Setup

### Create Complex Simulation Scene

#### Environment Design
```bash
# In Isaac Sim, create a complex environment with:
# - Multiple rooms/areas
# - Various obstacles (static and dynamic)
# - Different lighting conditions
# - Objects to detect and navigate around
# - Navigation waypoints

# Example USD scene setup
# Create a scene that includes:
# 1. Indoor office environment
# 2. Various furniture and obstacles
# 3. Moving objects for dynamic obstacle testing
# 4. Different surface materials for domain randomization
```

#### Robot Configuration
1. Import or create a humanoid robot model
2. Configure sensors (camera, IMU, optional LiDAR)
3. Set up physics properties for realistic simulation
4. Configure ROS 2 bridge for communication

### Domain Randomization Setup

#### Environmental Randomization
1. Randomize lighting conditions
2. Vary surface materials and textures
3. Randomize object positions and orientations
4. Add weather effects if applicable

#### Sensor Noise Modeling
1. Add realistic camera noise
2. Simulate IMU drift and noise
3. Model LiDAR noise patterns
4. Include sensor delays and frame drops

## Phase 3: Data Generation and Model Training

### Synthetic Dataset Creation

#### Object Detection Dataset
```python
# ~/isaac_ws/src/mini_project/scripts/generate_detection_dataset.py

import omni
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np
import cv2
import os
import json
from PIL import Image

class DetectionDatasetGenerator:
    def __init__(self, output_dir="/path/to/dataset"):
        self.output_dir = output_dir
        self.world = World(stage_units_in_meters=1.0)
        self.synthetic_data = SyntheticDataHelper()

        # Create output directories
        os.makedirs(f"{output_dir}/images", exist_ok=True)
        os.makedirs(f"{output_dir}/labels", exist_ok=True)
        os.makedirs(f"{output_dir}/annotations", exist_ok=True)

        self.annotation_data = {
            "info": {
                "description": "Synthetic Detection Dataset for Mini Project",
                "version": "1.0",
                "year": 2024
            },
            "licenses": [],
            "images": [],
            "annotations": [],
            "categories": [
                {"id": 1, "name": "chair", "supercategory": "furniture"},
                {"id": 2, "name": "table", "supercategory": "furniture"},
                {"id": 3, "name": "person", "supercategory": "human"},
                {"id": 4, "name": "box", "supercategory": "obstacle"}
            ]
        }
        self.image_id = 0
        self.annotation_id = 0

    def generate_dataset(self, num_samples=5000):
        """Generate synthetic dataset with domain randomization"""
        for i in range(num_samples):
            # Apply domain randomization
            self.apply_domain_randomization()

            # Render frame
            self.world.step(render=True)

            # Capture data
            rgb_data = self.get_rgb_data()
            depth_data = self.get_depth_data()
            semantic_data = self.get_semantic_data()

            # Generate annotations
            annotations = self.generate_annotations(semantic_data)

            # Save data
            image_filename = f"image_{i:06d}.jpg"
            label_filename = f"labels_{i:06d}.txt"

            # Save RGB image
            cv2.imwrite(f"{self.output_dir}/images/{image_filename}",
                       cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR))

            # Save annotations in YOLO format
            self.save_yolo_annotations(annotations,
                                     f"{self.output_dir}/labels/{label_filename}")

            # Update COCO format annotations
            self.update_coco_annotations(rgb_data.shape, image_filename, annotations)

            print(f"Generated sample {i+1}/{num_samples}")

        # Save COCO format annotations
        with open(f"{self.output_dir}/annotations/instances_train2017.json", 'w') as f:
            json.dump(self.annotation_data, f)

    def apply_domain_randomization(self):
        """Apply domain randomization to scene"""
        # Randomize lighting
        # Randomize object positions
        # Randomize materials
        # Randomize camera parameters
        pass

    def get_rgb_data(self):
        """Get RGB image data"""
        # Implementation to capture RGB image
        pass

    def get_depth_data(self):
        """Get depth data"""
        # Implementation to capture depth
        pass

    def get_semantic_data(self):
        """Get semantic segmentation data"""
        # Implementation to capture semantic labels
        pass

    def generate_annotations(self, semantic_data):
        """Generate bounding box annotations from semantic data"""
        # Convert semantic segmentation to bounding boxes
        pass

    def save_yolo_annotations(self, annotations, filepath):
        """Save annotations in YOLO format"""
        with open(filepath, 'w') as f:
            for ann in annotations:
                # YOLO format: class_id center_x center_y width height
                f.write(f"{ann['class_id']} {ann['center_x']} {ann['center_y']} "
                       f"{ann['width']} {ann['height']}\n")

    def update_coco_annotations(self, img_shape, filename, annotations):
        """Update COCO format annotations"""
        # Add image info
        self.annotation_data["images"].append({
            "id": self.image_id,
            "file_name": filename,
            "height": img_shape[0],
            "width": img_shape[1]
        })

        # Add annotations
        for ann in annotations:
            self.annotation_data["annotations"].append({
                "id": self.annotation_id,
                "image_id": self.image_id,
                "category_id": ann['class_id'],
                "bbox": ann['bbox'],  # [x, y, width, height]
                "area": ann['area'],
                "iscrowd": 0
            })
            self.annotation_id += 1

        self.image_id += 1

def main():
    generator = DetectionDatasetGenerator("/path/to/output/dataset")
    generator.generate_dataset(num_samples=5000)

if __name__ == "__main__":
    main()
```

### Model Training Pipeline

#### Training Script
```python
# ~/isaac_ws/src/mini_project/scripts/train_detection_model.py

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
import torchvision.transforms as transforms
from torchvision.models.detection import fasterrcnn_resnet50_fpn
import albumentations as A
from albumentations.pytorch import ToTensorV2
import numpy as np
import cv2
import os

class SyntheticDetectionDataset(Dataset):
    def __init__(self, image_dir, label_dir, transform=None):
        self.image_dir = image_dir
        self.label_dir = label_dir
        self.transform = transform
        self.images = [f for f in os.listdir(image_dir) if f.endswith('.jpg')]

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img_name = self.images[idx]
        img_path = os.path.join(self.image_dir, img_name)
        label_path = os.path.join(self.label_dir, img_name.replace('.jpg', '.txt'))

        # Load image
        image = cv2.imread(img_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Load annotations (simplified - implement based on your format)
        boxes = []  # [x_min, y_min, x_max, y_max]
        labels = []  # class IDs

        if os.path.exists(label_path):
            with open(label_path, 'r') as f:
                for line in f:
                    # Parse YOLO format: class_id center_x center_y width height
                    parts = line.strip().split()
                    if len(parts) == 5:
                        class_id, center_x, center_y, width, height = map(float, parts)

                        # Convert to corner format
                        x_min = center_x - width/2
                        y_min = center_y - height/2
                        x_max = center_x + width/2
                        y_max = center_y + height/2

                        boxes.append([x_min, y_min, x_max, y_max])
                        labels.append(int(class_id) + 1)  # COCO format starts from 1

        boxes = torch.as_tensor(boxes, dtype=torch.float32)
        labels = torch.as_tensor(labels, dtype=torch.int64)

        target = {
            "boxes": boxes,
            "labels": labels,
            "image_id": torch.tensor([idx])
        }

        if self.transform:
            image = self.transform(image=image)["image"]

        return image, target

def get_transform(train=True):
    if train:
        return A.Compose([
            A.Resize(416, 416),
            A.HorizontalFlip(p=0.5),
            A.ColorJitter(brightness=0.1, contrast=0.1, saturation=0.1, hue=0.1),
            A.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ToTensorV2(),
        ])
    else:
        return A.Compose([
            A.Resize(416, 416),
            A.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ToTensorV2(),
        ])

def train_model():
    # Initialize model
    model = fasterrcnn_resnet50_fpn(pretrained=True)

    # Replace the classifier with the number of classes in your dataset
    num_classes = 5  # background + 4 object classes
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = torch.nn.Linear(in_features, num_classes)

    # Move to GPU if available
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    model.to(device)

    # Define optimizer
    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(params, lr=0.005, momentum=0.9, weight_decay=0.0005)

    # Learning rate scheduler
    lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=3, gamma=0.1)

    # Create dataset and data loader
    dataset = SyntheticDetectionDataset(
        image_dir="/path/to/dataset/images",
        label_dir="/path/to/dataset/labels",
        transform=get_transform(train=True)
    )

    data_loader = DataLoader(dataset, batch_size=2, shuffle=True,
                            collate_fn=lambda x: tuple(zip(*x)))

    # Training loop
    num_epochs = 10
    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0

        for batch_idx, (images, targets) in enumerate(data_loader):
            images = [img.to(device) for img in images]
            targets = [{k: v.to(device) for k, v in t.items()} for t in targets]

            loss_dict = model(images, targets)
            losses = sum(loss for loss in loss_dict.values())

            optimizer.zero_grad()
            losses.backward()
            optimizer.step()

            epoch_loss += losses.item()

            if batch_idx % 10 == 0:
                print(f"Epoch {epoch}, Batch {batch_idx}, Loss: {losses.item():.4f}")

        print(f"Epoch {epoch} completed, Average Loss: {epoch_loss/len(data_loader):.4f}")

        # Update learning rate
        lr_scheduler.step()

    # Save model
    torch.save(model.state_dict(), "detection_model.pth")
    print("Model saved as detection_model.pth")

if __name__ == "__main__":
    train_model()
```

## Phase 4: TensorRT Optimization for Jetson

### Model Optimization Script
```python
# ~/isaac_ws/src/mini_project/scripts/optimize_for_jetson.py

import torch
import torchvision
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np
import os

def export_to_onnx(model_path, onnx_path, input_shape=(1, 3, 416, 416)):
    """Export PyTorch model to ONNX format"""
    # Load model
    model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=False)
    num_classes = 5  # background + 4 object classes
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = torch.nn.Linear(in_features, num_classes)

    model.load_state_dict(torch.load(model_path, map_location='cpu'))
    model.eval()

    # Create dummy input
    dummy_input = torch.randn(input_shape)

    # Export to ONNX
    torch.onnx.export(
        model,
        dummy_input,
        onnx_path,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch_size'},
            'output': {0: 'batch_size'}
        }
    )
    print(f"Model exported to {onnx_path}")

def build_tensorrt_engine(onnx_path, engine_path, precision='fp16'):
    """Build TensorRT engine from ONNX model"""
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

    with trt.Builder(TRT_LOGGER) as builder, \
         builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)) as network, \
         trt.OnnxParser(network, TRT_LOGGER) as parser:

        # Parse ONNX
        with open(onnx_path, 'rb') as model:
            if not parser.parse(model.read()):
                for error in range(parser.num_errors):
                    print(parser.get_error(error))
                return False

        # Create optimization profile
        config = builder.create_optimization_profile()
        profile = builder.create_optimization_profile()

        # Set input shape (adjust based on your model)
        input_name = network.get_input(0).name
        profile.set_shape(input_name, (1, 3, 416, 416), (1, 3, 416, 416), (4, 3, 416, 416))
        config.add_optimization_profile(profile)

        # Set precision
        if precision == 'fp16':
            config.set_flag(trt.BuilderFlag.FP16)

        # Build engine
        serialized_engine = builder.build_serialized_network(network, config)

        # Save engine
        with open(engine_path, 'wb') as f:
            f.write(serialized_engine)

        print(f"TensorRT engine saved to {engine_path}")
        return True

def main():
    # Export model to ONNX
    export_to_onnx("detection_model.pth", "detection_model.onnx")

    # Build TensorRT engine
    build_tensorrt_engine("detection_model.onnx", "detection_model.plan", precision='fp16')

if __name__ == "__main__":
    main()
```

## Phase 5: Integration on Jetson Hardware

### Complete System Launch File
```python
# ~/isaac_ros_ws/src/mini_project/launch/complete_system.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')
    model_path = LaunchConfiguration('model_path', default='/path/to/model.plan')

    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    mini_project_dir = get_package_share_directory('mini_project')

    # Perception container
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image rectification
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
            # TensorRT inference
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::tensor_rt::TensorRtNode',
                name='tensor_rt_node',
                parameters=[{
                    'engine_file_path': model_path,
                    'input_tensor_names': ['input'],
                    'output_tensor_names': ['output'],
                    'max_batch_size': 1,
                    'input_tensor_formats': ['nitros_tensor_list_nchw'],
                    'output_tensor_formats': ['nitros_tensor_list_nchw'],
                    'verbose': False,
                }],
                remappings=[
                    ('tensor_sub', '/camera/image_rect'),
                    ('tensor_pub', '/detections'),
                ]
            ),
        ],
        output='screen'
    )

    # Navigation container
    nav_container = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        nav2_bringup_dir,
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': PathJoinSubstitution([
                        mini_project_dir,
                        'config',
                        'navigation_params.yaml'
                    ])
                }.items()
            )
        ]
    )

    # Safety monitor
    safety_monitor_node = Node(
        package='mini_project',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_safety'),
            ('/cmd_vel_raw', '/cmd_vel'),
        ]
    )

    # Data collection (optional)
    data_collector_node = Node(
        package='mini_project',
        executable='data_collector',
        name='data_collector',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        perception_container,
        nav_container,
        safety_monitor_node,
        data_collector_node,
    ])
```

### Main Control Node
```python
# ~/isaac_ros_ws/src/mini_project/scripts/control_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import numpy as np
import math

class IntegratedController(Node):
    def __init__(self):
        super().__init__('integrated_controller')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_rect', self.image_callback, 10)
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/system_status', 10)

        # Robot state
        self.current_position = None
        self.current_heading = 0.0
        self.detections = []
        self.obstacle_distances = []
        self.current_goal = None

        # Control parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.safe_distance = 0.5  # meters
        self.detection_threshold = 0.7  # confidence threshold

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('Integrated Controller initialized')

    def image_callback(self, msg):
        """Process incoming camera images"""
        # Image processing would happen here
        pass

    def detection_callback(self, msg):
        """Process object detections"""
        self.detections = []
        for detection in msg.detections:
            if detection.results[0].score > self.detection_threshold:
                self.detections.append({
                    'id': detection.results[0].id,
                    'confidence': detection.results[0].score,
                    'bbox': detection.bbox
                })

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.obstacle_distances = [
            r for r in msg.ranges
            if not (math.isnan(r) or math.isinf(r))
        ]

    def odom_callback(self, msg):
        """Update robot position and orientation"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        # Extract heading from quaternion
        q = msg.pose.pose.orientation
        self.current_heading = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def control_loop(self):
        """Main control loop"""
        cmd_vel = Twist()

        # Check for safety conditions
        if self.is_safe_to_proceed():
            # Object-based navigation
            if self.detections:
                cmd_vel = self.navigate_with_detections()
            # Goal-based navigation
            elif self.current_goal:
                cmd_vel = self.navigate_to_goal()
            # Default behavior
            else:
                cmd_vel = self.default_behavior()
        else:
            # Emergency stop
            cmd_vel = Twist()  # Stop the robot
            self.get_logger().warn('Safety condition detected - stopping robot')

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        # Publish system status
        status_msg = String()
        status_msg.data = f"Detections: {len(self.detections)}, Obstacles: {len(self.obstacle_distances)}, Position: {self.current_position}"
        self.status_pub.publish(status_msg)

    def is_safe_to_proceed(self):
        """Check if it's safe to proceed"""
        # Check for obstacles in path
        if self.obstacle_distances:
            min_distance = min(self.obstacle_distances)
            if min_distance < self.safe_distance:
                return False

        return True

    def navigate_with_detections(self):
        """Navigate based on object detections"""
        cmd_vel = Twist()

        # Example: Move toward detected objects
        for detection in self.detections:
            if detection['id'] == 1:  # Example: chair
                # Move toward chair
                cmd_vel.linear.x = self.linear_speed
                break
            elif detection['id'] == 3:  # Example: person
                # Avoid person
                cmd_vel.linear.x = -self.linear_speed * 0.5
                cmd_vel.angular.z = self.angular_speed
                break

        return cmd_vel

    def navigate_to_goal(self):
        """Navigate to predefined goal"""
        cmd_vel = Twist()

        if self.current_position and self.current_goal:
            # Calculate distance to goal
            dx = self.current_goal[0] - self.current_position[0]
            dy = self.current_goal[1] - self.current_position[1]
            distance = math.sqrt(dx*dx + dy*dy)

            if distance > 0.2:  # 20cm tolerance
                # Calculate required heading
                required_heading = math.atan2(dy, dx)
                heading_error = required_heading - self.current_heading

                # Normalize heading error
                while heading_error > math.pi:
                    heading_error -= 2 * math.pi
                while heading_error < -math.pi:
                    heading_error += 2 * math.pi

                # Set velocities
                cmd_vel.linear.x = min(self.linear_speed, distance * 2.0)
                cmd_vel.angular.z = max(-self.angular_speed, min(self.angular_speed, heading_error * 2.0))

        return cmd_vel

    def default_behavior(self):
        """Default behavior when no specific task"""
        cmd_vel = Twist()
        # Example: Explore environment
        cmd_vel.linear.x = self.linear_speed * 0.5
        return cmd_vel

    def set_goal(self, x, y, theta=0.0):
        """Set navigation goal"""
        self.current_goal = (x, y, theta)
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        # Set orientation
        import quaternion
        q = quaternion.from_euler_angles(0, 0, theta)
        goal_msg.pose.orientation.x = q.x
        goal_msg.pose.orientation.y = q.y
        goal_msg.pose.orientation.z = q.z
        goal_msg.pose.orientation.w = q.w

        self.nav_goal_pub.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = IntegratedController()

    # Example: Set a goal after 5 seconds
    def set_example_goal():
        controller.set_goal(2.0, 2.0, 0.0)

    # Schedule example goal
    timer = controller.create_timer(5.0, set_example_goal)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down integrated controller')
    finally:
        # Stop robot on shutdown
        stop_cmd = Twist()
        controller.cmd_vel_pub.publish(stop_cmd)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 6: Testing and Validation

### Performance Testing Script
```python
# ~/isaac_ros_ws/src/mini_project/scripts/performance_test.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import time
import numpy as np
from collections import deque
import psutil
import subprocess

class PerformanceTester(Node):
    def __init__(self):
        super().__init__('performance_tester')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_rect', self.image_callback, 10)
        self.status_sub = self.create_subscription(
            Bool, '/system_status', self.status_callback, 10)

        # Publishers
        self.metrics_pub = self.create_publisher(Float32, '/performance_metrics', 10)

        # Performance tracking
        self.frame_times = deque(maxlen=100)
        self.cpu_usage = deque(maxlen=100)
        self.gpu_usage = deque(maxlen=100)
        self.memory_usage = deque(maxlen=100)

        # Test parameters
        self.test_duration = 60  # seconds
        self.start_time = time.time()
        self.test_active = True

        # Timer for performance measurement
        self.measure_timer = self.create_timer(0.1, self.measure_performance)
        self.report_timer = self.create_timer(5.0, self.report_performance)

    def image_callback(self, msg):
        """Measure frame processing time"""
        if self.test_active:
            current_time = time.time()
            if hasattr(self, 'last_frame_time'):
                frame_time = current_time - self.last_frame_time
                self.frame_times.append(frame_time)
            self.last_frame_time = current_time

    def status_callback(self, msg):
        """Monitor system status"""
        pass

    def measure_performance(self):
        """Measure system performance metrics"""
        if not self.test_active:
            return

        # CPU usage
        cpu_percent = psutil.cpu_percent()
        self.cpu_usage.append(cpu_percent)

        # Memory usage
        memory_percent = psutil.virtual_memory().percent
        self.memory_usage.append(memory_percent)

        # GPU usage (NVIDIA Jetson specific)
        try:
            result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu',
                                   '--format=csv,noheader,nounits'],
                                   capture_output=True, text=True)
            if result.returncode == 0:
                gpu_util = float(result.stdout.strip())
                self.gpu_usage.append(gpu_util)
        except:
            # Fallback to 0 if nvidia-smi is not available
            self.gpu_usage.append(0.0)

    def report_performance(self):
        """Report performance metrics"""
        if not self.frame_times:
            return

        # Calculate metrics
        avg_frame_time = np.mean(self.frame_times)
        fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0
        avg_cpu = np.mean(self.cpu_usage) if self.cpu_usage else 0
        avg_gpu = np.mean(self.gpu_usage) if self.gpu_usage else 0
        avg_memory = np.mean(self.memory_usage) if self.memory_usage else 0

        # Log performance
        self.get_logger().info(
            f'Performance - FPS: {fps:.2f}, CPU: {avg_cpu:.1f}%, '
            f'GPU: {avg_gpu:.1f}%, Memory: {avg_memory:.1f}%'
        )

        # Check if test is complete
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.test_duration:
            self.test_complete(fps, avg_cpu, avg_gpu, avg_memory)

    def test_complete(self, fps, cpu, gpu, memory):
        """Handle test completion"""
        self.test_active = False
        self.get_logger().info('Performance test completed!')
        self.get_logger().info(
            f'Final Results - FPS: {fps:.2f}, CPU: {cpu:.1f}%, '
            f'GPU: {gpu:.1f}%, Memory: {memory:.1f}%'
        )

        # Evaluate performance against requirements
        requirements_met = True
        if fps < 30:
            self.get_logger().error(f'FPS requirement not met: {fps:.2f} < 30')
            requirements_met = False
        if cpu > 80:
            self.get_logger().warn(f'High CPU usage: {cpu:.1f}%')
        if gpu > 85:
            self.get_logger().warn(f'High GPU usage: {gpu:.1f}%')

        if requirements_met:
            self.get_logger().info('All performance requirements met!')
        else:
            self.get_logger().warn('Some performance requirements not met!')

def main(args=None):
    rclpy.init(args=args)
    tester = PerformanceTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Performance test interrupted')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 7: Execution and Evaluation

### Execute the Complete System
```bash
# Build the workspace
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mini_project
source install/setup.bash

# Launch the complete system
ros2 launch mini_project complete_system.launch.py

# In another terminal, start performance testing
ros2 run mini_project performance_test

# In another terminal, monitor the system
source install/setup.bash
rviz2
```

### Mission Execution Script
```python
# ~/isaac_ros_ws/src/mini_project/scripts/execute_mission.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')

        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)

        # Mission waypoints
        self.waypoints = [
            (2.0, 0.0, 0.0),   # Waypoint 1
            (2.0, 2.0, 1.57),  # Waypoint 2
            (0.0, 2.0, 3.14),  # Waypoint 3
            (0.0, 0.0, 0.0),   # Return to start
        ]

        self.current_waypoint = 0
        self.mission_active = False

        # Timer for mission execution
        self.mission_timer = self.create_timer(10.0, self.execute_mission_step)

        self.get_logger().info('Mission Executor initialized')

    def start_mission(self):
        """Start the mission execution"""
        self.mission_active = True
        self.current_waypoint = 0
        self.get_logger().info('Mission started')

        status_msg = String()
        status_msg.data = f"Mission started, waypoint {self.current_waypoint + 1}/{len(self.waypoints)}"
        self.status_pub.publish(status_msg)

        self.go_to_waypoint()

    def execute_mission_step(self):
        """Execute next mission step"""
        if not self.mission_active:
            return

        if self.current_waypoint < len(self.waypoints):
            self.go_to_waypoint()
        else:
            self.mission_complete()

    def go_to_waypoint(self):
        """Navigate to current waypoint"""
        if self.current_waypoint >= len(self.waypoints):
            return

        x, y, theta = self.waypoints[self.current_waypoint]

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0

        # Convert Euler to quaternion
        import math
        cy = math.cos(theta * 0.5)
        sy = math.sin(theta * 0.5)
        goal_msg.pose.orientation.z = sy
        goal_msg.pose.orientation.w = cy

        self.nav_goal_pub.publish(goal_msg)

        self.get_logger().info(f'Going to waypoint {self.current_waypoint + 1}: ({x}, {y}, {theta})')

        status_msg = String()
        status_msg.data = f"Going to waypoint {self.current_waypoint + 1}: ({x}, {y}, {theta})"
        self.status_pub.publish(status_msg)

        self.current_waypoint += 1

    def mission_complete(self):
        """Handle mission completion"""
        self.mission_active = False
        self.get_logger().info('Mission completed successfully!')

        status_msg = String()
        status_msg.data = "Mission completed successfully!"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    executor = MissionExecutor()

    # Start mission after 5 seconds
    def start_mission():
        executor.start_mission()

    timer = executor.create_timer(5.0, start_mission)

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        executor.get_logger().info('Mission interrupted by user')
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Project Assessment

### Knowledge Check Questions
1. How does the integration of perception and navigation systems create synergistic effects?
2. What are the key challenges in deploying end-to-end systems on edge hardware?
3. How do you validate that a complete robotic system meets all requirements?

### Practical Assessment
- Successfully integrate perception and navigation systems
- Achieve real-time performance on Jetson hardware
- Execute complete navigation mission with object detection
- Demonstrate safety and reliability features

### Deliverables
1. Complete integrated system running on Jetson
2. Performance validation report
3. Mission execution demonstration
4. Technical documentation of the complete system

## Advanced Extensions

### Multi-Modal Perception
- Integrate multiple sensor types (camera, LiDAR, IMU)
- Implement sensor fusion algorithms
- Evaluate robustness to sensor failures

### Learning-Based Navigation
- Implement reinforcement learning for navigation
- Adapt navigation behavior based on experience
- Learn optimal paths in dynamic environments

## Troubleshooting

### Common Integration Issues
- **Timing problems**: Ensure proper synchronization between perception and navigation
- **Coordinate frame mismatches**: Verify all transforms are correctly configured
- **Performance bottlenecks**: Monitor and optimize computational pipeline
- **Communication failures**: Check ROS 2 communication patterns and QoS settings

## References

1. NVIDIA Isaac ROS Integration Guide: https://github.com/NVIDIA-ISAAC-ROS
2. ROS 2 Navigation: https://navigation.ros.org/
3. TensorRT Optimization: https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html