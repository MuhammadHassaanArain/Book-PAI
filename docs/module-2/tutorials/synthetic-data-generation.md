# Synthetic Data Generation Tutorial

## Overview

This tutorial covers the generation of synthetic data for robotics applications using Gazebo and Unity integration. We'll explore how to create large-scale, diverse, and accurately labeled datasets for training machine learning models in robotics perception and navigation tasks.

## Introduction to Synthetic Data

### Why Synthetic Data?

Synthetic data generation offers several advantages for robotics:

1. **Cost Reduction**: Eliminates expensive real-world data collection
2. **Safety**: Train on dangerous scenarios without risk
3. **Diversity**: Generate unlimited variations of scenarios
4. **Annotations**: Perfect ground truth for all sensor data
5. **Control**: Precise control over environmental conditions

### Types of Synthetic Data

1. **RGB Images**: Color images for computer vision tasks
2. **Depth Maps**: Depth information for 3D understanding
3. **LiDAR Point Clouds**: 3D spatial information
4. **Semantic Segmentation**: Pixel-level class labels
5. **Instance Segmentation**: Individual object identification
6. **Normal Maps**: Surface orientation information
7. **Optical Flow**: Motion vectors between frames
8. **Sensor Fusion Data**: Combined sensor information

## Gazebo-Based Data Generation

### Setting Up Gazebo for Data Collection

First, let's create a Gazebo world optimized for data generation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="synthetic_data_world">
    <!-- Physics configuration -->
    <physics type="bullet">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include ground plane and sun -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Environment with diverse objects for training -->
    <!-- Buildings and structures -->
    <model name="building_1">
      <pose>5 0 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>4 4 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>4 4 3</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="building_2">
      <pose>-5 0 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>4 4 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>4 4 3</size></box></geometry>
          <material><ambient>0.6 0.6 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Obstacles and furniture -->
    <model name="table_1">
      <pose>0 3 0.4 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1.5 0.8 0.8</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.5 0.8 0.8</size></geometry>
          <material><ambient>0.8 0.6 0.4 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="chair_1">
      <pose>0.8 3 0.2 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.5 0.5 0.4</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.5 0.5 0.4</size></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Random objects for diversity -->
    <model name="box_1">
      <pose>2 2 0.1 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.3 0.3 0.3</size></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="cylinder_1">
      <pose>-2 2 0.2 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>0.2</radius><length>0.4</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.2</radius><length>0.4</length></cylinder></geometry>
          <material><ambient>0.2 0.8 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="sphere_1">
      <pose>2 -2 0.2 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><sphere><radius>0.2</radius></sphere></geometry>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>0.2</radius></sphere></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Robot with sensors -->
    <model name="training_robot">
      <pose>0 0 0.2 0 0 0</pose>
      <link name="chassis">
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.416</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.416</iyy>
            <iyz>0</iyz>
            <izz>0.833</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.5 0.5 0.2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.5 0.5 0.2</size></box></geometry>
          <material><ambient>0.8 0.8 0.2 1</ambient></material>
        </visual>
      </link>

      <!-- LiDAR sensor -->
      <sensor name="lidar_360" type="ray">
        <pose>0.2 0 0.3 0 0 0</pose>
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

      <!-- RGB camera -->
      <sensor name="camera_rgb" type="camera">
        <pose>0.2 0 0.4 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>1.0472</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>robot1</namespace>
            <remapping>~/image_raw:=camera/image_raw</remapping>
            <remapping>~/camera_info:=camera/camera_info</remapping>
          </ros>
          <frame_name>robot1/camera_optical_frame</frame_name>
        </plugin>
      </sensor>

      <!-- Depth camera -->
      <sensor name="camera_depth" type="depth">
        <pose>0.2 0 0.4 0 0 0</pose>
        <visualize>false</visualize>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>1.0472</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
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
        </plugin>
      </sensor>
    </model>
  </world>
</sdf>
```

### Data Collection Script

Create a ROS 2 node to automate data collection:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
from datetime import datetime
import yaml

class SyntheticDataCollector(Node):
    def __init__(self):
        super().__init__('synthetic_data_collector')

        # Initialize parameters
        self.declare_parameter('collection_directory', '~/synthetic_data')
        self.declare_parameter('collection_duration', 300)  # 5 minutes
        self.declare_parameter('image_save_frequency', 1.0)  # Every second
        self.declare_parameter('save_rgb', True)
        self.declare_parameter('save_depth', True)
        self.declare_parameter('save_lidar', True)
        self.declare_parameter('save_segmentation', False)

        # Get parameters
        self.collection_directory = self.get_parameter('collection_directory').value
        self.collection_duration = self.get_parameter('collection_duration').value
        self.image_save_frequency = self.get_parameter('image_save_frequency').value
        self.save_rgb = self.get_parameter('save_rgb').value
        self.save_depth = self.get_parameter('save_depth').value
        self.save_lidar = self.get_parameter('save_lidar').value
        self.save_segmentation = self.get_parameter('save_segmentation').value

        # Create directory structure
        self.setup_directories()

        # Initialize data storage
        self.bridge = CvBridge()
        self.data_counter = 0
        self.last_save_time = self.get_clock().now()

        # Subscriptions for sensor data
        if self.save_rgb:
            self.rgb_subscription = self.create_subscription(
                Image,
                '/robot1/camera/image_raw',
                self.rgb_callback,
                10
            )

        if self.save_depth:
            self.depth_subscription = self.create_subscription(
                Image,
                '/robot1/depth/image_raw',
                self.depth_callback,
                10
            )

        if self.save_lidar:
            self.lidar_subscription = self.create_subscription(
                LaserScan,
                '/robot1/scan',
                self.lidar_callback,
                10
            )

        # Timer for saving data at specified frequency
        self.save_timer = self.create_timer(
            self.image_save_frequency,
            self.save_data_if_ready
        )

        # Timer for collection duration
        self.duration_timer = self.create_timer(
            self.collection_duration,
            self.end_collection
        )

        self.get_logger().info(f'Synthetic Data Collector initialized. Saving to: {self.collection_directory}')

    def setup_directories(self):
        """Create directory structure for data collection"""
        base_path = os.path.expanduser(self.collection_directory)
        self.base_path = base_path

        # Create subdirectories
        self.rgb_dir = os.path.join(base_path, 'rgb')
        self.depth_dir = os.path.join(base_path, 'depth')
        self.lidar_dir = os.path.join(base_path, 'lidar')
        self.metadata_dir = os.path.join(base_path, 'metadata')
        self.annotations_dir = os.path.join(base_path, 'annotations')

        for directory in [self.rgb_dir, self.depth_dir, self.lidar_dir, self.metadata_dir, self.annotations_dir]:
            os.makedirs(directory, exist_ok=True)

        # Create dataset configuration
        self.create_dataset_config()

    def create_dataset_config(self):
        """Create dataset configuration file"""
        config = {
            'dataset_name': 'Synthetic Robotics Dataset',
            'collection_date': datetime.now().isoformat(),
            'parameters': {
                'collection_duration': self.collection_duration,
                'image_save_frequency': self.image_save_frequency,
                'save_rgb': self.save_rgb,
                'save_depth': self.save_depth,
                'save_lidar': self.save_lidar,
                'save_segmentation': self.save_segmentation
            },
            'sensor_configurations': {
                'camera': {
                    'resolution': [640, 480],
                    'fov': 60,  # degrees
                    'format': 'R8G8B8'
                },
                'lidar': {
                    'samples': 360,
                    'range_min': 0.1,
                    'range_max': 30.0,
                    'fov_horizontal': 360  # degrees
                }
            }
        }

        config_path = os.path.join(self.base_path, 'dataset_config.yaml')
        with open(config_path, 'w') as f:
            yaml.dump(config, f)

    def rgb_callback(self, msg):
        """Handle RGB image messages"""
        if self.should_save_data():
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                # Save image
                filename = f'rgb_{self.data_counter:06d}.png'
                filepath = os.path.join(self.rgb_dir, filename)
                cv2.imwrite(filepath, cv_image)

                # Save metadata
                metadata = {
                    'filename': filename,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                    'sensor': 'rgb_camera',
                    'width': msg.width,
                    'height': msg.height,
                    'encoding': msg.encoding
                }

                self.save_metadata(metadata)

                self.get_logger().info(f'Saved RGB image: {filename}')

            except Exception as e:
                self.get_logger().error(f'Error processing RGB image: {e}')

    def depth_callback(self, msg):
        """Handle depth image messages"""
        if self.should_save_data():
            try:
                cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

                # Save depth image (as EXR for high precision)
                filename = f'depth_{self.data_counter:06d}.exr'
                filepath = os.path.join(self.depth_dir, filename)

                # Use OpenCV to save depth as EXR
                cv2.imwrite(filepath, cv_depth)

                # Save metadata
                metadata = {
                    'filename': filename,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                    'sensor': 'depth_camera',
                    'width': msg.width,
                    'height': msg.height,
                    'encoding': msg.encoding,
                    'range_min': 0.1,
                    'range_max': 10.0
                }

                self.save_metadata(metadata)

                self.get_logger().info(f'Saved depth image: {filename}')

            except Exception as e:
                self.get_logger().error(f'Error processing depth image: {e}')

    def lidar_callback(self, msg):
        """Handle LiDAR scan messages"""
        if self.should_save_data():
            try:
                # Convert LiDAR data to dictionary for JSON serialization
                lidar_data = {
                    'ranges': [float(r) if not np.isnan(r) and r > 0 else float('inf') for r in msg.ranges],
                    'intensities': [float(i) for i in msg.intensities],
                    'angle_min': msg.angle_min,
                    'angle_max': msg.angle_max,
                    'angle_increment': msg.angle_increment,
                    'time_increment': msg.time_increment,
                    'scan_time': msg.scan_time,
                    'range_min': msg.range_min,
                    'range_max': msg.range_max
                }

                # Save LiDAR data as JSON
                filename = f'lidar_{self.data_counter:06d}.json'
                filepath = os.path.join(self.lidar_dir, filename)

                with open(filepath, 'w') as f:
                    json.dump(lidar_data, f, indent=2)

                # Save metadata
                metadata = {
                    'filename': filename,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                    'sensor': 'lidar_360',
                    'num_beams': len(msg.ranges),
                    'fov_degrees': np.degrees(msg.angle_max - msg.angle_min),
                    'range_min': msg.range_min,
                    'range_max': msg.range_max
                }

                self.save_metadata(metadata)

                self.get_logger().info(f'Saved LiDAR data: {filename}')

            except Exception as e:
                self.get_logger().error(f'Error processing LiDAR data: {e}')

    def should_save_data(self):
        """Check if enough time has passed to save data"""
        current_time = self.get_clock().now()
        time_diff = (current_time.nanoseconds - self.last_save_time.nanoseconds) / 1e9

        return time_diff >= self.image_save_frequency

    def save_data_if_ready(self):
        """Save data if conditions are met"""
        if self.should_save_data():
            self.data_counter += 1
            self.last_save_time = self.get_clock().now()

    def save_metadata(self, metadata):
        """Save metadata for the current data sample"""
        metadata_filename = f'metadata_{self.data_counter:06d}.json'
        metadata_path = os.path.join(self.metadata_dir, metadata_filename)

        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)

    def end_collection(self):
        """End data collection and finalize dataset"""
        self.get_logger().info(f'Data collection completed. Total samples: {self.data_counter}')

        # Create summary statistics
        self.create_summary_statistics()

        # Shutdown node
        self.destroy_timer(self.save_timer)
        self.destroy_timer(self.duration_timer)

    def create_summary_statistics(self):
        """Create summary statistics for the dataset"""
        summary = {
            'total_samples': self.data_counter,
            'collection_duration_seconds': self.collection_duration,
            'samples_per_second': self.data_counter / self.collection_duration if self.collection_duration > 0 else 0,
            'start_time': self.get_clock().now().nanoseconds / 1e9,
            'end_time': self.get_clock().now().nanoseconds / 1e9,
            'data_types_collected': {
                'rgb': self.save_rgb,
                'depth': self.save_depth,
                'lidar': self.save_lidar,
                'segmentation': self.save_segmentation
            }
        }

        summary_path = os.path.join(self.base_path, 'summary_statistics.json')
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)

def main(args=None):
    rclpy.init(args=args)

    collector = SyntheticDataCollector()

    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('Interrupted by user')
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Unity-Based Data Generation

### Unity Data Generation Script

Create a Unity script for generating synthetic data with Unity's rendering capabilities:

```csharp
using UnityEngine;
using System.Collections;
using System.IO;
using System.Collections.Generic;

public class UnityDataGenerator : MonoBehaviour
{
    [Header("Data Generation Configuration")]
    public string outputDirectory = "SyntheticData";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public int antiAliasing = 1;
    public float captureInterval = 1.0f;  // Seconds between captures
    public bool captureRGB = true;
    public bool captureDepth = true;
    public bool captureSegmentation = true;
    public bool captureNormals = true;
    public bool captureOpticalFlow = false;

    [Header("Environment Configuration")]
    public GameObject[] objectsToRandomize;
    public Material[] materialOptions;
    public Color[] segmentationColors;
    public Light[] lightsToRandomize;

    [Header("Camera Configuration")]
    public Camera rgbCamera;
    public Camera depthCamera;
    public Camera segmentationCamera;
    public Camera normalCamera;

    private RenderTexture rgbTexture;
    private RenderTexture depthTexture;
    private RenderTexture segmentationTexture;
    private RenderTexture normalTexture;
    private int captureCounter = 0;
    private float lastCaptureTime = 0f;

    void Start()
    {
        InitializeDataGeneration();
        StartCoroutine(DataGenerationLoop());
    }

    void InitializeDataGeneration()
    {
        // Create render textures for different data types
        if (captureRGB)
        {
            rgbTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
            rgbTexture.antiAliasing = antiAliasing;
            rgbTexture.Create();
            if (rgbCamera != null) rgbCamera.targetTexture = rgbTexture;
        }

        if (captureDepth)
        {
            depthTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.RFloat);
            depthTexture.Create();
            if (depthCamera != null) depthCamera.targetTexture = depthTexture;
        }

        if (captureSegmentation)
        {
            segmentationTexture = new RenderTexture(imageWidth, imageHeight, 0, RenderTextureFormat.ARGB32);
            segmentationTexture.Create();
            if (segmentationCamera != null) segmentationCamera.targetTexture = segmentationTexture;
        }

        if (captureNormals)
        {
            normalTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
            normalTexture.Create();
            if (normalCamera != null) normalCamera.targetTexture = normalTexture;
        }

        // Create output directories
        Directory.CreateDirectory(Path.Combine(outputDirectory, "rgb"));
        Directory.CreateDirectory(Path.Combine(outputDirectory, "depth"));
        Directory.CreateDirectory(Path.Combine(outputDirectory, "segmentation"));
        Directory.CreateDirectory(Path.Combine(outputDirectory, "normals"));
        Directory.CreateDirectory(Path.Combine(outputDirectory, "metadata"));

        Debug.Log($"Data generation initialized. Output directory: {outputDirectory}");
    }

    IEnumerator DataGenerationLoop()
    {
        while (true)
        {
            float currentTime = Time.time;
            if (currentTime - lastCaptureTime >= captureInterval)
            {
                // Randomize environment
                RandomizeEnvironment();

                // Capture all enabled data types
                if (captureRGB) CaptureRGBImage();
                if (captureDepth) CaptureDepthImage();
                if (captureSegmentation) CaptureSegmentationImage();
                if (captureNormals) CaptureNormalImage();

                // Save metadata
                SaveMetadata();

                captureCounter++;
                lastCaptureTime = currentTime;

                Debug.Log($"Captured sample {captureCounter}");
            }

            yield return null; // Wait for next frame
        }
    }

    void RandomizeEnvironment()
    {
        // Randomize object positions and rotations
        foreach (GameObject obj in objectsToRandomize)
        {
            if (obj != null)
            {
                // Random position within bounds
                Vector3 randomPos = new Vector3(
                    Random.Range(-5f, 5f),
                    Random.Range(0.1f, 2f),
                    Random.Range(-5f, 5f)
                );
                obj.transform.position = randomPos;

                // Random rotation
                obj.transform.rotation = Quaternion.Euler(
                    Random.Range(0f, 360f),
                    Random.Range(0f, 360f),
                    Random.Range(0f, 360f)
                );

                // Random scale (maintaining some constraints)
                float scale = Random.Range(0.5f, 2f);
                obj.transform.localScale = Vector3.one * scale;

                // Random material
                if (materialOptions.Length > 0)
                {
                    Renderer renderer = obj.GetComponent<Renderer>();
                    if (renderer != null)
                    {
                        Material randomMaterial = materialOptions[Random.Range(0, materialOptions.Length)];
                        renderer.material = randomMaterial;
                    }
                }
            }
        }

        // Randomize lighting
        foreach (Light light in lightsToRandomize)
        {
            if (light != null)
            {
                light.intensity = Random.Range(0.5f, 2f);
                light.color = Random.ColorHSV(0f, 1f, 0.5f, 1f, 0.5f, 1f);

                // Random position and rotation
                light.transform.position = new Vector3(
                    Random.Range(-3f, 3f),
                    Random.Range(3f, 8f),
                    Random.Range(-3f, 3f)
                );

                light.transform.rotation = Quaternion.Euler(
                    Random.Range(0f, 45f),
                    Random.Range(0f, 360f),
                    Random.Range(0f, 360f)
                );
            }
        }
    }

    void CaptureRGBImage()
    {
        if (rgbCamera == null) return;

        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = rgbCamera.targetTexture;
        rgbCamera.Render();

        Texture2D image = new Texture2D(rgbCamera.targetTexture.width, rgbCamera.targetTexture.height, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, rgbCamera.targetTexture.width, rgbCamera.targetTexture.height), 0, 0);
        image.Apply();

        RenderTexture.active = currentRT;

        // Save image
        string filename = Path.Combine(outputDirectory, "rgb", $"rgb_{captureCounter:D6}.png");
        byte[] bytes = image.EncodeToPNG();
        File.WriteAllBytes(filename, bytes);

        DestroyImmediate(image);
    }

    void CaptureDepthImage()
    {
        if (depthCamera == null) return;

        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = depthCamera.targetTexture;
        depthCamera.Render();

        Texture2D depthTexture2D = new Texture2D(depthCamera.targetTexture.width, depthCamera.targetTexture.height, TextureFormat.RFloat, false);
        depthTexture2D.ReadPixels(new Rect(0, 0, depthCamera.targetTexture.width, depthCamera.targetTexture.height), 0, 0);
        depthTexture2D.Apply();

        RenderTexture.active = currentRT;

        // Save depth image
        string filename = Path.Combine(outputDirectory, "depth", $"depth_{captureCounter:D6}.exr");
        byte[] bytes = depthTexture2D.EncodeToEXR();
        File.WriteAllBytes(filename, bytes);

        DestroyImmediate(depthTexture2D);
    }

    void CaptureSegmentationImage()
    {
        if (segmentationCamera == null) return;

        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = segmentationCamera.targetTexture;
        segmentationCamera.Render();

        Texture2D segmentationTexture2D = new Texture2D(segmentationCamera.targetTexture.width, segmentationCamera.targetTexture.height, TextureFormat.RGB24, false);
        segmentationTexture2D.ReadPixels(new Rect(0, 0, segmentationCamera.targetTexture.width, segmentationCamera.targetTexture.height), 0, 0);
        segmentationTexture2D.Apply();

        RenderTexture.active = currentRT;

        // Save segmentation image
        string filename = Path.Combine(outputDirectory, "segmentation", $"seg_{captureCounter:D6}.png");
        byte[] bytes = segmentationTexture2D.EncodeToPNG();
        File.WriteAllBytes(filename, bytes);

        DestroyImmediate(segmentationTexture2D);
    }

    void CaptureNormalImage()
    {
        if (normalCamera == null) return;

        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = normalCamera.targetTexture;
        normalCamera.Render();

        Texture2D normalTexture2D = new Texture2D(normalCamera.targetTexture.width, normalCamera.targetTexture.height, TextureFormat.RGB24, false);
        normalTexture2D.ReadPixels(new Rect(0, 0, normalCamera.targetTexture.width, normalCamera.targetTexture.height), 0, 0);
        normalTexture2D.Apply();

        RenderTexture.active = currentRT;

        // Save normal image
        string filename = Path.Combine(outputDirectory, "normals", $"normals_{captureCounter:D6}.png");
        byte[] bytes = normalTexture2D.EncodeToPNG();
        File.WriteAllBytes(filename, bytes);

        DestroyImmediate(normalTexture2D);
    }

    void SaveMetadata()
    {
        // Create metadata for current capture
        var metadata = new
        {
            capture_number = captureCounter,
            timestamp = System.DateTime.UtcNow.ToString("o"),
            camera_position = transform.position,
            camera_rotation = transform.rotation.eulerAngles,
            environment_state = GetEnvironmentState(),
            objects_present = GetObjectInfo()
        };

        string metadataPath = Path.Combine(outputDirectory, "metadata", $"meta_{captureCounter:D6}.json");
        string json = JsonUtility.ToJson(metadata, true);
        File.WriteAllText(metadataPath, json);
    }

    object GetEnvironmentState()
    {
        // Return environment state information
        return new
        {
            time_of_day = Time.time % 24,  // Simplified time representation
            lighting_conditions = GetLightingConditions(),
            weather_simulation = "clear"  // Could be extended for weather effects
        };
    }

    object GetLightingConditions()
    {
        // Get information about current lighting
        List<object> lightsInfo = new List<object>();
        foreach (Light light in lightsToRandomize)
        {
            if (light != null)
            {
                lightsInfo.Add(new
                {
                    name = light.name,
                    position = light.transform.position,
                    rotation = light.transform.rotation.eulerAngles,
                    intensity = light.intensity,
                    color = light.color
                });
            }
        }
        return lightsInfo;
    }

    object GetObjectInfo()
    {
        // Get information about objects in the scene
        List<object> objectsInfo = new List<object>();
        foreach (GameObject obj in objectsToRandomize)
        {
            if (obj != null)
            {
                objectsInfo.Add(new
                {
                    name = obj.name,
                    position = obj.transform.position,
                    rotation = obj.transform.rotation.eulerAngles,
                    scale = obj.transform.localScale,
                    material = obj.GetComponent<Renderer>()?.material?.name ?? "Unknown"
                });
            }
        }
        return objectsInfo;
    }

    void OnDisable()
    {
        // Clean up render textures
        if (rgbTexture != null) RenderTexture.ReleaseTemporary(rgbTexture);
        if (depthTexture != null) RenderTexture.ReleaseTemporary(depthTexture);
        if (segmentationTexture != null) RenderTexture.ReleaseTemporary(segmentationTexture);
        if (normalTexture != null) RenderTexture.ReleaseTemporary(normalTexture);
    }
}
```

## Domain Randomization Techniques

### Advanced Domain Randomization

Implement systematic variation of visual properties:

```python
# domain_randomization.py
import numpy as np
import cv2
import random
from PIL import Image, ImageEnhance, ImageFilter

class DomainRandomizer:
    def __init__(self):
        # Lighting parameters
        self.lighting_params = {
            'intensity_range': (0.3, 2.0),
            'color_temperature_range': (3000, 8000),  # Kelvin
            'direction_variation': (0.1, 0.5)  # Random variation in light direction
        }

        # Material parameters
        self.material_params = {
            'roughness_range': (0.1, 0.9),
            'metallic_range': (0.0, 1.0),
            'specular_range': (0.1, 1.0),
            'albedo_variation': 0.3
        }

        # Camera parameters
        self.camera_params = {
            'exposure_range': (-1.0, 1.0),  # EV adjustment
            'white_balance_variation': 0.1,
            'focus_range': (0.1, 10.0),  # Focus distance range
            'aperture_range': (1.4, 16.0)  # Aperture range
        }

        # Environmental parameters
        self.environment_params = {
            'fog_density_range': (0.0, 0.1),
            'fog_color_range': (0.8, 1.0),  # Range for fog color components
            'atmospheric_scattering': True
        }

    def randomize_lighting(self, scene):
        """Apply random lighting variations"""
        # Randomize light intensities
        for light in scene.lights:
            intensity_factor = random.uniform(
                self.lighting_params['intensity_range'][0],
                self.lighting_params['intensity_range'][1]
            )
            light.intensity *= intensity_factor

            # Randomize light color temperature
            color_temp = random.uniform(
                self.lighting_params['color_temperature_range'][0],
                self.lighting_params['color_temperature_range'][1]
            )
            light.color = self.color_temperature_to_rgb(color_temp)

            # Randomize light direction (slightly)
            direction_variation = self.lighting_params['direction_variation']
            light.direction += np.random.uniform(
                -direction_variation[0], direction_variation[1], 3
            )

    def color_temperature_to_rgb(self, kelvin):
        """Convert color temperature in Kelvin to RGB"""
        temp = kelvin / 100
        r, g, b = 0, 0, 0

        # Red calculation
        if temp <= 66:
            r = 255
        else:
            r = temp - 60
            r = 329.698727446 * (r ** -0.1332047592)
            r = max(0, min(255, r))

        # Green calculation
        if temp <= 66:
            g = temp
            g = 99.4708025861 * np.log(g) - 161.1195681661
        else:
            g = temp - 60
            g = 288.1221695283 * (g ** -0.0755148492)
        g = max(0, min(255, g))

        # Blue calculation
        if temp >= 66:
            b = 255
        elif temp <= 19:
            b = 0
        else:
            b = temp - 10
            b = 138.5177312231 * np.log(b) - 305.0447927307
            b = max(0, min(255, b))

        return np.array([r, g, b]) / 255.0

    def randomize_materials(self, materials):
        """Apply random material variations"""
        for material in materials:
            # Randomize roughness
            roughness_factor = random.uniform(
                self.material_params['roughness_range'][0],
                self.material_params['roughness_range'][1]
            )
            material.roughness = roughness_factor

            # Randomize metallic
            metallic_factor = random.uniform(
                self.material_params['metallic_range'][0],
                self.material_params['metallic_range'][1]
            )
            material.metallic = metallic_factor

            # Randomize albedo with some variation
            albedo_variation = self.material_params['albedo_variation']
            material.albedo += np.random.uniform(
                -albedo_variation, albedo_variation, 3
            )
            material.albedo = np.clip(material.albedo, 0, 1)

    def apply_post_processing(self, image):
        """Apply post-processing effects to simulate real camera"""
        img_pil = Image.fromarray((image * 255).astype(np.uint8))

        # Random exposure adjustment
        exposure_factor = random.uniform(
            self.camera_params['exposure_range'][0],
            self.camera_params['exposure_range'][1]
        )
        enhancer = ImageEnhance.Brightness(img_pil)
        img_pil = enhancer.enhance(2 ** exposure_factor)

        # Random contrast adjustment
        contrast_factor = random.uniform(0.8, 1.2)
        enhancer = ImageEnhance.Contrast(img_pil)
        img_pil = enhancer.enhance(contrast_factor)

        # Random saturation adjustment
        saturation_factor = random.uniform(0.8, 1.2)
        enhancer = ImageEnhance.Color(img_pil)
        img_pil = enhancer.enhance(saturation_factor)

        # Add slight blur to simulate lens imperfection
        blur_factor = random.uniform(0.5, 1.5)
        img_pil = img_pil.filter(ImageFilter.GaussianBlur(radius=blur_factor/10))

        return np.array(img_pil) / 255.0

    def add_sensor_noise(self, image):
        """Add realistic sensor noise"""
        # Shot noise (photon noise) - proportional to signal
        shot_noise = np.random.poisson(image * 255) / 255.0
        shot_noise = shot_noise - np.mean(shot_noise)  # Center around 0

        # Read noise (constant regardless of signal)
        read_noise_std = random.uniform(0.005, 0.02)
        read_noise = np.random.normal(0, read_noise_std, image.shape)

        # Combined noise
        noisy_image = image + shot_noise + read_noise

        return np.clip(noisy_image, 0, 1)

    def generate_random_background(self):
        """Generate random background textures"""
        bg_type = random.choice(['natural', 'urban', 'indoor', 'synthetic'])

        if bg_type == 'natural':
            # Generate natural background
            width, height = 640, 480
            bg = np.zeros((height, width, 3))

            # Add sky gradient
            for y in range(height):
                ratio = y / height
                bg[y, :, :] = [0.5 * (1 - ratio), 0.7 * (1 - ratio), 1.0]

            # Add some clouds or trees
            if random.random() > 0.5:
                # Add simple cloud
                center_x = random.randint(50, width-50)
                center_y = random.randint(50, height//2)
                radius = random.randint(20, 50)
                cv2.circle(bg, (center_x, center_y), radius, (0.9, 0.9, 1.0), -1)

            return bg

        elif bg_type == 'urban':
            # Generate urban background
            width, height = 640, 480
            bg = np.full((height, width, 3), [0.3, 0.3, 0.4])  # Gray urban sky

            # Add buildings
            for _ in range(random.randint(3, 8)):
                x = random.randint(0, width)
                width_building = random.randint(30, 100)
                height_building = random.randint(100, height-50)
                y = height - height_building

                color = [random.uniform(0.2, 0.6), random.uniform(0.2, 0.6), random.uniform(0.2, 0.6)]
                cv2.rectangle(bg, (x, y), (x + width_building, height), color, -1)

            return bg

        else:
            # Default neutral background
            return np.full((480, 640, 3), [0.5, 0.5, 0.5])

    def randomize_environment(self, scene):
        """Apply comprehensive environment randomization"""
        # Randomize lighting
        self.randomize_lighting(scene)

        # Randomize materials
        materials = scene.get_all_materials()
        self.randomize_materials(materials)

        # Randomize background
        if random.random() > 0.7:  # 30% chance to change background
            scene.background = self.generate_random_background()

        # Randomize fog/atmospheric effects
        if self.environment_params['atmospheric_scattering']:
            fog_density = random.uniform(
                self.environment_params['fog_density_range'][0],
                self.environment_params['fog_density_range'][1]
            )
            scene.set_fog_density(fog_density)

            fog_color_factor = random.uniform(
                self.environment_params['fog_color_range'][0],
                self.environment_params['fog_color_range'][1]
            )
            scene.set_fog_color([fog_color_factor] * 3)

    def process_synthetic_image(self, image):
        """Complete processing pipeline for synthetic image"""
        # Apply post-processing
        processed_image = self.apply_post_processing(image)

        # Add sensor noise
        noisy_image = self.add_sensor_noise(processed_image)

        # Apply any final adjustments
        final_image = np.clip(noisy_image, 0, 1)

        return final_image
```

## Annotation Generation

### Automatic Annotation System

Generate accurate annotations for synthetic data:

```python
# annotation_generator.py
import numpy as np
import json
import cv2
from typing import List, Dict, Tuple

class AnnotationGenerator:
    def __init__(self):
        self.instance_id_counter = 0
        self.category_mapping = {}  # Maps object names to category IDs

    def generate_segmentation_mask(self, scene_objects: List[Dict]) -> np.ndarray:
        """Generate semantic/instance segmentation mask"""
        # Assuming we have a rendered segmentation image where each object has a unique color
        # In practice, this would come from a special Unity pass or Gazebo plugin
        height, width = 480, 640
        segmentation_mask = np.zeros((height, width), dtype=np.uint8)

        for i, obj in enumerate(scene_objects):
            # Create mask for this object
            obj_mask = self.create_object_mask(obj, width, height)
            segmentation_mask[obj_mask > 0] = i + 1  # Instance ID

        return segmentation_mask

    def create_object_mask(self, obj_info: Dict, width: int, height: int) -> np.ndarray:
        """Create binary mask for a single object"""
        mask = np.zeros((height, width), dtype=np.uint8)

        # Get object's bounding box in image coordinates
        bbox = self.get_object_bbox(obj_info, width, height)

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)

            # Create rough mask for the object (in practice, this would be more accurate)
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)

            # Add some object-specific shape refinement
            if 'shape' in obj_info:
                if obj_info['shape'] == 'circular':
                    center = ((x1 + x2) // 2, (y1 + y2) // 2)
                    radius = min(x2 - x1, y2 - y1) // 2
                    cv2.circle(mask, center, radius, 255, -1)
                elif obj_info['shape'] == 'cylindrical':
                    center = ((x1 + x2) // 2, (y1 + y2) // 2)
                    axes = ((x2 - x1) // 2, (y2 - y1) // 2)
                    cv2.ellipse(mask, center, axes, 0, 0, 360, 255, -1)

        return mask

    def get_object_bbox(self, obj_info: Dict, width: int, height: int) -> Tuple[int, int, int, int]:
        """Get bounding box for object in image coordinates"""
        # This would typically involve projecting 3D object bounds to 2D image
        # For simplicity, we'll use a basic projection
        if 'position_3d' in obj_info and 'size_3d' in obj_info:
            pos_3d = obj_info['position_3d']
            size_3d = obj_info['size_3d']

            # Simple perspective projection (would be more complex in reality)
            # Using hardcoded camera parameters
            focal_length = 525  # pixels
            principal_point = (width/2, height/2)

            # Calculate 3D bounding box corners
            corners_3d = [
                [pos_3d[0] - size_3d[0]/2, pos_3d[1] - size_3d[1]/2, pos_3d[2] - size_3d[2]/2],
                [pos_3d[0] + size_3d[0]/2, pos_3d[1] + size_3d[1]/2, pos_3d[2] + size_3d[2]/2]
            ]

            # Project to 2D
            corners_2d = []
            for corner in corners_3d:
                if corner[2] > 0:  # In front of camera
                    x_2d = principal_point[0] + (corner[0] * focal_length) / corner[2]
                    y_2d = principal_point[1] + (corner[1] * focal_length) / corner[2]
                    corners_2d.append([int(x_2d), int(y_2d)])

            if len(corners_2d) >= 2:
                x_coords = [c[0] for c in corners_2d]
                y_coords = [c[1] for c in corners_2d]
                return min(x_coords), min(y_coords), max(x_coords), max(y_coords)

        return None

    def generate_detection_annotations(self, scene_objects: List[Dict], image_width: int, image_height: int) -> List[Dict]:
        """Generate object detection annotations in COCO format"""
        annotations = []

        for obj in scene_objects:
            bbox = self.get_object_bbox(obj, image_width, image_height)
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                width = x2 - x1
                height = y2 - y1

                # Get category ID
                category_id = self.get_category_id(obj.get('name', 'unknown'))

                annotation = {
                    "id": self.instance_id_counter,
                    "image_id": 0,  # Would be set externally
                    "category_id": category_id,
                    "bbox": [float(x1), float(y1), float(width), float(height)],
                    "area": float(width * height),
                    "iscrowd": 0,
                    "segmentation": self.get_polygon_segmentation(obj, bbox)
                }

                annotations.append(annotation)
                self.instance_id_counter += 1

        return annotations

    def get_category_id(self, object_name: str) -> int:
        """Get category ID for an object name"""
        if object_name not in self.category_mapping:
            # Assign new category ID
            new_id = len(self.category_mapping) + 1
            self.category_mapping[object_name] = new_id

            # Also add to categories list (would be maintained externally)
            # self.categories.append({"id": new_id, "name": object_name})

        return self.category_mapping[object_name]

    def get_polygon_segmentation(self, obj_info: Dict, bbox: Tuple[int, int, int, int]) -> List[List[float]]:
        """Generate polygon segmentation for an object"""
        if bbox is None:
            return []

        x1, y1, x2, y2 = bbox
        center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
        width, height = x2 - x1, y2 - y1

        # Generate polygon based on object shape
        if obj_info.get('shape') == 'circular':
            # Generate circular polygon
            points = []
            for angle in range(0, 360, 10):  # Every 10 degrees
                rad = np.radians(angle)
                px = center_x + (width/2) * np.cos(rad)
                py = center_y + (height/2) * np.sin(rad)
                points.extend([px, py])
        else:
            # Default to rectangular polygon
            points = [x1, y1, x2, y1, x2, y2, x1, y2]

        return [points]

    def generate_coco_format(self, images: List[Dict], annotations: List[List[Dict]], categories: List[Dict]) -> Dict:
        """Generate complete COCO format dataset"""
        coco_dataset = {
            "info": {
                "year": 2023,
                "version": "1.0",
                "description": "Synthetic Robotics Dataset",
                "contributor": "Robotics Research Team",
                "url": "",
                "date_created": "2023-12-09"
            },
            "licenses": [{
                "id": 1,
                "name": "Creative Commons Attribution 4.0 License",
                "url": "http://creativecommons.org/licenses/by/4.0/"
            }],
            "images": images,
            "annotations": [ann for img_anns in annotations for ann in img_anns],  # Flatten annotations
            "categories": categories
        }

        return coco_dataset

    def save_annotations(self, dataset_dict: Dict, output_path: str):
        """Save annotations to JSON file"""
        with open(output_path, 'w') as f:
            json.dump(dataset_dict, f, indent=2)
        print(f"Annotations saved to {output_path}")

# Example usage in data collection pipeline
def integrate_annotations_into_pipeline():
    """Example of how to integrate annotation generation into data pipeline"""
    annotator = AnnotationGenerator()

    # During data collection, store object information
    scene_objects = [
        {
            "name": "box_red",
            "position_3d": [1.0, 0.5, 0.2],
            "size_3d": [0.3, 0.3, 0.3],
            "shape": "rectangular"
        },
        {
            "name": "cylinder_blue",
            "position_3d": [-0.5, 1.2, 0.3],
            "size_3d": [0.2, 0.2, 0.4],
            "shape": "cylindrical"
        }
    ]

    # Generate annotations for this frame
    image_width, image_height = 640, 480
    detections = annotator.generate_detection_annotations(scene_objects, image_width, image_height)

    # Create image entry
    image_entry = {
        "id": 1,
        "width": image_width,
        "height": image_height,
        "file_name": "rgb_000001.png",
        "license": 1,
        "date_captured": "2023-12-09 10:00:00"
    }

    # Create categories from the mapping
    categories = [{"id": cat_id, "name": name} for name, cat_id in annotator.category_mapping.items()]

    # Generate full COCO dataset
    coco_dataset = annotator.generate_coco_format([image_entry], [detections], categories)

    # Save annotations
    annotator.save_annotations(coco_dataset, "annotations/instances_train2023.json")
```

## Quality Assurance and Validation

### Data Quality Metrics

Validate synthetic data quality:

```python
# data_quality_assessment.py
import numpy as np
import cv2
from scipy import ndimage
from skimage import feature, measure
import matplotlib.pyplot as plt

class DataQualityAssessment:
    def __init__(self):
        self.metrics_history = []

    def assess_image_quality(self, image: np.ndarray) -> Dict[str, float]:
        """Assess quality of synthetic image"""
        metrics = {}

        # Convert to grayscale for some metrics
        if len(image.shape) == 3:
            gray = cv2.cvtColor((image * 255).astype(np.uint8), cv2.COLOR_RGB2GRAY)
        else:
            gray = (image * 255).astype(np.uint8)

        # Sharpness (Laplacian variance)
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        metrics['sharpness'] = float(laplacian_var)

        # Contrast (standard deviation)
        contrast = np.std(gray.astype(np.float64))
        metrics['contrast'] = float(contrast)

        # Brightness (mean intensity)
        brightness = np.mean(gray.astype(np.float64))
        metrics['brightness'] = float(brightness)

        # Entropy (information content)
        hist, _ = np.histogram(gray.flatten(), bins=256, range=[0, 256])
        hist = hist / hist.sum()  # Normalize
        hist = hist[hist > 0]  # Remove zeros to avoid log(0)
        entropy = -np.sum(hist * np.log2(hist))
        metrics['entropy'] = float(entropy)

        # Edge density
        edges = feature.canny(gray, sigma=2)
        edge_density = np.sum(edges) / edges.size
        metrics['edge_density'] = float(edge_density)

        return metrics

    def assess_depth_quality(self, depth_image: np.ndarray, gt_depth: np.ndarray = None) -> Dict[str, float]:
        """Assess quality of depth image"""
        metrics = {}

        # Basic statistics
        metrics['mean_depth'] = float(np.mean(depth_image[depth_image > 0]))
        metrics['std_depth'] = float(np.std(depth_image[depth_image > 0]))
        metrics['min_depth'] = float(np.min(depth_image[depth_image > 0]))
        metrics['max_depth'] = float(np.max(depth_image[depth_image > 0]))

        # Valid pixel percentage
        total_pixels = depth_image.size
        valid_pixels = np.sum(depth_image > 0)
        metrics['valid_pixel_percentage'] = float(valid_pixels / total_pixels * 100)

        # If ground truth is provided, calculate accuracy metrics
        if gt_depth is not None:
            # Calculate errors
            valid_mask = (depth_image > 0) & (gt_depth > 0)
            if np.sum(valid_mask) > 0:
                diff = np.abs(depth_image[valid_mask] - gt_depth[valid_mask])

                metrics['mae'] = float(np.mean(diff))  # Mean Absolute Error
                metrics['rmse'] = float(np.sqrt(np.mean(diff**2)))  # Root Mean Square Error
                metrics['median_error'] = float(np.median(diff))

                # Percentage of pixels within thresholds
                for thresh in [0.01, 0.05, 0.1, 0.2]:  # 1cm, 5cm, 10cm, 20cm
                    within_thresh = np.sum(diff <= thresh) / len(diff) * 100
                    metrics[f'within_{thresh*100:.0f}cm'] = float(within_thresh)

        return metrics

    def assess_lidar_quality(self, lidar_data: List[float], gt_lidar: List[float] = None) -> Dict[str, float]:
        """Assess quality of LiDAR data"""
        metrics = {}

        # Basic statistics
        valid_ranges = [r for r in lidar_data if r > 0 and r < float('inf')]
        if valid_ranges:
            metrics['mean_range'] = float(np.mean(valid_ranges))
            metrics['std_range'] = float(np.std(valid_ranges))
            metrics['min_range'] = float(np.min(valid_ranges))
            metrics['max_range'] = float(np.max(valid_ranges))
            metrics['valid_beam_percentage'] = len(valid_ranges) / len(lidar_data) * 100
        else:
            metrics['mean_range'] = 0
            metrics['std_range'] = 0
            metrics['valid_beam_percentage'] = 0

        # If ground truth is available
        if gt_lidar is not None and len(lidar_data) == len(gt_lidar):
            valid_comparison = []
            for r1, r2 in zip(lidar_data, gt_lidar):
                if r1 > 0 and r1 < float('inf') and r2 > 0 and r2 < float('inf'):
                    valid_comparison.append(abs(r1 - r2))

            if valid_comparison:
                diff = np.array(valid_comparison)
                metrics['lidar_mae'] = float(np.mean(diff))
                metrics['lidar_rmse'] = float(np.sqrt(np.mean(diff**2)))

        return metrics

    def generate_quality_report(self, image_metrics_list: List[Dict],
                              depth_metrics_list: List[Dict] = None,
                              lidar_metrics_list: List[Dict] = None) -> Dict:
        """Generate comprehensive quality report"""
        report = {
            'timestamp': '2023-12-09T10:00:00Z',
            'total_samples': len(image_metrics_list),
            'image_quality': {},
            'depth_quality': {},
            'lidar_quality': {}
        }

        # Image quality summary
        if image_metrics_list:
            sharpness_vals = [m.get('sharpness', 0) for m in image_metrics_list]
            contrast_vals = [m.get('contrast', 0) for m in image_metrics_list]
            brightness_vals = [m.get('brightness', 0) for m in image_metrics_list]
            entropy_vals = [m.get('entropy', 0) for m in image_metrics_list]

            report['image_quality'] = {
                'sharpness_mean': float(np.mean(sharpness_vals)),
                'sharpness_std': float(np.std(sharpness_vals)),
                'contrast_mean': float(np.mean(contrast_vals)),
                'brightness_mean': float(np.mean(brightness_vals)),
                'entropy_mean': float(np.mean(entropy_vals)),
                'quality_score': self.calculate_image_quality_score(image_metrics_list)
            }

        # Depth quality summary (if available)
        if depth_metrics_list:
            mean_depth_vals = [m.get('mean_depth', 0) for m in depth_metrics_list]
            valid_pct_vals = [m.get('valid_pixel_percentage', 0) for m in depth_metrics_list]

            report['depth_quality'] = {
                'mean_depth_mean': float(np.mean(mean_depth_vals)),
                'valid_pixel_percentage_mean': float(np.mean(valid_pct_vals)),
                'quality_score': self.calculate_depth_quality_score(depth_metrics_list)
            }

        # LiDAR quality summary (if available)
        if lidar_metrics_list:
            mean_range_vals = [m.get('mean_range', 0) for m in lidar_metrics_list]
            valid_beam_pct_vals = [m.get('valid_beam_percentage', 0) for m in lidar_metrics_list]

            report['lidar_quality'] = {
                'mean_range_mean': float(np.mean(mean_range_vals)),
                'valid_beam_percentage_mean': float(np.mean(valid_beam_pct_vals)),
                'quality_score': self.calculate_lidar_quality_score(lidar_metrics_list)
            }

        return report

    def calculate_image_quality_score(self, metrics_list: List[Dict]) -> float:
        """Calculate overall image quality score (0-1 scale)"""
        scores = []
        for metrics in metrics_list:
            # Normalize each metric to 0-1 scale and weight them
            sharpness_score = min(1.0, metrics.get('sharpness', 0) / 1000)  # Adjust threshold as needed
            contrast_score = min(1.0, metrics.get('contrast', 0) / 50)  # Adjust threshold as needed
            entropy_score = min(1.0, metrics.get('entropy', 0) / 7)  # Max entropy for 8-bit is ~8

            # Weighted average (adjust weights based on importance)
            sample_score = 0.3 * sharpness_score + 0.3 * contrast_score + 0.4 * entropy_score
            scores.append(sample_score)

        return float(np.mean(scores)) if scores else 0.0

    def calculate_depth_quality_score(self, metrics_list: List[Dict]) -> float:
        """Calculate overall depth quality score (0-1 scale)"""
        scores = []
        for metrics in metrics_list:
            # Valid pixel percentage contributes significantly to quality
            valid_pct_score = metrics.get('valid_pixel_percentage', 0) / 100.0

            # Depth range reasonableness
            mean_depth = metrics.get('mean_depth', 0)
            depth_reasonable_score = 1.0 if 0.1 <= mean_depth <= 20.0 else 0.5  # Penalize unreasonable depths

            sample_score = 0.7 * valid_pct_score + 0.3 * depth_reasonable_score
            scores.append(sample_score)

        return float(np.mean(scores)) if scores else 0.0

    def calculate_lidar_quality_score(self, metrics_list: List[Dict]) -> float:
        """Calculate overall LiDAR quality score (0-1 scale)"""
        scores = []
        for metrics in metrics_list:
            # Valid beam percentage
            valid_beam_score = metrics.get('valid_beam_percentage', 0) / 100.0

            # Range reasonableness
            mean_range = metrics.get('mean_range', 0)
            range_reasonable_score = 1.0 if 0.1 <= mean_range <= 50.0 else 0.5

            sample_score = 0.6 * valid_beam_score + 0.4 * range_reasonable_score
            scores.append(sample_score)

        return float(np.mean(scores)) if scores else 0.0

    def visualize_quality_metrics(self, metrics_list: List[Dict], title: str = "Quality Metrics"):
        """Create visualization of quality metrics"""
        if not metrics_list:
            return

        # Extract metrics
        sharpness_vals = [m.get('sharpness', 0) for m in metrics_list]
        contrast_vals = [m.get('contrast', 0) for m in metrics_list]
        brightness_vals = [m.get('brightness', 0) for m in metrics_list]
        entropy_vals = [m.get('entropy', 0) for m in metrics_list]

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(title)

        # Sharpness histogram
        axes[0, 0].hist(sharpness_vals, bins=50, alpha=0.7)
        axes[0, 0].set_title('Sharpness Distribution')
        axes[0, 0].set_xlabel('Laplacian Variance')
        axes[0, 0].set_ylabel('Frequency')

        # Contrast histogram
        axes[0, 1].hist(contrast_vals, bins=50, alpha=0.7, color='orange')
        axes[0, 1].set_title('Contrast Distribution')
        axes[0, 1].set_xlabel('Standard Deviation')
        axes[0, 1].set_ylabel('Frequency')

        # Brightness histogram
        axes[1, 0].hist(brightness_vals, bins=50, alpha=0.7, color='green')
        axes[1, 0].set_title('Brightness Distribution')
        axes[1, 0].set_xlabel('Mean Intensity')
        axes[1, 0].set_ylabel('Frequency')

        # Entropy histogram
        axes[1, 1].hist(entropy_vals, bins=50, alpha=0.7, color='red')
        axes[1, 1].set_title('Entropy Distribution')
        axes[1, 1].set_xlabel('Entropy')
        axes[1, 1].set_ylabel('Frequency')

        plt.tight_layout()
        plt.show()

def validate_synthetic_dataset(dataset_path: str):
    """Validate an entire synthetic dataset"""
    validator = DataQualityAssessment()

    # Load and analyze sample images
    image_metrics = []
    depth_metrics = []
    lidar_metrics = []

    # This would iterate through the dataset and collect metrics
    # For demonstration, we'll create some sample metrics
    for i in range(100):  # Sample first 100 images
        # Simulate loading and assessing a synthetic image
        sample_image = np.random.rand(480, 640, 3)  # Random image for demo
        metrics = validator.assess_image_quality(sample_image)
        image_metrics.append(metrics)

    # Generate quality report
    quality_report = validator.generate_quality_report(image_metrics)

    # Print summary
    print("Synthetic Dataset Quality Report:")
    print(f"Total samples analyzed: {quality_report['total_samples']}")
    print(f"Image quality score: {quality_report['image_quality']['quality_score']:.3f}")
    print(f"Average sharpness: {quality_report['image_quality']['sharpness_mean']:.2f}")
    print(f"Average contrast: {quality_report['image_quality']['contrast_mean']:.2f}")
    print(f"Average entropy: {quality_report['image_quality']['entropy_mean']:.2f}")

    return quality_report
```

## Performance Optimization

### Efficient Data Generation Pipeline

Optimize for high-throughput data generation:

```python
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import queue
import time

class EfficientDataGenerator:
    def __init__(self, num_processes=4, batch_size=10):
        self.num_processes = num_processes
        self.batch_size = batch_size
        self.process_pool = ProcessPoolExecutor(max_workers=num_processes)
        self.data_queue = queue.Queue(maxsize=100)  # Bounded queue to control memory usage

    def generate_batch_parallel(self, num_samples, config_func):
        """Generate a batch of data samples in parallel"""
        configs = [config_func() for _ in range(num_samples)]

        # Submit jobs to process pool
        futures = []
        for config in configs:
            future = self.process_pool.submit(self.generate_single_sample, config)
            futures.append(future)

        # Collect results
        results = []
        for future in futures:
            result = future.result()
            results.append(result)

        return results

    def generate_single_sample(self, config):
        """Generate a single data sample with given configuration"""
        # This would typically involve:
        # 1. Setting up Gazebo/Unity scene with config
        # 2. Running simulation for required time
        # 3. Capturing sensor data
        # 4. Generating annotations
        # 5. Saving data to disk

        # For demonstration, return a mock result
        import numpy as np
        sample_data = {
            'rgb_image': np.random.rand(480, 640, 3).astype(np.float32),
            'depth_image': np.random.rand(480, 640).astype(np.float32),
            'lidar_data': np.random.rand(360).astype(np.float32),
            'annotations': {'objects': []},
            'metadata': config
        }
        return sample_data

    def generate_dataset_streaming(self, total_samples, config_func, output_dir):
        """Generate dataset with streaming to disk to manage memory"""
        batch_size = self.batch_size
        completed_samples = 0

        while completed_samples < total_samples:
            current_batch_size = min(batch_size, total_samples - completed_samples)

            # Generate batch
            batch_results = self.generate_batch_parallel(current_batch_size, config_func)

            # Save batch to disk
            for i, sample in enumerate(batch_results):
                sample_id = completed_samples + i
                self.save_sample_to_disk(sample, sample_id, output_dir)

            completed_samples += len(batch_results)
            print(f"Completed {completed_samples}/{total_samples} samples")

    def save_sample_to_disk(self, sample_data, sample_id, output_dir):
        """Save individual sample to disk"""
        import os
        import json
        import cv2

        # Create sample directory
        sample_dir = os.path.join(output_dir, f"sample_{sample_id:06d}")
        os.makedirs(sample_dir, exist_ok=True)

        # Save RGB image
        rgb_path = os.path.join(sample_dir, "rgb.png")
        cv2.imwrite(rgb_path, (sample_data['rgb_image'] * 255).astype(np.uint8))

        # Save depth image
        depth_path = os.path.join(sample_dir, "depth.exr")
        cv2.imwrite(depth_path, sample_data['depth_image'])

        # Save LiDAR data
        lidar_path = os.path.join(sample_dir, "lidar.json")
        with open(lidar_path, 'w') as f:
            json.dump(sample_data['lidar_data'].tolist(), f)

        # Save annotations
        annotations_path = os.path.join(sample_dir, "annotations.json")
        with open(annotations_path, 'w') as f:
            json.dump(sample_data['annotations'], f, indent=2)

        # Save metadata
        metadata_path = os.path.join(sample_dir, "metadata.json")
        with open(metadata_path, 'w') as f:
            json.dump(sample_data['metadata'], f, indent=2)

    def benchmark_generation_performance(self, num_samples=100):
        """Benchmark data generation performance"""
        start_time = time.time()

        def dummy_config():
            return {"random_param": np.random.rand()}

        # Generate samples
        results = self.generate_batch_parallel(num_samples, dummy_config)

        end_time = time.time()
        duration = end_time - start_time
        rate = num_samples / duration

        print(f"Generated {num_samples} samples in {duration:.2f}s")
        print(f"Generation rate: {rate:.2f} samples/sec")
        print(f"Average per sample: {duration/num_samples*1000:.2f} ms")

        return {
            'total_samples': num_samples,
            'duration': duration,
            'generation_rate': rate,
            'samples_per_second': rate
        }

# Example usage
def run_efficient_generation():
    """Example of efficient data generation"""
    generator = EfficientDataGenerator(num_processes=4, batch_size=20)

    def create_random_config():
        """Create random configuration for data generation"""
        return {
            'lighting': {
                'intensity': np.random.uniform(0.5, 2.0),
                'direction': [np.random.uniform(-1, 1) for _ in range(3)],
                'color_temp': np.random.uniform(3000, 8000)
            },
            'objects': [
                {
                    'type': np.random.choice(['box', 'cylinder', 'sphere']),
                    'position': [np.random.uniform(-3, 3) for _ in range(3)],
                    'size': [np.random.uniform(0.1, 1.0) for _ in range(3)],
                    'material': np.random.choice(['metal', 'plastic', 'wood'])
                } for _ in range(np.random.randint(1, 5))
            ],
            'camera_pose': {
                'position': [np.random.uniform(-2, 2) for _ in range(3)],
                'rotation': [np.random.uniform(-np.pi, np.pi) for _ in range(3)]
            }
        }

    # Benchmark performance
    performance_metrics = generator.benchmark_generation_performance(50)

    # Generate actual dataset
    # generator.generate_dataset_streaming(
    #     total_samples=1000,
    #     config_func=create_random_config,
    #     output_dir="./synthetic_dataset"
    # )

    return performance_metrics
```

## Best Practices for Synthetic Data Generation

### 1. Domain Randomization Strategy

Implement systematic variation to improve model generalization:

```python
class DomainRandomizationStrategy:
    def __init__(self):
        self.strategies = {
            'lighting': {
                'intensity_range': (0.3, 2.0),
                'color_temperature_range': (3000, 8000),
                'position_variation': 2.0
            },
            'materials': {
                'roughness_range': (0.1, 0.9),
                'metallic_range': (0.0, 1.0),
                'albedo_range': (0.1, 1.0)
            },
            'geometry': {
                'size_variation': 0.5,
                'position_bounds': (-5, 5),
                'rotation_range': (0, 360)
            },
            'sensor': {
                'noise_levels': (0.001, 0.05),
                'exposure_range': (-2, 2)
            }
        }

    def apply_randomization(self, scene_config, domain_level='medium'):
        """Apply domain randomization based on level"""
        levels = {
            'low': 0.3,
            'medium': 0.6,
            'high': 1.0
        }

        factor = levels.get(domain_level, 0.6)

        # Apply lighting randomization
        lighting_factor = factor * 0.8  # Lighting is very important
        scene_config['lighting']['intensity'] *= np.random.uniform(
            1 - lighting_factor * 0.7, 1 + lighting_factor * 1.0
        )

        # Apply material randomization
        material_factor = factor * 0.6
        for obj in scene_config.get('objects', []):
            if np.random.random() < material_factor:
                obj['material']['roughness'] = np.random.uniform(0.1, 0.9)
                obj['material']['metallic'] = np.random.uniform(0.0, 1.0)

        # Apply geometric randomization
        geo_factor = factor * 0.4
        for obj in scene_config.get('objects', []):
            if np.random.random() < geo_factor:
                # Randomize position
                obj['position'] = [
                    pos + np.random.uniform(-factor, factor) * 2
                    for pos in obj['position']
                ]

                # Randomize rotation
                obj['rotation'] = [
                    rot + np.random.uniform(-factor, factor) * 30
                    for rot in obj['rotation']
                ]

        return scene_config
```

### 2. Validation Against Real Data

Compare synthetic data to real data characteristics:

```python
def validate_synthetic_vs_real(synthetic_data, real_data):
    """Validate synthetic data against real data characteristics"""
    validation_results = {}

    # Compare statistical properties
    for data_type in ['rgb', 'depth', 'lidar']:
        if data_type in synthetic_data and data_type in real_data:
            syn_data = synthetic_data[data_type]
            real_data_type = real_data[data_type]

            # Calculate statistical similarity
            syn_stats = calculate_image_statistics(syn_data)
            real_stats = calculate_image_statistics(real_data_type)

            # Compare distributions
            similarity_score = compare_distributions(syn_stats, real_stats)
            validation_results[f'{data_type}_similarity'] = similarity_score

    return validation_results

def calculate_image_statistics(image):
    """Calculate statistical properties of an image"""
    if len(image.shape) == 3:  # Color image
        gray = cv2.cvtColor((image * 255).astype(np.uint8), cv2.COLOR_RGB2GRAY)
    else:
        gray = (image * 255).astype(np.uint8)

    stats = {
        'mean': float(np.mean(gray)),
        'std': float(np.std(gray)),
        'skewness': float(calculate_skewness(gray)),
        'kurtosis': float(calculate_kurtosis(gray)),
        'histogram': np.histogram(gray.flatten(), bins=64)[0].tolist()
    }

    return stats

def calculate_skewness(data):
    """Calculate skewness of data distribution"""
    mean = np.mean(data)
    std = np.std(data)
    n = len(data)
    skewness = (n / ((n - 1) * (n - 2))) * np.sum(((data - mean) / std) ** 3)
    return skewness

def calculate_kurtosis(data):
    """Calculate kurtosis of data distribution"""
    mean = np.mean(data)
    std = np.std(data)
    n = len(data)
    kurtosis = (n * (n + 1) / ((n - 1) * (n - 2) * (n - 3))) * np.sum(((data - mean) / std) ** 4) - (3 * (n - 1) ** 2 / ((n - 2) * (n - 3)))
    return kurtosis

def compare_distributions(stats1, stats2):
    """Compare two sets of statistical properties"""
    # Simple comparison of key statistics
    mean_diff = abs(stats1['mean'] - stats2['mean']) / max(stats2['mean'], 1e-6)
    std_diff = abs(stats1['std'] - stats2['std']) / max(stats2['std'], 1e-6)

    # Histogram similarity (intersection over union)
    hist1 = np.array(stats1['histogram'])
    hist2 = np.array(stats2['histogram'])
    hist_intersection = np.sum(np.minimum(hist1, hist2))
    hist_union = np.sum(np.maximum(hist1, hist2))
    hist_similarity = hist_intersection / hist_union if hist_union > 0 else 0

    # Overall similarity score (weighted average)
    similarity = 0.3 * (1 - min(mean_diff, 1)) + \
                 0.3 * (1 - min(std_diff, 1)) + \
                 0.4 * hist_similarity

    return float(similarity)
```

## Troubleshooting Common Issues

### Performance Issues

1. **Slow Generation**: Use parallel processing and optimize rendering settings
2. **Memory Overflow**: Implement streaming and batch processing
3. **Disk Space**: Use efficient compression and selective saving

### Quality Issues

1. **Unrealistic Data**: Validate against real sensor specifications
2. **Inconsistent Annotations**: Implement validation checks
3. **Missing Edge Cases**: Expand domain randomization

### Integration Issues

1. **Format Compatibility**: Ensure proper message formats
2. **Synchronization**: Maintain proper timing relationships
3. **Coordinate Systems**: Verify consistent coordinate frames

## Summary

This tutorial covered comprehensive synthetic data generation for robotics applications, including:
- Gazebo-based sensor simulation with realistic noise models
- Unity-based high-fidelity rendering and data capture
- Domain randomization for improved model generalization
- Automatic annotation generation for training data
- Quality assessment and validation techniques
- Performance optimization for large-scale generation
- ROS 2 integration for real-time applications

Synthetic data generation is crucial for robotics development, enabling the creation of large, diverse, and perfectly annotated datasets without the costs and limitations of real-world data collection. The combination of Gazebo physics simulation and Unity rendering provides a powerful platform for generating realistic training data for computer vision, navigation, and control applications.

## References

1. Shapira, O., et al. (2019). "Synthetic Data Generation for End-to-End Thermal Infrared Tracking." arXiv preprint arXiv:1909.00695.
2. Tremblay, J., et al. (2018). "Training Deep Networks with Synthetic Data: Bridging the Reality Gap by Domain Randomization." IEEE Conference on Computer Vision and Pattern Recognition Workshops.
3. Johnson-Roberson, M., et al. (2017). "Driving in the Matrix: Can Virtual Worlds Replace Human-Generated Annotations for Real World Tasks?" IEEE Robotics and Automation Letters.
4. Open Source Robotics Foundation. (2023). Gazebo Sensor Simulation Documentation. http://gazebosim.org/tutorials?tut=sensor_simulation

## Exercises

1. Implement a domain randomization system for your specific robotics application
2. Create a validation pipeline to compare synthetic and real sensor data
3. Develop an automatic annotation system for your robot's specific objects
4. Optimize your data generation pipeline for your hardware constraints
5. Validate the effectiveness of synthetic data on a real-world robotics task