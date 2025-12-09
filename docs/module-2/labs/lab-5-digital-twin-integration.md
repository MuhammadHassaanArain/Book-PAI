# Lab 5: Digital Twin Integration

## Objective

In this final lab, you will integrate all components of the digital twin system: physics simulation in Gazebo, environment building, sensor simulation, and Unity visualization. You will create a complete pipeline from Gazebo → Unity → ROS 2 and validate the full system integration.

## Prerequisites

- Completed all previous labs
- Working Gazebo simulation with all sensors
- Unity visualization system
- ROS 2 communication infrastructure
- Understanding of all individual components

## Lab Tasks

### Task 1: Create Complete Digital Twin Architecture

Design and implement the complete digital twin system architecture:

```yaml
# digital_twin_architecture.yaml
system:
  name: "Full Digital Twin System"
  description: "Complete digital twin with Gazebo physics, Unity visualization, and ROS 2 integration"

components:
  gazebo_simulation:
    name: "Gazebo Physics Engine"
    version: "Fortress/Harmonic"
    physics_engine: "bullet"
    update_rate: 1000
    real_time_factor: 1.0

  unity_visualization:
    name: "Unity Visualization Engine"
    version: "LTS"
    rendering_pipeline: "HDRP"
    sync_frequency: 60
    quality_settings: "High"

  ros2_infrastructure:
    name: "ROS 2 Communication Layer"
    version: "Humble/Iron"
    middleware: "FastDDS"
    topics:
      - "/robot1/scan"
      - "/robot1/rgb/image_raw"
      - "/robot1/depth/image_raw"
      - "/robot1/imu/data"
      - "/robot1/joint_states"
      - "/robot1/odom"

  sensor_simulation:
    lidar:
      type: "ray"
      samples: 360
      range_min: 0.1
      range_max: 30.0
      update_rate: 10

    camera:
      type: "camera"
      width: 640
      height: 480
      update_rate: 30

    imu:
      type: "imu"
      update_rate: 100

synchronization:
  frequency: 60
  latency_threshold: 0.1
  interpolation_enabled: true
  coordinate_conversion:
    gazebo_to_unity: true
    scale_factor: 1.0

performance:
  target_fps: 30
  max_objects: 50
  memory_limit: "4GB"
  cpu_cores: 8
```

### Task 2: Implement Gazebo-Unity Communication Bridge

Create a communication bridge between Gazebo and Unity:

```python
#!/usr/bin/env python3
# gazebo_unity_bridge.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
import numpy as np
import json
import websocket
import threading
import time

class GazeboUnityBridge(Node):
    def __init__(self):
        super().__init__('gazebo_unity_bridge')

        # Initialize WebSocket connection to Unity
        self.unity_connection = None
        self.connect_to_unity()

        # Create subscribers for all sensor data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.lidar_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/robot1/rgb/image_raw',
            self.camera_callback,
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

        self.joint_subscription = self.create_subscription(
            JointState,
            '/robot1/joint_states',
            self.joint_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10
        )

        # Create publishers for Unity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot1/cmd_vel',
            10
        )

        self.get_logger().info('Gazebo-Unity Bridge initialized')

    def connect_to_unity(self):
        """Establish connection to Unity via WebSocket"""
        try:
            self.unity_connection = websocket.WebSocketApp(
                "ws://localhost:8080",
                on_open=self.on_unity_connect,
                on_message=self.on_unity_message,
                on_error=self.on_unity_error,
                on_close=self.on_unity_close
            )

            # Run WebSocket in separate thread
            self.ws_thread = threading.Thread(target=self.unity_connection.run_forever)
            self.ws_thread.daemon = True
            self.ws_thread.start()

            self.get_logger().info('Attempting to connect to Unity...')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Unity: {e}')

    def on_unity_connect(self, ws):
        self.get_logger().info('Connected to Unity')

    def on_unity_message(self, ws, message):
        try:
            data = json.loads(message)
            self.handle_unity_command(data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON from Unity: {message}')

    def on_unity_error(self, ws, error):
        self.get_logger().error(f'Unity connection error: {error}')

    def on_unity_close(self, ws, close_status_code, close_msg):
        self.get_logger().info('Unity connection closed')

    def lidar_callback(self, msg):
        """Process LiDAR data and send to Unity"""
        lidar_data = {
            'type': 'lidar',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'ranges': list(msg.ranges),
            'intensities': list(msg.intensities),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }

        self.send_to_unity(lidar_data)

    def camera_callback(self, msg):
        """Process camera data and send to Unity"""
        # Convert image to compressed format for transmission
        image_data = {
            'type': 'camera',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'data': self.compress_image_data(msg.data)
        }

        self.send_to_unity(image_data)

    def depth_callback(self, msg):
        """Process depth data and send to Unity"""
        depth_data = {
            'type': 'depth',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'data': self.compress_depth_data(msg.data)
        }

        self.send_to_unity(depth_data)

    def imu_callback(self, msg):
        """Process IMU data and send to Unity"""
        imu_data = {
            'type': 'imu',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }

        self.send_to_unity(imu_data)

    def joint_callback(self, msg):
        """Process joint state data and send to Unity"""
        joint_data = {
            'type': 'joint_state',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort)
        }

        self.send_to_unity(joint_data)

    def odom_callback(self, msg):
        """Process odometry data and send to Unity"""
        odom_data = {
            'type': 'odometry',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'pose': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            },
            'twist': {
                'linear': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            }
        }

        self.send_to_unity(odom_data)

    def send_to_unity(self, data):
        """Send data to Unity via WebSocket"""
        if self.unity_connection and self.unity_connection.sock.connected:
            try:
                json_data = json.dumps(data)
                self.unity_connection.send(json_data)
            except Exception as e:
                self.get_logger().error(f'Failed to send data to Unity: {e}')

    def handle_unity_command(self, data):
        """Handle commands received from Unity"""
        command_type = data.get('type')

        if command_type == 'cmd_vel':
            self.publish_velocity_command(data)
        elif command_type == 'teleop':
            self.handle_teleoperation(data)
        elif command_type == 'request_state':
            self.send_current_state()
        else:
            self.get_logger().warn(f'Unknown command type: {command_type}')

    def publish_velocity_command(self, data):
        """Publish velocity command to robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = data.get('linear_x', 0.0)
        cmd_vel.linear.y = data.get('linear_y', 0.0)
        cmd_vel.linear.z = data.get('linear_z', 0.0)
        cmd_vel.angular.x = data.get('angular_x', 0.0)
        cmd_vel.angular.y = data.get('angular_y', 0.0)
        cmd_vel.angular.z = data.get('angular_z', 0.0)

        self.cmd_vel_publisher.publish(cmd_vel)

    def handle_teleoperation(self, data):
        """Handle teleoperation commands from Unity"""
        # Implement teleoperation logic
        pass

    def send_current_state(self):
        """Send current robot state to Unity"""
        # This would involve collecting current state from various topics
        # and sending it to Unity
        pass

    def compress_image_data(self, raw_data):
        """Compress image data for transmission"""
        # In a real implementation, you would compress the image data
        # For now, return a placeholder
        return raw_data[:100]  # Just first 100 bytes as example

    def compress_depth_data(self, raw_data):
        """Compress depth data for transmission"""
        # Compress depth data
        return raw_data[:100]  # Just first 100 bytes as example

def main(args=None):
    rclpy.init(args=args)
    bridge = GazeboUnityBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 3: Create Unity Digital Twin Manager

Implement the Unity side of the digital twin system:

```csharp
// DigitalTwinManager.cs
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using WebSocketSharp;
using Newtonsoft.Json;

public class DigitalTwinManager : MonoBehaviour
{
    [Header("Connection Settings")]
    public string gazeboWebSocketUrl = "ws://localhost:8080";
    public float syncFrequency = 60.0f;
    public bool enableCompression = true;

    [Header("Digital Twin Components")]
    public GameObject robotModel;
    public Camera unityCamera;
    public GameObject[] sensorObjects;
    public GameObject environment;

    [Header("Synchronization Settings")]
    public bool enableInterpolation = true;
    public float interpolationTime = 0.1f;
    public bool enableExtrapolation = true;

    private WebSocket webSocket;
    private Dictionary<string, object> gazeboData = new Dictionary<string, object>();
    private Dictionary<string, object> unityData = new Dictionary<string, object>();

    private bool isConnected = false;
    private float lastSyncTime = 0f;

    void Start()
    {
        ConnectToGazebo();
        StartCoroutine(SynchronizationLoop());
    }

    void ConnectToGazebo()
    {
        try
        {
            webSocket = new WebSocket(gazeboWebSocketUrl);

            webSocket.OnOpen += (sender, e) =>
            {
                isConnected = true;
                Debug.Log("Connected to Gazebo bridge");

                // Request initial state
                SendCommand(new { type = "request_state" });
            };

            webSocket.OnMessage += (sender, e) =>
            {
                ProcessGazeboData(e.Data);
            };

            webSocket.OnError += (sender, e) =>
            {
                Debug.LogError($"WebSocket error: {e.Message}");
                isConnected = false;
            };

            webSocket.OnClose += (sender, e) =>
            {
                Debug.Log("WebSocket connection closed");
                isConnected = false;
            };

            webSocket.Connect();
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to connect to Gazebo: {e.Message}");
        }
    }

    IEnumerator SynchronizationLoop()
    {
        float syncInterval = 1.0f / syncFrequency;

        while (true)
        {
            if (isConnected)
            {
                // Update Unity objects based on Gazebo data
                UpdateUnityObjects();

                // Send Unity commands to Gazebo if needed
                SendUnityCommands();
            }

            yield return new WaitForSeconds(syncInterval);
        }
    }

    void ProcessGazeboData(string jsonData)
    {
        try
        {
            var data = JsonConvert.DeserializeObject<Dictionary<string, object>>(jsonData);
            string dataType = data["type"].ToString();

            gazeboData[dataType] = data;

            // Update specific components based on data type
            switch (dataType)
            {
                case "lidar":
                    UpdateLidarVisualization(data);
                    break;
                case "camera":
                    UpdateCameraVisualization(data);
                    break;
                case "imu":
                    UpdateIMUVisualization(data);
                    break;
                case "joint_state":
                    UpdateJointVisualization(data);
                    break;
                case "odometry":
                    UpdateOdometryVisualization(data);
                    break;
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error processing Gazebo data: {e.Message}");
        }
    }

    void UpdateUnityObjects()
    {
        // Update robot model based on odometry data
        if (gazeboData.ContainsKey("odometry"))
        {
            var odomData = (Dictionary<string, object>)gazeboData["odometry"];
            var poseData = (Dictionary<string, object>)((Dictionary<string, object>)odomData["pose"]);
            var positionData = (Dictionary<string, object>)((Dictionary<string, object>)poseData["position"]);

            Vector3 targetPosition = new Vector3(
                Convert.ToDouble(positionData["x"]),
                Convert.ToDouble(positionData["y"]),
                Convert.ToDouble(positionData["z"])
            );

            // Apply coordinate conversion
            targetPosition = ConvertGazeboToUnityPosition(targetPosition);

            if (enableInterpolation)
            {
                robotModel.transform.position = Vector3.Lerp(
                    robotModel.transform.position,
                    targetPosition,
                    Time.deltaTime / interpolationTime
                );
            }
            else
            {
                robotModel.transform.position = targetPosition;
            }
        }

        // Update sensor objects
        foreach (var sensorObj in sensorObjects)
        {
            UpdateSensorObject(sensorObj);
        }

        // Update environment based on sensor data
        UpdateEnvironment();
    }

    void UpdateLidarVisualization(Dictionary<string, object> lidarData)
    {
        // Update LiDAR visualization in Unity
        // This could involve drawing lines or point clouds
        var ranges = (Newtonsoft.Json.Linq.JArray)lidarData["ranges"];
        var angleMin = Convert.ToDouble(lidarData["angle_min"]);
        var angleInc = Convert.ToDouble(lidarData["angle_increment"]);

        // Create LiDAR visualization
        DrawLidarPoints(ranges, angleMin, angleInc);
    }

    void UpdateCameraVisualization(Dictionary<string, object> cameraData)
    {
        // Update camera feed visualization
        // This would involve updating a texture with camera data
        UpdateCameraFeed(cameraData);
    }

    void UpdateIMUVisualization(Dictionary<string, object> imuData)
    {
        // Update IMU visualization
        var orientationData = (Dictionary<string, object>)((Dictionary<string, object>)imuData["orientation"]);

        Quaternion targetRotation = new Quaternion(
            Convert.ToSingle(orientationData["x"]),
            Convert.ToSingle(orientationData["y"]),
            Convert.ToSingle(orientationData["z"]),
            Convert.ToSingle(orientationData["w"])
        );

        // Apply coordinate conversion
        targetRotation = ConvertGazeboToUnityRotation(targetRotation);

        if (enableInterpolation)
        {
            robotModel.transform.rotation = Quaternion.Slerp(
                robotModel.transform.rotation,
                targetRotation,
                Time.deltaTime / interpolationTime
            );
        }
        else
        {
            robotModel.transform.rotation = targetRotation;
        }
    }

    void UpdateJointVisualization(Dictionary<string, object> jointData)
    {
        // Update joint positions based on joint state data
        var jointNames = (Newtonsoft.Json.Linq.JArray)((Dictionary<string, object>)jointData)["names"];
        var jointPositions = (Newtonsoft.Json.Linq.JArray)((Dictionary<string, object>)jointData)["positions"];

        for (int i = 0; i < jointNames.Count; i++)
        {
            string jointName = jointNames[i].ToString();
            float jointPosition = Convert.ToSingle(jointPositions[i]);

            UpdateJoint(jointName, jointPosition);
        }
    }

    void UpdateOdometryVisualization(Dictionary<string, object> odomData)
    {
        // Already handled in UpdateUnityObjects
    }

    void DrawLidarPoints(Newtonsoft.Json.Linq.JArray ranges, double angleMin, double angleInc)
    {
        // Draw LiDAR points in Unity scene
        GameObject lidarViz = GameObject.Find("LidarVisualization");
        if (lidarViz == null)
        {
            lidarViz = new GameObject("LidarVisualization");
        }

        // Clear previous points
        foreach (Transform child in lidarViz.transform)
        {
            Destroy(child.gameObject);
        }

        // Create new points
        for (int i = 0; i < ranges.Count; i++)
        {
            double range = Convert.ToDouble(ranges[i]);
            double angle = angleMin + i * angleInc;

            if (range > 0 && range < 30) // Valid range
            {
                Vector3 pointPos = new Vector3(
                    (float)(range * Math.Cos(angle)),
                    0.1f, // Height
                    (float)(range * Math.Sin(angle))
                );

                // Apply coordinate conversion
                pointPos = ConvertGazeboToUnityPosition(pointPos);

                GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                point.transform.position = pointPos;
                point.transform.localScale = Vector3.one * 0.05f;
                point.GetComponent<Renderer>().material.color = Color.red;
                point.transform.parent = lidarViz.transform;

                // Remove collider for performance
                DestroyImmediate(point.GetComponent<Collider>());
            }
        }
    }

    void UpdateCameraFeed(Dictionary<string, object> cameraData)
    {
        // Update camera texture with received image data
        // This is a simplified example - in practice, you'd need to decode the image data
    }

    void UpdateJoint(string jointName, float position)
    {
        // Find joint in robot model and update its position
        Transform jointTransform = robotModel.transform.Find(jointName);
        if (jointTransform != null)
        {
            // Update joint position/rotation based on type
            // This would depend on the joint type (revolute, prismatic, etc.)
            jointTransform.localRotation = Quaternion.Euler(0, position * Mathf.Rad2Deg, 0);
        }
    }

    void UpdateSensorObject(GameObject sensorObj)
    {
        // Update sensor visualization based on sensor data
        string sensorType = sensorObj.tag; // Assuming sensor type is stored in tag

        switch (sensorType)
        {
            case "LiDAR":
                // Update LiDAR visualization
                break;
            case "Camera":
                // Update camera visualization
                break;
            case "IMU":
                // Update IMU visualization
                break;
        }
    }

    void UpdateEnvironment()
    {
        // Update environment based on sensor data
        // For example, update lighting based on camera exposure
    }

    void SendUnityCommands()
    {
        // Send Unity commands back to Gazebo if needed
        // This could include user input, teleoperation commands, etc.

        if (Input.GetKeyDown(KeyCode.Space))
        {
            // Send stop command
            SendCommand(new { type = "cmd_vel", linear_x = 0.0f, angular_z = 0.0f });
        }

        // Send velocity commands based on input
        float linearVel = Input.GetAxis("Vertical");
        float angularVel = Input.GetAxis("Horizontal");

        if (Mathf.Abs(linearVel) > 0.1f || Mathf.Abs(angularVel) > 0.1f)
        {
            SendCommand(new {
                type = "cmd_vel",
                linear_x = linearVel,
                angular_z = angularVel
            });
        }
    }

    void SendCommand(object command)
    {
        if (webSocket != null && webSocket.IsAlive)
        {
            string jsonCommand = JsonConvert.SerializeObject(command);
            webSocket.Send(jsonCommand);
        }
    }

    Vector3 ConvertGazeboToUnityPosition(Vector3 gazeboPos)
    {
        // Convert from Gazebo coordinate system (X-forward, Y-left, Z-up)
        // to Unity coordinate system (X-right, Y-up, Z-forward)
        return new Vector3(-gazeboPos.y, gazeboPos.z, gazeboPos.x);
    }

    Quaternion ConvertGazeboToUnityRotation(Quaternion gazeboRot)
    {
        // Convert rotation quaternion from Gazebo to Unity coordinate system
        return new Quaternion(-gazeboRot.z, gazeboRot.x, -gazeboRot.y, gazeboRot.w);
    }

    void OnDestroy()
    {
        if (webSocket != null)
        {
            webSocket.Close();
        }
    }
}
```

### Task 4: Implement Performance Monitoring

Create a performance monitoring system for the digital twin:

```python
#!/usr/bin/env python3
# performance_monitor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image, LaserScan
from builtin_interfaces.msg import Time
import time
import psutil
import threading
from collections import deque
import json

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # Performance tracking
        self.fps_history = deque(maxlen=100)
        self.cpu_history = deque(maxlen=100)
        self.memory_history = deque(maxlen=100)
        self.network_history = deque(maxlen=100)
        self.latency_history = deque(maxlen=100)

        # Publishers for performance data
        self.fps_publisher = self.create_publisher(Float32, '/performance/fps', 10)
        self.cpu_publisher = self.create_publisher(Float32, '/performance/cpu', 10)
        self.memory_publisher = self.create_publisher(Float32, '/performance/memory', 10)
        self.status_publisher = self.create_publisher(String, '/performance/status', 10)

        # Subscribers for sensor data (to measure processing time)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.lidar_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/robot1/rgb/image_raw',
            self.camera_callback,
            10
        )

        # Timer for performance reporting
        self.performance_timer = self.create_timer(1.0, self.report_performance)

        # Performance counters
        self.frame_count = 0
        self.last_report_time = time.time()
        self.lidar_processing_times = deque(maxlen=50)
        self.camera_processing_times = deque(maxlen=50)

        self.get_logger().info('Performance Monitor initialized')

    def lidar_callback(self, msg):
        start_time = time.time()
        # Simulate processing time
        time.sleep(0.001)  # 1ms processing time
        processing_time = time.time() - start_time
        self.lidar_processing_times.append(processing_time)

    def camera_callback(self, msg):
        start_time = time.time()
        # Simulate processing time
        time.sleep(0.01)  # 10ms processing time
        processing_time = time.time() - start_time
        self.camera_processing_times.append(processing_time)

    def report_performance(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_report_time

        # Calculate FPS
        fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        self.fps_history.append(fps)
        self.frame_count = 0
        self.last_report_time = current_time

        # Get system metrics
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent
        network_io = psutil.net_io_counters()

        # Calculate averages
        avg_fps = sum(self.fps_history) / len(self.fps_history) if self.fps_history else 0
        avg_cpu = sum(self.cpu_history) / len(self.cpu_history) if self.cpu_history else cpu_percent
        avg_memory = sum(self.memory_history) / len(self.memory_history) if self.memory_history else memory_percent

        # Store metrics
        self.cpu_history.append(cpu_percent)
        self.memory_history.append(memory_percent)
        self.network_history.append(network_io.bytes_sent + network_io.bytes_recv)

        # Publish performance data
        fps_msg = Float32()
        fps_msg.data = avg_fps
        self.fps_publisher.publish(fps_msg)

        cpu_msg = Float32()
        cpu_msg.data = cpu_percent
        self.cpu_publisher.publish(cpu_msg)

        memory_msg = Float32()
        memory_msg.data = memory_percent
        self.memory_publisher.publish(memory_msg)

        # Create performance status message
        status_msg = String()
        status_data = {
            'timestamp': current_time,
            'fps': avg_fps,
            'cpu_percent': cpu_percent,
            'memory_percent': memory_percent,
            'lidar_processing_avg': sum(self.lidar_processing_times) / len(self.lidar_processing_times) if self.lidar_processing_times else 0,
            'camera_processing_avg': sum(self.camera_processing_times) / len(self.camera_processing_times) if self.camera_processing_times else 0,
            'status': self.get_system_status(avg_fps, cpu_percent, memory_percent)
        }
        status_msg.data = json.dumps(status_data)
        self.status_publisher.publish(status_msg)

        # Log performance summary
        self.get_logger().info(
            f'Performance: FPS={avg_fps:.1f}, CPU={cpu_percent:.1f}%, '
            f'Memory={memory_percent:.1f}%, '
            f'LidarProc={sum(self.lidar_processing_times)/len(self.lidar_processing_times)*1000:.1f}ms, '
            f'CameraProc={sum(self.camera_processing_times)/len(self.camera_processing_times)*1000:.1f}ms'
        )

    def get_system_status(self, fps, cpu, memory):
        """Determine system status based on performance metrics"""
        if fps < 15 or cpu > 80 or memory > 80:
            return "WARNING"
        elif fps < 5 or cpu > 90 or memory > 90:
            return "CRITICAL"
        else:
            return "OK"

    def increment_frame_count(self):
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab Exercises

1. **Full System Integration**: Connect all components (Gazebo, Unity, ROS 2) and verify communication
2. **Performance Optimization**: Optimize the system for real-time performance
3. **Latency Measurement**: Measure and minimize communication latency between components
4. **Data Consistency**: Verify data consistency across all system components
5. **Scalability Testing**: Test the system with multiple robots and sensors

## Validation Steps

1. Launch the complete digital twin system:
   ```bash
   ros2 launch your_robot_package digital_twin_system.launch.py
   ```

2. Start Unity visualization:
   ```bash
   # Run Unity project
   # Or use Unity's standalone build
   ```

3. Monitor system performance:
   ```bash
   ros2 run your_package performance_monitor
   ros2 topic echo /performance/status
   ```

4. Test all sensor data flows:
   ```bash
   ros2 topic echo /robot1/scan
   ros2 topic echo /robot1/rgb/image_raw
   ros2 topic echo /robot1/imu/data
   ```

5. Verify synchronization accuracy between Gazebo and Unity

## Expected Outcomes

- Fully integrated digital twin system
- Stable communication between all components
- Acceptable performance metrics (30+ FPS, `<50ms` latency)
- Proper sensor data flow from Gazebo to Unity
- Functional HRI interface
- Performance monitoring and optimization

## References

1. Open Source Robotics Foundation. (2023). Digital Twin Architecture for Robotics. https://gazebosim.org/docs/fortress/digital_twin
2. Unity Technologies. (2023). Unity Robotics Integration Guide. Unity Technologies.
3. ROS-Industrial Consortium. (2023). Digital Twin Best Practices. https://ros-industrial.github.io/industrial_training/