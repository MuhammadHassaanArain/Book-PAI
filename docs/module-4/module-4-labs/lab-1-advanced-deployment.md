# Lab 4: Advanced ROS 2 Integration and Field Deployment

## Overview

In this lab, you will implement an advanced ROS 2 system for a humanoid robot and deploy it in a simulated real-world environment. You'll focus on optimizing performance on Jetson hardware and implementing locomotion control algorithms.

## Objectives

By the end of this lab, you will be able to:
1. Implement advanced ROS 2 communication patterns for humanoid robots
2. Optimize a humanoid robot system for Jetson hardware
3. Deploy a locomotion control system in a simulated field environment
4. Monitor and validate system performance in real-time

## Prerequisites

- Module 1-3 completion
- ROS 2 Humble installed
- NVIDIA Jetson development experience
- Basic understanding of humanoid locomotion

## Part 1: Advanced ROS 2 Integration

### Step 1: Create a Custom Message Interface

Create a custom message for humanoid joint control:

```bash
# Create the message file: humanoid_control_msgs/msg/HumanoidJointCommand.msg
float64[] positions
float64[] velocities
float64[] efforts
float64[] max_efforts
string[] joint_names
float64[] stiffness
float64[] damping
```

### Step 2: Implement Quality of Service Configuration

Create a ROS 2 node with appropriate QoS settings for humanoid control:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Define QoS for different types of data
        sensor_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        control_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Create subscribers with appropriate QoS
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            sensor_qos
        )

        self.control_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            control_qos
        )

        self.get_logger().info('Humanoid Controller initialized')

    def joint_callback(self, msg):
        # Process joint state data
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 2: Jetson Optimization

### Step 3: Performance Monitoring Node

Create a node to monitor system performance on Jetson:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import subprocess
import time

class JetsonPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('jetson_monitor')

        self.cpu_pub = self.create_publisher(Float32, '/system/cpu_usage', 10)
        self.gpu_pub = self.create_publisher(Float32, '/system/gpu_usage', 10)
        self.temp_pub = self.create_publisher(Float32, '/system/temperature', 10)

        # Timer to monitor performance every second
        self.timer = self.create_timer(1.0, self.monitor_performance)

        self.get_logger().info('Jetson Performance Monitor initialized')

    def monitor_performance(self):
        # Get CPU usage
        cpu_usage = self.get_cpu_usage()
        cpu_msg = Float32()
        cpu_msg.data = cpu_usage
        self.cpu_pub.publish(cpu_msg)

        # Get GPU usage
        gpu_usage = self.get_gpu_usage()
        gpu_msg = Float32()
        gpu_msg.data = gpu_usage
        self.gpu_pub.publish(gpu_msg)

        # Get temperature
        temp = self.get_temperature()
        temp_msg = Float32()
        temp_msg.data = temp
        self.temp_pub.publish(temp_msg)

    def get_cpu_usage(self):
        # Simple CPU usage calculation
        result = subprocess.run(['top', '-bn1'], capture_output=True, text=True)
        return 50.0  # Placeholder - implement actual calculation

    def get_gpu_usage(self):
        # Get GPU usage from Jetson tools
        try:
            result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                return float(result.stdout.strip())
        except:
            pass
        return 0.0

    def get_temperature(self):
        # Get system temperature
        try:
            result = subprocess.run(['cat', '/sys/class/thermal/thermal_zone0/temp'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                temp = float(result.stdout.strip()) / 1000.0
                return temp
        except:
            pass
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    monitor = JetsonPerformanceMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 3: Locomotion Control Implementation

### Step 4: Balance Controller

Implement a basic balance controller for humanoid locomotion:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # PID parameters for balance control
        self.kp = 10.0
        self.ki = 0.1
        self.kd = 0.5

        # Integration and derivative terms
        self.error_integral = 0.0
        self.prev_error = 0.0

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.balance_pub = self.create_publisher(Float32, '/balance_error', 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.balance_control)  # 100Hz

        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.balance_active = True

        self.get_logger().info('Balance Controller initialized')

    def imu_callback(self, msg):
        # Extract orientation from IMU
        # Convert quaternion to roll/pitch
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z

        # Calculate roll and pitch (simplified)
        self.current_roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        self.current_pitch = np.arcsin(2*(w*y - z*x))

    def balance_control(self):
        if not self.balance_active:
            return

        # Target is upright (0 pitch, 0 roll)
        target_pitch = 0.0
        target_roll = 0.0

        # Calculate errors
        pitch_error = target_pitch - self.current_pitch
        roll_error = target_roll - self.current_roll

        # Use pitch error for balance control (simplified)
        error = pitch_error

        # Update integral and derivative terms
        self.error_integral += error * 0.01  # dt = 0.01s
        error_derivative = (error - self.prev_error) / 0.01 if 0.01 > 0 else 0

        # Calculate control output
        control_output = (self.kp * error +
                         self.ki * self.error_integral +
                         self.kd * error_derivative)

        # Publish balance error for monitoring
        error_msg = Float32()
        error_msg.data = float(abs(error))
        self.balance_pub.publish(error_msg)

        # Publish velocity command to correct balance
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0  # No forward movement for balance
        cmd_msg.angular.z = float(control_output)  # Adjust for balance

        self.cmd_pub.publish(cmd_msg)

        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    controller = BalanceController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 4: Field Deployment Simulation

### Step 5: Deployment Monitor Node

Create a system to monitor the deployed humanoid robot:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import BatteryState
import time

class DeploymentMonitor(Node):
    def __init__(self):
        super().__init__('deployment_monitor')

        # Subscriptions
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10
        )
        self.balance_error_sub = self.create_subscription(
            Float32, '/balance_error', self.balance_error_callback, 10
        )
        self.cpu_usage_sub = self.create_subscription(
            Float32, '/system/cpu_usage', self.cpu_callback, 10
        )

        # Publishers
        self.safety_pub = self.create_publisher(Bool, '/safety_status', 10)

        # Timer for safety checks
        self.safety_timer = self.create_timer(0.5, self.safety_check)

        self.battery_level = 1.0
        self.balance_error = 0.0
        self.cpu_usage = 0.0
        self.safety_status = True

        self.get_logger().info('Deployment Monitor initialized')

    def battery_callback(self, msg):
        self.battery_level = msg.percentage

    def balance_error_callback(self, msg):
        self.balance_error = msg.data

    def cpu_callback(self, msg):
        self.cpu_usage = msg.data

    def safety_check(self):
        # Check safety conditions
        safety_ok = True

        # Check battery (minimum 15%)
        if self.battery_level < 0.15:
            safety_ok = False
            self.get_logger().warn('Battery level critically low')

        # Check balance error (maximum 0.5 radians)
        if self.balance_error > 0.5:
            safety_ok = False
            self.get_logger().warn('Balance error exceeds safe threshold')

        # Check CPU usage (maximum 90%)
        if self.cpu_usage > 90.0:
            safety_ok = False
            self.get_logger().warn('CPU usage exceeds safe threshold')

        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = safety_ok
        self.safety_pub.publish(safety_msg)

        if not safety_ok:
            self.get_logger().error('SAFETY CONDITION VIOLATED - SHUTDOWN RECOMMENDED')

        self.safety_status = safety_ok

def main(args=None):
    rclpy.init(args=args)
    monitor = DeploymentMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Assessment

### Evaluation Criteria

1. **ROS 2 Integration (25 points)**: Proper implementation of custom messages and QoS settings
2. **Jetson Optimization (25 points)**: Effective performance monitoring and optimization
3. **Locomotion Control (25 points)**: Functional balance controller with appropriate response
4. **Field Deployment (25 points)**: Comprehensive safety monitoring and validation

### Submission Requirements

1. All source code files
2. A brief report (1-2 pages) describing your implementation approach
3. Performance metrics from your system
4. Any challenges encountered and how you addressed them

## Conclusion

This lab demonstrates the integration of advanced ROS 2 techniques, Jetson optimization, locomotion control, and field deployment considerations for humanoid robots. The skills developed here are essential for real-world humanoid robot deployment.