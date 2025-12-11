# Benchmarking Methodology for Inference Throughput and Latency

## Overview

This document outlines the comprehensive benchmarking methodology for evaluating inference throughput and latency in Isaac ROS perception pipelines running on NVIDIA Jetson platforms. The methodology ensures consistent, accurate, and reproducible performance measurements for GPU-accelerated computer vision applications.

## Benchmarking Objectives

### Primary Metrics
1. **Inference Throughput**: Frames per second (FPS) processing rate
2. **Inference Latency**: Time from input to output completion
3. **End-to-End Latency**: Total system response time including preprocessing and postprocessing
4. **Resource Utilization**: CPU, GPU, and memory usage during inference

### Secondary Metrics
1. **Power Consumption**: Energy efficiency under different loads
2. **Thermal Performance**: Temperature stability during sustained operation
3. **Memory Bandwidth**: Utilization of system and GPU memory
4. **Pipeline Efficiency**: Bottleneck identification and optimization opportunities

## Benchmarking Environment Setup

### Hardware Configuration
```bash
# Verify Jetson platform
sudo jetson_release

# Set power mode for consistent results
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks  # Lock clocks to maximum

# Check system status
nvidia-smi
cat /etc/nv_tegra_release
```

### Software Environment
```bash
# Source ROS 2 and Isaac ROS
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Verify Isaac ROS installation
dpkg -l | grep isaac-ros
```

## Benchmarking Tools and Frameworks

### 1. System Monitoring Tools

#### Jetson Stats for Real-time Monitoring
```bash
# Install jetson-stats
sudo pip3 install jetson-stats

# Monitor system in real-time
sudo jtop  # Provides CPU, GPU, memory, temperature, power consumption
```

#### Performance Analysis with Standard Tools
```bash
# CPU and memory monitoring
htop
iotop -ao  # I/O monitoring

# GPU monitoring
sudo tegrastats --interval 1000  # Every 1 second
```

### 2. Isaac ROS Benchmarking Tools

#### Isaac ROS Benchmark Package
```bash
# Install benchmark package
sudo apt install ros-humble-isaac-ros-benchmark

# Run basic benchmark
ros2 run isaac_ros_benchmark benchmark_node
```

#### Custom Benchmarking Nodes
```python
# benchmarking_tools.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from builtin_interfaces.msg import Time
import time
import numpy as np
from collections import deque

class PerformanceBenchmarkNode(Node):
    def __init__(self):
        super().__init__('performance_benchmark')

        # Parameters
        self.declare_parameter('window_size', 100)
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/input_image', self.image_callback, 10)

        # Publishers
        self.fps_pub = self.create_publisher(Float32, '/benchmark/fps', 10)
        self.latency_pub = self.create_publisher(Float32, '/benchmark/latency', 10)
        self.throughput_pub = self.create_publisher(Int32, '/benchmark/throughput', 10)

        # Performance tracking
        self.frame_times = deque(maxlen=self.window_size)
        self.processing_times = deque(maxlen=self.window_size)
        self.frame_counter = 0
        self.start_time = time.time()

        # Timer for periodic reporting
        self.report_timer = self.create_timer(1.0, self.report_metrics)

        self.get_logger().info('Performance benchmark node initialized')

    def image_callback(self, msg):
        current_time = time.time()
        self.frame_counter += 1

        # Track frame intervals
        if hasattr(self, 'last_frame_time'):
            frame_interval = current_time - self.last_frame_time
            self.frame_times.append(frame_interval)

        # Track processing time (from message timestamp to processing)
        msg_age = current_time - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.processing_times.append(msg_age)

        self.last_frame_time = current_time

    def report_metrics(self):
        if len(self.frame_times) > 0:
            # Calculate FPS
            avg_frame_time = np.mean(self.frame_times)
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0

            # Calculate latency
            avg_latency = np.mean(self.processing_times) if len(self.processing_times) > 0 else 0

            # Publish metrics
            fps_msg = Float32()
            fps_msg.data = float(fps)
            self.fps_pub.publish(fps_msg)

            latency_msg = Float32()
            latency_msg.data = float(avg_latency)
            self.latency_pub.publish(latency_msg)

            throughput_msg = Int32()
            throughput_msg.data = self.frame_counter
            self.throughput_pub.publish(throughput_msg)

            # Log metrics
            self.get_logger().info(
                f'Performance - FPS: {fps:.2f}, '
                f'Avg Latency: {avg_latency*1000:.2f}ms, '
                f'Throughput: {self.frame_counter}/s'
            )

            # Reset counter
            self.frame_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceBenchmarkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Performance benchmark stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Throughput Benchmarking

### 1. Frame Rate Measurement

#### Continuous Throughput Test
```python
# throughput_test.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import time
from collections import deque
import numpy as np

class ThroughputTest(Node):
    def __init__(self):
        super().__init__('throughput_test')

        # Parameters
        self.declare_parameter('test_duration', 60)  # seconds
        self.test_duration = self.get_parameter('test_duration').get_parameter_value().integer_value

        # Publishers
        self.image_pub = self.create_publisher(Image, '/synthetic_image', 10)

        # Variables for throughput calculation
        self.frame_count = 0
        self.start_time = time.time()
        self.test_start = time.time()

        # Timer for publishing synthetic images
        self.publish_timer = self.create_timer(0.033, self.publish_image)  # ~30 FPS

        self.get_logger().info(f'Starting throughput test for {self.test_duration} seconds')

    def publish_image(self):
        current_time = time.time()
        if current_time - self.test_start > self.test_duration:
            self.calculate_and_report_throughput()
            return

        # Create synthetic image
        image_msg = Image()
        image_msg.header = Header()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'synthetic_camera'
        image_msg.height = 480
        image_msg.width = 640
        image_msg.encoding = 'rgb8'
        image_msg.is_bigendian = False
        image_msg.step = 640 * 3
        image_msg.data = [128] * (640 * 480 * 3)  # Synthetic RGB data

        self.image_pub.publish(image_msg)
        self.frame_count += 1

    def calculate_and_report_throughput(self):
        elapsed_time = time.time() - self.test_start
        avg_throughput = self.frame_count / elapsed_time

        self.get_logger().info(f'Throughput Test Results:')
        self.get_logger().info(f'  Total Frames: {self.frame_count}')
        self.get_logger().info(f'  Elapsed Time: {elapsed_time:.2f}s')
        self.get_logger().info(f'  Average Throughput: {avg_throughput:.2f} FPS')

        # Cancel the timer
        self.publish_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    test = ThroughputTest()

    # Run for specified duration
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Burst Throughput Test
```python
# burst_throughput_test.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class BurstThroughputTest(Node):
    def __init__(self):
        super().__init__('burst_throughput_test')

        # Publishers
        self.image_pub = self.create_publisher(Image, '/burst_image', 10)

        self.get_logger().info('Starting burst throughput test')

    def run_burst_test(self, burst_size=100, burst_interval=1.0):
        """Send a burst of images and measure processing capability"""
        for burst in range(5):  # Run 5 bursts
            self.get_logger().info(f'Starting burst {burst + 1}/5')

            # Send burst of images
            start_time = time.time()
            for i in range(burst_size):
                image_msg = self.create_synthetic_image()
                self.image_pub.publish(image_msg)

            burst_time = time.time() - start_time
            self.get_logger().info(f'Burst {burst + 1}: Sent {burst_size} frames in {burst_time:.4f}s')

            # Wait before next burst
            time.sleep(burst_interval)

    def create_synthetic_image(self):
        from sensor_msgs.msg import Image
        from std_msgs.msg import Header

        image_msg = Image()
        image_msg.header = Header()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'synthetic_camera'
        image_msg.height = 480
        image_msg.width = 640
        image_msg.encoding = 'rgb8'
        image_msg.is_bigendian = False
        image_msg.step = 640 * 3
        image_msg.data = [128] * (640 * 480 * 3)
        return image_msg

def main(args=None):
    rclpy.init(args=args)
    test = BurstThroughputTest()

    test.run_burst_test(burst_size=100, burst_interval=2.0)

    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Latency Benchmarking

### 1. End-to-End Latency Measurement

#### Timestamp-Based Latency Tracking
```python
# latency_tracker.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import time
from collections import deque
import numpy as np

class LatencyTracker(Node):
    def __init__(self):
        super().__init__('latency_tracker')

        # Subscriptions
        self.input_sub = self.create_subscription(
            Image, '/input/image_raw', self.input_callback, 10)
        self.output_sub = self.create_subscription(
            Image, '/output/processed', self.output_callback, 10)

        # Publishers
        self.latency_pub = self.create_publisher(Float32, '/latency/ms', 10)

        # Storage for tracking timestamps
        self.input_timestamps = {}
        self.latency_measurements = deque(maxlen=1000)

        self.get_logger().info('Latency tracker initialized')

    def input_callback(self, msg):
        """Track input timestamp with unique ID"""
        timestamp_key = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        self.input_timestamps[timestamp_key] = time.time()

    def output_callback(self, msg):
        """Calculate and publish latency"""
        timestamp_key = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"

        if timestamp_key in self.input_timestamps:
            input_time = self.input_timestamps[timestamp_key]
            output_time = time.time()
            latency_ms = (output_time - input_time) * 1000

            # Store measurement
            self.latency_measurements.append(latency_ms)

            # Publish latency
            latency_msg = Float32()
            latency_msg.data = float(latency_ms)
            self.latency_pub.publish(latency_msg)

            # Log if latency is above threshold
            if latency_ms > 100:  # Alert if over 100ms
                self.get_logger().warn(f'High latency detected: {latency_ms:.2f}ms')

            # Clean up old timestamps
            if len(self.input_timestamps) > 1000:
                # Remove oldest entries
                oldest_keys = list(self.input_timestamps.keys())[:100]
                for key in oldest_keys:
                    del self.input_timestamps[key]

    def get_statistics(self):
        """Get latency statistics"""
        if len(self.latency_measurements) == 0:
            return None

        stats = {
            'mean': np.mean(self.latency_measurements),
            'median': np.median(self.latency_measurements),
            'std': np.std(self.latency_measurements),
            'min': np.min(self.latency_measurements),
            'max': np.max(self.latency_measurements),
            'p95': np.percentile(self.latency_measurements, 95),
            'p99': np.percentile(self.latency_measurements, 99),
            'count': len(self.latency_measurements)
        }
        return stats

def main(args=None):
    rclpy.init(args=args)
    tracker = LatencyTracker()

    # Print statistics periodically
    def print_stats():
        stats = tracker.get_statistics()
        if stats:
            tracker.get_logger().info(
                f'Latency Stats - Mean: {stats["mean"]:.2f}ms, '
                f'Max: {stats["max"]:.2f}ms, '
                f'95th percentile: {stats["p95"]:.2f}ms'
            )

    # Create timer to print stats every 5 seconds
    timer = tracker.create_timer(5.0, print_stats)

    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        tracker.get_logger().info('Latency tracker stopped')
        stats = tracker.get_statistics()
        if stats:
            print(f"\nFinal Latency Statistics:")
            print(f"  Mean: {stats['mean']:.2f}ms")
            print(f"  Median: {stats['median']:.2f}ms")
            print(f"  Std Dev: {stats['std']:.2f}ms")
            print(f"  Min: {stats['min']:.2f}ms")
            print(f"  Max: {stats['max']:.2f}ms")
            print(f"  95th percentile: {stats['p95']:.2f}ms")
            print(f"  99th percentile: {stats['p99']:.2f}ms")
            print(f"  Total measurements: {stats['count']}")
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Component-Level Latency Analysis
```python
# component_latency_analyzer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time
from collections import defaultdict

class ComponentLatencyAnalyzer(Node):
    def __init__(self):
        super().__init__('component_latency_analyzer')

        # Subscriptions for different pipeline stages
        self.raw_sub = self.create_subscription(
            Image, '/camera/image_raw', self.raw_callback, 10)
        self.rectified_sub = self.create_subscription(
            Image, '/camera/image_rect', self.rectified_callback, 10)
        self.processed_sub = self.create_subscription(
            Image, '/processed_output', self.processed_callback, 10)

        # Storage for timestamps
        self.timestamps = defaultdict(dict)
        self.latency_stats = defaultdict(list)

        self.get_logger().info('Component latency analyzer initialized')

    def raw_callback(self, msg):
        key = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        self.timestamps[key]['raw'] = time.time()

    def rectified_callback(self, msg):
        key = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        if key in self.timestamps:
            self.timestamps[key]['rectified'] = time.time()
            # Calculate raw->rectified latency
            if 'raw' in self.timestamps[key]:
                latency = (self.timestamps[key]['rectified'] -
                          self.timestamps[key]['raw']) * 1000
                self.latency_stats['raw_to_rectified'].append(latency)

    def processed_callback(self, msg):
        key = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        if key in self.timestamps:
            self.timestamps[key]['processed'] = time.time()
            # Calculate total end-to-end latency
            if 'raw' in self.timestamps[key]:
                total_latency = (self.timestamps[key]['processed'] -
                               self.timestamps[key]['raw']) * 1000
                self.latency_stats['end_to_end'].append(total_latency)

            # Calculate rectified->processed latency
            if 'rectified' in self.timestamps[key]:
                processing_latency = (self.timestamps[key]['processed'] -
                                    self.timestamps[key]['rectified']) * 1000
                self.latency_stats['processing'].append(processing_latency)

            # Clean up processed timestamps
            del self.timestamps[key]

    def print_component_stats(self):
        """Print statistics for each component"""
        for component, latencies in self.latency_stats.items():
            if len(latencies) > 0:
                avg_latency = sum(latencies) / len(latencies)
                min_latency = min(latencies)
                max_latency = max(latencies)

                self.get_logger().info(
                    f'{component} - Avg: {avg_latency:.2f}ms, '
                    f'Min: {min_latency:.2f}ms, Max: {max_latency:.2f}ms, '
                    f'Count: {len(latencies)}'
                )

def main(args=None):
    rclpy.init(args=args)
    analyzer = ComponentLatencyAnalyzer()

    # Print stats every 10 seconds
    timer = analyzer.create_timer(10.0, analyzer.print_component_stats)

    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info('Component latency analyzer stopped')
        analyzer.print_component_stats()
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## GPU Performance Benchmarking

### 1. TensorRT Inference Benchmarking
```python
# tensorrt_benchmark.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np
import time
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

class TensorRTBenchmark(Node):
    def __init__(self):
        super().__init__('tensorrt_benchmark')

        # Load TensorRT engine
        self.engine = self.load_engine('/path/to/model.plan')
        self.context = self.engine.create_execution_context()

        # Input/Output bindings
        self.input_binding = self.engine.get_binding_name(0)
        self.output_binding = self.engine.get_binding_name(1)

        # CUDA streams
        self.stream = cuda.Stream()

        # Publishers
        self.inference_time_pub = self.create_publisher(Float32, '/tensorrt/inference_time', 10)

        self.get_logger().info('TensorRT benchmark initialized')

    def load_engine(self, engine_path):
        """Load TensorRT engine from file"""
        with open(engine_path, 'rb') as f:
            engine_data = f.read()

        runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        engine = runtime.deserialize_cuda_engine(engine_data)
        return engine

    def run_inference(self, input_data):
        """Run inference on TensorRT engine"""
        # Allocate device memory
        input_size = trt.volume(self.engine.get_binding_shape(0)) * self.engine.max_batch_size * np.dtype(np.float32).itemsize
        output_size = trt.volume(self.engine.get_binding_shape(1)) * self.engine.max_batch_size * np.dtype(np.float32).itemsize

        d_input = cuda.mem_alloc(input_size)
        d_output = cuda.mem_alloc(output_size)

        # Transfer input data to device
        cuda.memcpy_htod_async(d_input, input_data, self.stream)

        # Run inference
        start_time = time.time()
        self.context.execute_async_v2(
            bindings=[int(d_input), int(d_output)],
            stream_handle=self.stream.handle
        )
        self.stream.synchronize()
        inference_time = (time.time() - start_time) * 1000  # Convert to ms

        # Transfer output data to host
        output_data = np.empty((self.engine.get_binding_shape(1)[1],), dtype=np.float32)
        cuda.memcpy_dtoh_async(output_data, d_output, self.stream)
        self.stream.synchronize()

        # Clean up
        d_input.free()
        d_output.free()

        # Publish inference time
        time_msg = Float32()
        time_msg.data = float(inference_time)
        self.inference_time_pub.publish(time_msg)

        return output_data, inference_time

def main(args=None):
    rclpy.init(args=args)
    benchmark = TensorRTBenchmark()

    # Example usage
    # Create dummy input data
    dummy_input = np.random.random((1, 3, 224, 224)).astype(np.float32)

    # Run inference and measure time
    try:
        output, inference_time = benchmark.run_inference(dummy_input)
        benchmark.get_logger().info(f'TensorRT Inference Time: {inference_time:.4f}ms')
    except Exception as e:
        benchmark.get_logger().error(f'TensorRT inference failed: {e}')

    benchmark.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## System Resource Monitoring

### 1. Comprehensive Resource Monitor
```python
# system_resource_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import psutil
import subprocess
import time

class SystemResourceMonitor(Node):
    def __init__(self):
        super().__init__('system_resource_monitor')

        # Publishers
        self.cpu_pub = self.create_publisher(Float32, '/system/cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float32, '/system/memory_usage', 10)
        self.gpu_pub = self.create_publisher(Float32, '/system/gpu_usage', 10)
        self.temperature_pub = self.create_publisher(Float32, '/system/temperature', 10)
        self.power_pub = self.create_publisher(Float32, '/system/power', 10)

        # Timer for monitoring
        self.monitor_timer = self.create_timer(0.5, self.monitor_system)  # 2 Hz

        self.get_logger().info('System resource monitor initialized')

    def monitor_system(self):
        # CPU usage
        cpu_percent = psutil.cpu_percent()
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_pub.publish(cpu_msg)

        # Memory usage
        memory_percent = psutil.virtual_memory().percent
        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_pub.publish(memory_msg)

        # GPU usage (NVIDIA Jetson specific)
        try:
            result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu',
                                   '--format=csv,noheader,nounits'],
                                   capture_output=True, text=True)
            if result.returncode == 0:
                gpu_util = float(result.stdout.strip())
                gpu_msg = Float32()
                gpu_msg.data = float(gpu_util)
                self.gpu_pub.publish(gpu_msg)
        except Exception as e:
            self.get_logger().warn(f'GPU monitoring failed: {e}')

        # Temperature
        try:
            temp_result = subprocess.run(['cat', '/sys/class/thermal/thermal_zone0/temp'],
                                       capture_output=True, text=True)
            if temp_result.returncode == 0:
                temperature = float(temp_result.stdout.strip()) / 1000.0  # Convert from millidegrees
                temp_msg = Float32()
                temp_msg.data = float(temperature)
                self.temperature_pub.publish(temp_msg)
        except Exception as e:
            self.get_logger().warn(f'Temperature monitoring failed: {e}')

        # Power consumption (if available on Jetson)
        try:
            power_result = subprocess.run(['sudo', 'trepctl', '-c'],
                                        capture_output=True, text=True)
            if power_result.returncode == 0:
                # Parse power data (format may vary by Jetson model)
                power_msg = Float32()
                power_msg.data = 5.0  # Placeholder - actual parsing needed
                self.power_pub.publish(power_msg)
        except Exception as e:
            self.get_logger().warn(f'Power monitoring failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    monitor = SystemResourceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('System resource monitor stopped')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Benchmarking Launch Files

### 1. Complete Benchmarking Setup
```python
# benchmarking_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Performance benchmark node
    perf_benchmark = Node(
        package='isaac_ros_benchmark',
        executable='performance_benchmark',
        name='performance_benchmark',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/input_image', '/camera/image_raw'),
        ]
    )

    # Latency tracker
    latency_tracker = Node(
        package='isaac_ros_benchmark',
        executable='latency_tracker',
        name='latency_tracker',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/input/image_raw', '/camera/image_raw'),
            ('/output/processed', '/perception/output'),
        ]
    )

    # System resource monitor
    system_monitor = Node(
        package='isaac_ros_benchmark',
        executable='system_resource_monitor',
        name='system_monitor',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='benchmark_rviz',
        arguments=['-d', 'path/to/benchmark_config.rviz']
    )

    return LaunchDescription([
        perf_benchmark,
        latency_tracker,
        system_monitor,
        rviz_node
    ])
```

## Statistical Analysis and Reporting

### 1. Benchmark Results Analysis
```python
# benchmark_analysis.py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import json
import os

class BenchmarkAnalyzer:
    def __init__(self, results_dir='benchmark_results'):
        self.results_dir = results_dir
        os.makedirs(results_dir, exist_ok=True)
        self.data = []

    def load_results(self, filename):
        """Load benchmark results from file"""
        filepath = os.path.join(self.results_dir, filename)
        with open(filepath, 'r') as f:
            return json.load(f)

    def analyze_performance(self, results):
        """Analyze performance metrics"""
        df = pd.DataFrame(results)

        analysis = {
            'throughput': {
                'mean': float(df['throughput'].mean()),
                'std': float(df['throughput'].std()),
                'min': float(df['throughput'].min()),
                'max': float(df['throughput'].max()),
                'median': float(df['throughput'].median()),
                'p95': float(df['throughput'].quantile(0.95)),
                'p99': float(df['throughput'].quantile(0.99))
            },
            'latency': {
                'mean': float(df['latency'].mean()),
                'std': float(df['latency'].std()),
                'min': float(df['latency'].min()),
                'max': float(df['latency'].max()),
                'median': float(df['latency'].median()),
                'p95': float(df['latency'].quantile(0.95)),
                'p99': float(df['latency'].quantile(0.99))
            }
        }

        return analysis

    def create_visualizations(self, results):
        """Create benchmark visualizations"""
        df = pd.DataFrame(results)

        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))

        # Throughput histogram
        axes[0, 0].hist(df['throughput'], bins=50, alpha=0.7)
        axes[0, 0].set_title('Throughput Distribution')
        axes[0, 0].set_xlabel('FPS')
        axes[0, 0].set_ylabel('Frequency')

        # Latency histogram
        axes[0, 1].hist(df['latency'], bins=50, alpha=0.7)
        axes[0, 1].set_title('Latency Distribution')
        axes[0, 1].set_xlabel('Latency (ms)')
        axes[0, 1].set_ylabel('Frequency')

        # Throughput over time
        axes[1, 0].plot(df.index, df['throughput'])
        axes[1, 0].set_title('Throughput Over Time')
        axes[1, 0].set_xlabel('Sample Index')
        axes[1, 0].set_ylabel('FPS')

        # Latency over time
        axes[1, 1].plot(df.index, df['latency'])
        axes[1, 1].set_title('Latency Over Time')
        axes[1, 1].set_xlabel('Sample Index')
        axes[1, 1].set_ylabel('Latency (ms)')

        plt.tight_layout()
        plt.savefig(os.path.join(self.results_dir, 'benchmark_analysis.png'))
        plt.close()

    def generate_report(self, analysis, filename='benchmark_report.md'):
        """Generate benchmark report"""
        report_content = f"""
# Isaac ROS Performance Benchmark Report

## Test Configuration
- Platform: NVIDIA Jetson Orin
- ROS 2: Humble Hawksbill
- Isaac ROS: Latest
- Test Duration: 60 seconds

## Performance Results

### Throughput Analysis
- **Mean Throughput**: {analysis['throughput']['mean']:.2f} FPS
- **Median Throughput**: {analysis['throughput']['median']:.2f} FPS
- **Std Deviation**: {analysis['throughput']['std']:.2f} FPS
- **Min**: {analysis['throughput']['min']:.2f} FPS
- **Max**: {analysis['throughput']['max']:.2f} FPS
- **95th Percentile**: {analysis['throughput']['p95']:.2f} FPS
- **99th Percentile**: {analysis['throughput']['p99']:.2f} FPS

### Latency Analysis
- **Mean Latency**: {analysis['latency']['mean']:.2f} ms
- **Median Latency**: {analysis['latency']['median']:.2f} ms
- **Std Deviation**: {analysis['latency']['std']:.2f} ms
- **Min**: {analysis['latency']['min']:.2f} ms
- **Max**: {analysis['latency']['max']:.2f} ms
- **95th Percentile**: {analysis['latency']['p95']:.2f} ms
- **99th Percentile**: {analysis['latency']['p99']:.2f} ms

## Performance Classification
- **Real-time**: < 33ms (30+ FPS)
- **Interactive**: 33-100ms (10-30 FPS)
- **Batch**: > 100ms (< 10 FPS)

## Recommendations
Based on the benchmark results, the system performance is suitable for [real-time/interactive/batch] applications.
        """

        with open(os.path.join(self.results_dir, filename), 'w') as f:
            f.write(report_content)

def main():
    analyzer = BenchmarkAnalyzer()

    # Example results data (in practice, this would come from actual benchmarking)
    sample_results = [
        {'throughput': 28.5, 'latency': 35.2, 'timestamp': 1634567890.123},
        {'throughput': 29.1, 'latency': 34.8, 'timestamp': 1634567891.123},
        {'throughput': 27.8, 'latency': 36.1, 'timestamp': 1634567892.123},
        # ... more results
    ]

    analysis = analyzer.analyze_performance(sample_results)
    analyzer.create_visualizations(sample_results)
    analyzer.generate_report(analysis)

    print("Benchmark analysis completed. Reports saved to benchmark_results/")

if __name__ == '__main__':
    main()
```

## Best Practices and Guidelines

### 1. Consistent Benchmarking Conditions
- Always run benchmarks with the same power mode and system settings
- Warm up the system before starting measurements
- Run multiple trials and average results
- Document all system configurations

### 2. Measurement Accuracy
- Use high-resolution timers for latency measurements
- Synchronize clocks between different components
- Account for system overhead in measurements
- Validate measurements against known baselines

### 3. Result Interpretation
- Consider statistical significance of results
- Account for variance in measurements
- Compare against performance requirements
- Document environmental conditions

This comprehensive benchmarking methodology provides the tools and procedures needed to accurately measure and evaluate inference throughput and latency in Isaac ROS perception pipelines on Jetson platforms.