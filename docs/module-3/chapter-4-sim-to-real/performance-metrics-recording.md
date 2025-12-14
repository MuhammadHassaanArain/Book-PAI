# Latency, Bandwidth, and Performance Metrics Recording

## Overview

This document provides comprehensive guidance for measuring and recording latency, bandwidth, and performance metrics during sim-to-real transfer of perception and navigation systems on Jetson hardware platforms.

## Latency Measurement

### 1. System Latency Components

#### End-to-End Latency
```python
# latency_measurement.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Header
import time
from collections import deque

class LatencyMeasurement(Node):
    def __init__(self):
        super().__init__('latency_measurement')

        # Subscriptions for different pipeline stages
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.processed_sub = self.create_subscription(
            Image, '/camera/image_processed', self.processed_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publishers for latency metrics
        self.end_to_end_pub = self.create_publisher(Float32, '/latency/end_to_end', 10)
        self.processing_pub = self.create_publisher(Float32, '/latency/processing', 10)
        self.controller_pub = self.create_publisher(Float32, '/latency/controller', 10)

        # Storage for timestamp tracking
        self.input_timestamps = {}
        self.processing_times = deque(maxlen=100)
        self.controller_times = deque(maxlen=100)

        self.get_logger().info('Latency Measurement Node initialized')

    def image_callback(self, msg):
        """Track input timestamp for latency calculation"""
        # Store timestamp with unique identifier
        timestamp_key = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        self.input_timestamps[timestamp_key] = time.time()

    def processed_callback(self, msg):
        """Calculate processing latency"""
        timestamp_key = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"

        if timestamp_key in self.input_timestamps:
            input_time = self.input_timestamps[timestamp_key]
            output_time = time.time()

            # Calculate latency in milliseconds
            latency_ms = (output_time - input_time) * 1000

            # Store for statistics
            self.processing_times.append(latency_ms)

            # Publish latency
            latency_msg = Float32()
            latency_msg.data = float(latency_ms)
            self.processing_pub.publish(latency_msg)

            # Log if latency exceeds threshold
            if latency_ms > 100:  # Alert if over 100ms
                self.get_logger().warn(f'High processing latency: {latency_ms:.2f}ms')

            # Clean up old timestamps
            if len(self.input_timestamps) > 1000:
                # Remove oldest entries
                oldest_keys = list(self.input_timestamps.keys())[:100]
                for key in oldest_keys:
                    del self.input_timestamps[key]

    def cmd_callback(self, msg):
        """Track controller latency"""
        # Controller latency measurement
        current_time = time.time()

        # For now, just track current time
        # In real implementation, compare with command generation time
        pass

    def get_latency_statistics(self):
        """Get latency statistics"""
        if len(self.processing_times) == 0:
            return None

        import numpy as np
        stats = {
            'mean': float(np.mean(self.processing_times)),
            'median': float(np.median(self.processing_times)),
            'std': float(np.std(self.processing_times)),
            'min': float(np.min(self.processing_times)),
            'max': float(np.max(self.processing_times)),
            'p95': float(np.percentile(self.processing_times, 95)),
            'p99': float(np.percentile(self.processing_times, 99)),
            'count': len(self.processing_times)
        }
        return stats

def main(args=None):
    rclpy.init(args=args)
    latency_node = LatencyMeasurement()

    # Print statistics periodically
    def print_stats():
        stats = latency_node.get_latency_statistics()
        if stats:
            latency_node.get_logger().info(
                f'Latency Stats - Mean: {stats["mean"]:.2f}ms, '
                f'Max: {stats["max"]:.2f}ms, '
                f'95th percentile: {stats["p95"]:.2f}ms'
            )

    # Create timer to print stats every 5 seconds
    timer = latency_node.create_timer(5.0, print_stats)

    try:
        rclpy.spin(latency_node)
    except KeyboardInterrupt:
        latency_node.get_logger().info('Latency measurement stopped')
        stats = latency_node.get_latency_statistics()
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
        latency_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Component-Level Latency Analysis

#### Pipeline Latency Breakdown
```python
# component_latency_analyzer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32
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
        self.feature_sub = self.create_subscription(
            Image, '/features', self.feature_callback, 10)
        self.detection_sub = self.create_subscription(
            Image, '/detections', self.detection_callback, 10)

        # Storage for timestamps
        self.timestamps = defaultdict(dict)
        self.latency_stats = defaultdict(list)

        self.get_logger().info('Component Latency Analyzer initialized')

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

    def feature_callback(self, msg):
        key = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        if key in self.timestamps:
            self.timestamps[key]['features'] = time.time()
            # Calculate rectified->features latency
            if 'rectified' in self.timestamps[key]:
                latency = (self.timestamps[key]['features'] -
                          self.timestamps[key]['rectified']) * 1000
                self.latency_stats['rectify_to_features'].append(latency)

    def detection_callback(self, msg):
        key = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        if key in self.timestamps:
            self.timestamps[key]['detections'] = time.time()
            # Calculate total end-to-end latency
            if 'raw' in self.timestamps[key]:
                total_latency = (self.timestamps[key]['detections'] -
                               self.timestamps[key]['raw']) * 1000
                self.latency_stats['end_to_end'].append(total_latency)

            # Calculate features->detection latency
            if 'features' in self.timestamps[key]:
                detection_latency = (self.timestamps[key]['detections'] -
                                    self.timestamps[key]['features']) * 1000
                self.latency_stats['feature_to_detection'].append(detection_latency)

            # Clean up processed timestamps
            del self.timestamps[key]

    def print_component_stats(self):
        """Print statistics for each component"""
        import numpy as np
        for component, latencies in self.latency_stats.items():
            if len(latencies) > 0:
                avg_latency = np.mean(latencies)
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

## Bandwidth Measurement

### 1. Network and Data Bandwidth

#### Bandwidth Monitoring
```python
# bandwidth_monitor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from std_msgs.msg import Float32
import time
import psutil

class BandwidthMonitor(Node):
    def __init__(self):
        super().__init__('bandwidth_monitor')

        # Subscriptions for different data types
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publishers for bandwidth metrics
        self.image_bw_pub = self.create_publisher(Float32, '/bandwidth/image', 10)
        self.pointcloud_bw_pub = self.create_publisher(Float32, '/bandwidth/pointcloud', 10)
        self.system_net_pub = self.create_publisher(Float32, '/bandwidth/network', 10)

        # Data tracking
        self.image_data_size = 0
        self.pointcloud_data_size = 0
        self.scan_data_size = 0
        self.start_time = time.time()

        # System monitoring
        self.prev_net_io = psutil.net_io_counters()

        self.get_logger().info('Bandwidth Monitor initialized')

    def image_callback(self, msg):
        """Track image data size"""
        # Calculate approximate data size (bytes)
        data_size_bytes = len(msg.data)
        self.image_data_size += data_size_bytes

    def pointcloud_callback(self, msg):
        """Track point cloud data size"""
        # Calculate data size based on point count
        data_size_bytes = len(msg.data)
        self.pointcloud_data_size += data_size_bytes

    def scan_callback(self, msg):
        """Track scan data size"""
        # Calculate data size
        data_size_bytes = len(msg.ranges) * 4  # Assuming 4 bytes per range
        self.scan_data_size += data_size_bytes

    def calculate_bandwidth(self):
        """Calculate bandwidth for all data streams"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if elapsed_time > 0:
            # Calculate image bandwidth (MB/s)
            image_bw_mb = (self.image_data_size / (1024 * 1024)) / elapsed_time
            image_bw_msg = Float32()
            image_bw_msg.data = float(image_bw_mb)
            self.image_bw_pub.publish(image_bw_msg)

            # Calculate point cloud bandwidth (MB/s)
            pc_bw_mb = (self.pointcloud_data_size / (1024 * 1024)) / elapsed_time
            pc_bw_msg = Float32()
            pc_bw_msg.data = float(pc_bw_mb)
            self.pointcloud_bw_pub.publish(pc_bw_msg)

            # Calculate system network bandwidth
            current_net_io = psutil.net_io_counters()
            net_sent = current_net_io.bytes_sent - self.prev_net_io.bytes_sent
            net_recv = current_net_io.bytes_recv - self.prev_net_io.bytes_recv
            net_total = net_sent + net_recv
            net_bw_mb = (net_total / (1024 * 1024)) / elapsed_time

            net_bw_msg = Float32()
            net_bw_msg.data = float(net_bw_mb)
            self.system_net_pub.publish(net_bw_msg)

            # Reset counters periodically
            if elapsed_time > 10:  # Reset every 10 seconds
                self.image_data_size = 0
                self.pointcloud_data_size = 0
                self.scan_data_size = 0
                self.start_time = current_time
                self.prev_net_io = current_net_io

            self.get_logger().info(
                f'Bandwidth - Image: {image_bw_mb:.2f} MB/s, '
                f'PointCloud: {pc_bw_mb:.2f} MB/s, '
                f'Network: {net_bw_mb:.2f} MB/s'
            )

def main(args=None):
    rclpy.init(args=args)
    monitor = BandwidthMonitor()

    # Calculate bandwidth every second
    timer = monitor.create_timer(1.0, monitor.calculate_bandwidth)

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Bandwidth monitor stopped')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. GPU Memory and Compute Bandwidth

#### GPU Resource Monitoring
```python
# gpu_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import subprocess
import time

class GPUMonitor(Node):
    def __init__(self):
        super().__init__('gpu_monitor')

        # Publishers for GPU metrics
        self.gpu_util_pub = self.create_publisher(Float32, '/gpu/utilization', 10)
        self.gpu_mem_pub = self.create_publisher(Float32, '/gpu/memory', 10)
        self.gpu_temp_pub = self.create_publisher(Float32, '/gpu/temperature', 10)

        self.get_logger().info('GPU Monitor initialized')

    def get_gpu_info(self):
        """Get GPU information using nvidia-smi"""
        try:
            # Get GPU utilization
            result = subprocess.run([
                'nvidia-smi',
                '--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu',
                '--format=csv,noheader,nounits'
            ], capture_output=True, text=True)

            if result.returncode == 0:
                gpu_info = result.stdout.strip().split(',')

                if len(gpu_info) >= 4:
                    gpu_util = float(gpu_info[0].strip())
                    mem_used = float(gpu_info[1].strip())
                    mem_total = float(gpu_info[2].strip())
                    gpu_temp = float(gpu_info[3].strip())

                    # Publish metrics
                    util_msg = Float32()
                    util_msg.data = float(gpu_util)
                    self.gpu_util_pub.publish(util_msg)

                    mem_msg = Float32()
                    mem_msg.data = float(mem_used / mem_total) if mem_total > 0 else 0.0
                    self.gpu_mem_pub.publish(mem_msg)

                    temp_msg = Float32()
                    temp_msg.data = float(gpu_temp)
                    self.gpu_temp_pub.publish(temp_msg)

                    self.get_logger().info(
                        f'GPU - Util: {gpu_util}%, '
                        f'Mem: {mem_used}/{mem_total}MB ({mem_used/mem_total*100:.1f}%), '
                        f'Temp: {gpu_temp}°C'
                    )
            else:
                self.get_logger().warn(f'nvidia-smi command failed: {result.stderr}')

        except Exception as e:
            self.get_logger().warn(f'GPU monitoring error: {e}')

def main(args=None):
    rclpy.init(args=args)
    monitor = GPUMonitor()

    # Monitor GPU every 2 seconds
    timer = monitor.create_timer(2.0, monitor.get_gpu_info)

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('GPU monitor stopped')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Metrics Recording

### 1. Comprehensive Performance Monitor

#### Performance Data Recorder
```python
# performance_recorder.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
import csv
import os
from datetime import datetime
import psutil
import subprocess

class PerformanceRecorder(Node):
    def __init__(self):
        super().__init__('performance_recorder')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # Publishers for metrics
        self.fps_pub = self.create_publisher(Float32, '/performance/fps', 10)
        self.cpu_pub = self.create_publisher(Float32, '/performance/cpu', 10)
        self.memory_pub = self.create_publisher(Float32, '/performance/memory', 10)

        # Performance tracking
        self.frame_times = []
        self.scan_times = []
        self.start_time = time.time()

        # Create data directory
        self.data_dir = f'/home/jetson/performance_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        os.makedirs(self.data_dir, exist_ok=True)

        # Open CSV files for logging
        self.metrics_file = open(f'{self.data_dir}/performance_metrics.csv', 'w', newline='')
        self.metrics_writer = csv.writer(self.metrics_file)
        self.metrics_writer.writerow([
            'timestamp', 'fps', 'cpu_percent', 'memory_percent',
            'gpu_util', 'gpu_memory', 'gpu_temp', 'latency_ms'
        ])

        # Timer for periodic recording
        self.record_timer = self.create_timer(1.0, self.record_performance)

        self.get_logger().info(f'Performance recorder initialized, saving to {self.data_dir}')

    def image_callback(self, msg):
        """Track frame timing"""
        current_time = time.time()
        if hasattr(self, 'last_frame_time'):
            frame_time = current_time - self.last_frame_time
            self.frame_times.append(frame_time)
        self.last_frame_time = current_time

    def scan_callback(self, msg):
        """Track scan timing"""
        current_time = time.time()
        if hasattr(self, 'last_scan_time'):
            scan_time = current_time - self.last_scan_time
            self.scan_times.append(scan_time)
        self.last_scan_time = current_time

    def cmd_callback(self, msg):
        """Track command processing"""
        pass

    def get_gpu_info(self):
        """Get GPU information"""
        try:
            result = subprocess.run([
                'nvidia-smi',
                '--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu',
                '--format=csv,noheader,nounits'
            ], capture_output=True, text=True)

            if result.returncode == 0:
                gpu_info = result.stdout.strip().split(',')
                if len(gpu_info) >= 4:
                    return {
                        'utilization': float(gpu_info[0].strip()),
                        'memory_used': float(gpu_info[1].strip()),
                        'memory_total': float(gpu_info[2].strip()),
                        'temperature': float(gpu_info[3].strip())
                    }
        except:
            pass
        return {'utilization': 0, 'memory_used': 0, 'memory_total': 0, 'temperature': 0}

    def record_performance(self):
        """Record and publish performance metrics"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Calculate FPS
        if self.frame_times:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times) if self.frame_times else 1.0
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0
            self.frame_times = []  # Reset for next interval
        else:
            fps = 0

        # Get system metrics
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent

        # Get GPU metrics
        gpu_info = self.get_gpu_info()

        # Calculate average latency (simplified)
        avg_latency = 0  # This would come from latency measurements

        # Publish metrics
        fps_msg = Float32()
        fps_msg.data = float(fps)
        self.fps_pub.publish(fps_msg)

        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_pub.publish(cpu_msg)

        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_pub.publish(memory_msg)

        # Write to CSV
        self.metrics_writer.writerow([
            current_time,
            fps,
            cpu_percent,
            memory_percent,
            gpu_info['utilization'],
            gpu_info['memory_used'] / max(gpu_info['memory_total'], 1),
            gpu_info['temperature'],
            avg_latency
        ])

        # Log metrics
        self.get_logger().info(
            f'Performance - FPS: {fps:.2f}, CPU: {cpu_percent:.1f}%, '
            f'Mem: {memory_percent:.1f}%, GPU: {gpu_info["utilization"]:.1f}%'
        )

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'metrics_file'):
            self.metrics_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = PerformanceRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Performance recorder stopped')
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Metrics Dashboard

#### Performance Dashboard
```python
# metrics_dashboard.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class MetricsDashboard(Node):
    def __init__(self):
        super().__init__('metrics_dashboard')

        # Subscriptions for all metrics
        self.fps_sub = self.create_subscription(Float32, '/performance/fps', self.fps_callback, 10)
        self.latency_sub = self.create_subscription(Float32, '/latency/processing', self.latency_callback, 10)
        self.bandwidth_sub = self.create_subscription(Float32, '/bandwidth/image', self.bandwidth_callback, 10)
        self.cpu_sub = self.create_subscription(Float32, '/performance/cpu', self.cpu_callback, 10)
        self.gpu_sub = self.create_subscription(Float32, '/gpu/utilization', self.gpu_callback, 10)

        # Performance tracking
        self.metrics_history = {
            'fps': [],
            'latency': [],
            'bandwidth': [],
            'cpu': [],
            'gpu': []
        }

        # Thresholds for alerts
        self.thresholds = {
            'fps': (15, 30),      # Warning, Critical
            'latency': (100, 200), # Warning, Critical (ms)
            'bandwidth': (50, 100), # Warning, Critical (MB/s)
            'cpu': (80, 95),      # Warning, Critical (%)
            'gpu': (85, 95)       # Warning, Critical (%)
        }

        self.get_logger().info('Metrics Dashboard initialized')

    def fps_callback(self, msg):
        self.metrics_history['fps'].append((time.time(), msg.data))
        self.check_threshold('fps', msg.data)

    def latency_callback(self, msg):
        self.metrics_history['latency'].append((time.time(), msg.data))
        self.check_threshold('latency', msg.data)

    def bandwidth_callback(self, msg):
        self.metrics_history['bandwidth'].append((time.time(), msg.data))
        self.check_threshold('bandwidth', msg.data)

    def cpu_callback(self, msg):
        self.metrics_history['cpu'].append((time.time(), msg.data))
        self.check_threshold('cpu', msg.data)

    def gpu_callback(self, msg):
        self.metrics_history['gpu'].append((time.time(), msg.data))
        self.check_threshold('gpu', msg.data)

    def check_threshold(self, metric_name, value):
        """Check if metric exceeds thresholds"""
        if metric_name in self.thresholds:
            warning, critical = self.thresholds[metric_name]

            if value >= critical:
                self.get_logger().fatal(f'{metric_name.upper()} CRITICAL: {value:.2f} (threshold: {critical})')
            elif value >= warning:
                self.get_logger().warn(f'{metric_name.upper()} WARNING: {value:.2f} (threshold: {warning})')

    def get_current_metrics(self):
        """Get current metric values"""
        current_metrics = {}
        for metric, history in self.metrics_history.items():
            if history:
                current_metrics[metric] = history[-1][1]  # Most recent value
            else:
                current_metrics[metric] = 0

        return current_metrics

    def print_dashboard(self):
        """Print current dashboard"""
        metrics = self.get_current_metrics()

        print("\n" + "="*50)
        print("PERFORMANCE DASHBOARD")
        print("="*50)
        for metric, value in metrics.items():
            print(f"{metric.upper()}: {value:.2f}")
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    dashboard = MetricsDashboard()

    # Print dashboard every 5 seconds
    timer = dashboard.create_timer(5.0, dashboard.print_dashboard)

    try:
        rclpy.spin(dashboard)
    except KeyboardInterrupt:
        dashboard.get_logger().info('Metrics dashboard stopped')
    finally:
        dashboard.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This guide provides comprehensive procedures for measuring and recording all critical performance metrics during sim-to-real transfer.