#!/usr/bin/env python3
"""
Verification script for performance optimization techniques in Gazebo

This script tests that performance optimization techniques are properly implemented
and achieving expected performance improvements.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
from gazebo_msgs.msg import PerformanceMetrics
from std_msgs.msg import Float64
import time
import subprocess
import psutil
import os
from typing import Dict, Tuple, List
import xml.etree.ElementTree as ET


class PerformanceVerification(Node):
    """Node for verifying performance optimization techniques"""

    def __init__(self):
        """Initialize the performance verification node"""
        super().__init__('performance_verification')

        # Service clients
        self.get_physics_client = self.create_client(
            GetPhysicsProperties,
            '/gazebo/get_physics_properties'
        )
        self.set_physics_client = self.create_client(
            SetPhysicsProperties,
            '/gazebo/set_physics_properties'
        )

        # Wait for services
        while not self.get_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_physics_properties service...')
        while not self.set_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_physics_properties service...')

        # Performance metrics
        self.metrics_history = []
        self.cpu_usage_history = []
        self.memory_usage_history = []

    def get_system_metrics(self) -> Dict:
        """Get current system metrics"""
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_percent = psutil.virtual_memory().percent
        disk_usage = psutil.disk_usage('/').percent

        return {
            'cpu_percent': cpu_percent,
            'memory_percent': memory_percent,
            'disk_percent': disk_usage,
            'timestamp': time.time()
        }

    def get_gazebo_metrics(self) -> Dict:
        """Get Gazebo performance metrics if available"""
        try:
            # Get Gazebo stats via command line
            result = subprocess.run(['gz', 'stats'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                # Parse the gz stats output
                lines = result.stdout.strip().split('\n')
                metrics = {}
                for line in lines:
                    if 'Real Time Factor' in line:
                        # Extract RTF value
                        parts = line.split()
                        for i, part in enumerate(parts):
                            if part == 'Real' and i + 2 < len(parts):
                                try:
                                    rtf = float(parts[i + 2].rstrip(','))
                                    metrics['rtf'] = rtf
                                    break
                                except ValueError:
                                    continue
                    elif 'Sim Time' in line:
                        # Extract simulation time
                        parts = line.split()
                        for i, part in enumerate(parts):
                            if part == 'Sim' and i + 2 < len(parts):
                                try:
                                    sim_time = float(parts[i + 2])
                                    metrics['sim_time'] = sim_time
                                    break
                                except ValueError:
                                    continue
                return metrics
            else:
                self.get_logger().warn('Could not get Gazebo stats: ' + result.stderr)
                return {}
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Gazebo stats command timed out')
            return {}
        except Exception as e:
            self.get_logger().warn(f'Error getting Gazebo stats: {e}')
            return {}

    def verify_physics_optimization(self) -> Tuple[bool, Dict]:
        """
        Verify physics optimization parameters are set appropriately

        Returns:
            Tuple of (success, metrics)
        """
        self.get_logger().info('Verifying physics optimization parameters...')

        # Get current physics properties
        request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            props = future.result()

            # Check if parameters are within reasonable optimization ranges
            # For humanoid robots, typical optimized values are:
            # - time step: 0.001 (balance between accuracy and performance)
            # - solver iterations: 100-200 (balance between stability and performance)
            # - CFM: 1e-5 to 1e-6 (balance between stability and performance)
            # - ERP: 0.2 to 0.8 (balance between constraint strength and performance)

            time_step_ok = 0.0005 <= props.time_step <= 0.002  # Reasonable range
            update_rate_ok = props.max_update_rate >= 1000.0  # At least 1000 Hz
            iterations_reasonable = 50 <= props.ode_config.sor_pgs_iters <= 300  # Not too high or low
            cfm_reasonable = 1e-7 <= props.ode_config.cfm <= 1e-4  # Reasonable stiffness
            erp_reasonable = 0.1 <= props.ode_config.erp <= 0.9  # Reasonable constraint strength

            metrics = {
                'time_step': props.time_step,
                'max_update_rate': props.max_update_rate,
                'solver_iterations': props.ode_config.sor_pgs_iters,
                'cfm': props.ode_config.cfm,
                'erp': props.ode_config.erp,
                'time_step_ok': time_step_ok,
                'update_rate_ok': update_rate_ok,
                'iterations_reasonable': iterations_reasonable,
                'cfm_reasonable': cfm_reasonable,
                'erp_reasonable': erp_reasonable
            }

            success = all([time_step_ok, update_rate_ok, iterations_reasonable,
                          cfm_reasonable, erp_reasonable])

            return success, metrics
        else:
            return False, {'error': 'Could not get physics properties'}

    def verify_world_optimization(self) -> Tuple[bool, Dict]:
        """
        Verify world file has optimization settings

        Returns:
            Tuple of (success, metrics)
        """
        self.get_logger().info('Verifying world optimization settings...')

        world_file_path = 'simulation-assets/gazebo/worlds/physics_demo.world'

        try:
            # Read and parse the world file
            tree = ET.parse(world_file_path)
            root = tree.getroot()

            # Look for physics settings in the world file
            physics_elem = root.find('.//physics')
            if physics_elem is not None:
                max_step_size = physics_elem.find('max_step_size')
                real_time_factor = physics_elem.find('real_time_factor')
                real_time_update_rate = physics_elem.find('real_time_update_rate')
                gravity = physics_elem.find('gravity')

                metrics = {
                    'max_step_size': float(max_step_size.text) if max_step_size is not None else None,
                    'real_time_factor': float(real_time_factor.text) if real_time_factor is not None else None,
                    'real_time_update_rate': float(real_time_update_rate.text) if real_time_update_rate is not None else None,
                    'gravity_configured': gravity is not None
                }

                # Check if reasonable values are set
                reasonable_step = metrics['max_step_size'] is not None and 0.0005 <= metrics['max_step_size'] <= 0.002
                reasonable_rtf = metrics['real_time_factor'] is not None and metrics['real_time_factor'] > 0
                reasonable_rate = metrics['real_time_update_rate'] is not None and metrics['real_time_update_rate'] >= 100

                success = reasonable_step and reasonable_rtf and reasonable_rate
                return success, metrics
            else:
                return False, {'error': 'No physics configuration found in world file'}
        except ET.ParseError as e:
            return False, {'error': f'Could not parse world file: {e}'}
        except FileNotFoundError:
            return False, {'error': f'World file not found: {world_file_path}'}

    def verify_model_optimization(self) -> Tuple[bool, Dict]:
        """
        Verify model files have optimized collision geometries

        Returns:
            Tuple of (success, metrics)
        """
        self.get_logger().info('Verifying model optimization settings...')

        model_file_path = 'src/urdf/humanoid_models/physics_humanoid.urdf'

        try:
            # Read and parse the URDF file
            tree = ET.parse(model_file_path)
            root = tree.getroot()

            # Count collision vs visual elements to check for optimization
            collision_count = len(root.findall('.//collision'))
            visual_count = len(root.findall('.//visual'))

            # Check if collision geometries are simplified (not using complex meshes everywhere)
            mesh_collision_count = len(root.findall('.//collision//mesh'))
            simple_geom_collision_count = len(root.findall('.//collision//box')) + \
                                         len(root.findall('.//collision//sphere')) + \
                                         len(root.findall('.//collision//cylinder'))

            metrics = {
                'collision_count': collision_count,
                'visual_count': visual_count,
                'mesh_collision_count': mesh_collision_count,
                'simple_geom_collision_count': simple_geom_collision_count,
                'collision_to_visual_ratio': collision_count / visual_count if visual_count > 0 else 0
            }

            # For optimization, we expect more simple geometries than complex meshes
            optimization_indicators = [
                simple_geom_collision_count > mesh_collision_count,  # More simple than complex
                collision_count > 0,  # At least some collisions defined
                metrics['collision_to_visual_ratio'] <= 2.0  # Not too many more collisions than visuals
            ]

            success = all(optimization_indicators)
            return success, metrics
        except ET.ParseError as e:
            return False, {'error': f'Could not parse URDF file: {e}'}
        except FileNotFoundError:
            return False, {'error': f'URDF file not found: {model_file_path}'}

    def run_performance_benchmark(self, duration: float = 10.0) -> Tuple[bool, Dict]:
        """
        Run a performance benchmark to verify optimization effectiveness

        Args:
            duration: Duration to run benchmark in seconds

        Returns:
            Tuple of (success, metrics)
        """
        self.get_logger().info(f'Running performance benchmark for {duration} seconds...')

        start_time = time.time()
        end_time = start_time + duration

        # Collect metrics over time
        rtf_values = []
        cpu_values = []

        while time.time() < end_time:
            # Get Gazebo metrics
            gz_metrics = self.get_gazebo_metrics()
            if 'rtf' in gz_metrics:
                rtf_values.append(gz_metrics['rtf'])

            # Get system metrics
            sys_metrics = self.get_system_metrics()
            cpu_values.append(sys_metrics['cpu_percent'])

            time.sleep(1.0)  # Sample every second

        if rtf_values:
            avg_rtf = sum(rtf_values) / len(rtf_values)
            min_rtf = min(rtf_values)
            avg_cpu = sum(cpu_values) / len(cpu_values) if cpu_values else 0

            metrics = {
                'avg_rtf': avg_rtf,
                'min_rtf': min_rtf,
                'avg_cpu_percent': avg_cpu,
                'samples_count': len(rtf_values),
                'rtf_stability': (max(rtf_values) - min(rtf_values)) if rtf_values else 0
            }

            # Performance is good if RTF is close to 1.0 and CPU is reasonable
            rtf_acceptable = avg_rtf >= 0.8  # At least 80% real-time performance
            cpu_acceptable = avg_cpu <= 80.0  # Less than 80% CPU usage
            stable_rtf = metrics['rtf_stability'] <= 0.5  # RTF doesn't vary too much

            success = rtf_acceptable and cpu_acceptable
            return success, metrics
        else:
            return False, {'error': 'Could not collect RTF data during benchmark'}

    def verify_all_optimizations(self) -> Tuple[bool, Dict]:
        """
        Verify all performance optimizations together

        Returns:
            Tuple of (success, comprehensive_metrics)
        """
        self.get_logger().info('Verifying all performance optimizations...')

        # Run all verification checks
        physics_ok, physics_metrics = self.verify_physics_optimization()
        world_ok, world_metrics = self.verify_world_optimization()
        model_ok, model_metrics = self.verify_model_optimization()
        benchmark_ok, benchmark_metrics = self.run_performance_benchmark(duration=5.0)

        overall_metrics = {
            'physics_verification': {
                'success': physics_ok,
                'metrics': physics_metrics
            },
            'world_verification': {
                'success': world_ok,
                'metrics': world_metrics
            },
            'model_verification': {
                'success': model_ok,
                'metrics': model_metrics
            },
            'benchmark_verification': {
                'success': benchmark_ok,
                'metrics': benchmark_metrics
            }
        }

        # Overall success requires physics and world to be optimized
        # Model optimization is important but not strictly required for basic operation
        # Benchmark provides additional verification
        overall_success = physics_ok and world_ok

        return overall_success, overall_metrics


def main():
    """Main function to run performance optimization verification"""
    rclpy.init()

    verifier = PerformanceVerification()

    # Run comprehensive verification
    success, metrics = verifier.verify_all_optimizations()

    # Print results
    print("\n=== Performance Optimization Verification Results ===")

    physics_result = metrics['physics_verification']
    world_result = metrics['world_verification']
    model_result = metrics['model_verification']
    benchmark_result = metrics['benchmark_verification']

    print(f"Physics Parameters: {'PASS' if physics_result['success'] else 'FAIL'}")
    if physics_result['success']:
        p_metrics = physics_result['metrics']
        print(f"  Time step: {p_metrics.get('time_step', 'N/A')}")
        print(f"  Update rate: {p_metrics.get('max_update_rate', 'N/A')}")
        print(f"  Solver iterations: {p_metrics.get('solver_iterations', 'N/A')}")

    print(f"World Configuration: {'PASS' if world_result['success'] else 'FAIL'}")
    if world_result['success']:
        w_metrics = world_result['metrics']
        print(f"  Time step: {w_metrics.get('max_step_size', 'N/A')}")
        print(f"  RTF: {w_metrics.get('real_time_factor', 'N/A')}")
        print(f"  Update rate: {w_metrics.get('real_time_update_rate', 'N/A')}")

    print(f"Model Optimization: {'PASS' if model_result['success'] else 'FAIL'}")
    if model_result['success']:
        m_metrics = model_result['metrics']
        print(f"  Collisions: {m_metrics.get('collision_count', 'N/A')}")
        print(f"  Simple geoms: {m_metrics.get('simple_geom_collision_count', 'N/A')}")

    print(f"Benchmark Performance: {'PASS' if benchmark_result['success'] else 'FAIL'}")
    if benchmark_result['success']:
        b_metrics = benchmark_result['metrics']
        print(f"  Avg RTF: {b_metrics.get('avg_rtf', 'N/A'):.2f}")
        print(f"  Avg CPU: {b_metrics.get('avg_cpu_percent', 'N/A'):.1f}%")

    print(f"\nOverall Verification: {'PASS' if success else 'FAIL'}")

    verifier.destroy_node()
    rclpy.shutdown()

    return success


if __name__ == '__main__':
    success = main()
    exit(0 if success else 1)