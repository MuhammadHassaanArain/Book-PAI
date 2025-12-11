# Exporting Trained Perception Models from Simulation

## Overview

This guide provides the process for exporting trained perception models from Isaac Sim to deploy on real hardware. The process involves optimizing models for edge deployment using TensorRT and preparing them for Jetson platforms.

## Model Optimization Process

### 1. ONNX Export

#### Export Model from Isaac Sim
```bash
# Navigate to Isaac Sim workspace
cd ~/.local/share/ov/pkg/isaac_sim-*

# Export trained model in ONNX format
python3 -c "
import torch
import onnx

# Load your trained model
model = torch.load('path/to/trained_model.pth')
model.eval()

# Create dummy input matching your model's expected input
dummy_input = torch.randn(1, 3, 224, 224)  # Adjust dimensions as needed

# Export to ONNX
torch.onnx.export(
    model,
    dummy_input,
    'model.onnx',
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
print('Model exported to ONNX format')
"
```

### 2. TensorRT Optimization

#### Optimize for Jetson Platform
```bash
# Install TensorRT tools
sudo apt install tensorrt tensorrt-dev python3-libnvinfer-dev

# Optimize model using trtexec
trtexec \
    --onnx=model.onnx \
    --saveEngine=model.plan \
    --fp16 \
    --workspace=1024 \
    --device=0 \
    --explicitBatch

# Verify optimization
trtexec --loadEngine=model.plan --verbose
```

### 3. Isaac ROS Integration

#### Prepare for Isaac ROS Pipeline
```python
# model_exporter.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import subprocess

class ModelExporter(Node):
    def __init__(self):
        super().__init__('model_exporter')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('output_path', '/models/')
        self.declare_parameter('precision', 'fp16')

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.precision = self.get_parameter('precision').get_parameter_value().string_value

        self.get_logger().info('Model exporter initialized')

    def export_model(self):
        """Export and optimize model for deployment"""
        if not self.model_path or not os.path.exists(self.model_path):
            self.get_logger().error(f'Model path does not exist: {self.model_path}')
            return False

        # Create output directory
        os.makedirs(self.output_path, exist_ok=True)

        # Get base filename
        base_name = os.path.splitext(os.path.basename(self.model_path))[0]

        # Step 1: Convert to ONNX if needed
        onnx_path = os.path.join(self.output_path, f'{base_name}.onnx')
        if self.model_path.endswith('.pth'):
            self.convert_to_onnx(self.model_path, onnx_path)
        else:
            onnx_path = self.model_path  # Already in ONNX format

        # Step 2: Optimize with TensorRT
        engine_path = os.path.join(self.output_path, f'{base_name}.plan')
        self.optimize_with_tensorrt(onnx_path, engine_path)

        self.get_logger().info(f'Model exported and optimized: {engine_path}')
        return True

    def convert_to_onnx(self, model_path, output_path):
        """Convert PyTorch model to ONNX"""
        cmd = [
            'python3', '-c',
            f'import torch; import torch.onnx; '
            f'model = torch.load("{model_path}"); '
            f'model.eval(); '
            f'dummy_input = torch.randn(1, 3, 224, 224); '
            f'torch.onnx.export(model, dummy_input, "{output_path}", '
            f'export_params=True, opset_version=11)'
        ]

        result = subprocess.run(' '.join(cmd), shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f'ONNX conversion failed: {result.stderr}')
            return False

        self.get_logger().info(f'Model converted to ONNX: {output_path}')
        return True

    def optimize_with_tensorrt(self, onnx_path, engine_path):
        """Optimize ONNX model with TensorRT"""
        precision_flag = '--fp16' if self.precision == 'fp16' else '--fp32'

        cmd = [
            'trtexec',
            f'--onnx={onnx_path}',
            f'--saveEngine={engine_path}',
            precision_flag,
            '--workspace=1024',
            '--explicitBatch'
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f'TensorRT optimization failed: {result.stderr}')
            return False

        self.get_logger().info(f'Model optimized with TensorRT: {engine_path}')
        return True

def main(args=None):
    rclpy.init(args=args)
    exporter = ModelExporter()

    success = exporter.export_model()
    if success:
        exporter.get_logger().info('Model export completed successfully')
    else:
        exporter.get_logger().error('Model export failed')

    exporter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Deployment to Jetson

### 1. Copy Models to Jetson
```bash
# Copy optimized model to Jetson
scp model.plan jetson@jetson_ip:/path/to/models/

# Or use rsync for multiple models
rsync -avz /path/to/models/ jetson@jetson_ip:/path/to/models/
```

### 2. Isaac ROS Integration
```yaml
# perception_pipeline_config.yaml
perception_pipeline:
  ros__parameters:
    # Model configuration
    model_engine_file_path: "/path/to/models/model.plan"
    input_tensor_names: ["input"]
    output_tensor_names: ["output"]
    max_batch_size: 1
    input_binding_names: ["input"]
    output_binding_names: ["output"]

    # Performance settings
    verbose: false
    enable_profiling: false
    collect_performance_data: false

    # TensorRT settings
    context_mode: "default"  # or "safe" for safety-critical applications
    timing_cache: true
```

This guide provides the essential steps for exporting and optimizing perception models from simulation for deployment on Jetson hardware.