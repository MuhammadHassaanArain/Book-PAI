---
title: Synthetic Dataset Generation with Domain Randomization Tutorial
sidebar_position: 10
description: Comprehensive tutorial for generating synthetic datasets with domain randomization in Isaac Sim
---

# Synthetic Dataset Generation with Domain Randomization Tutorial

## Overview

This tutorial provides a comprehensive guide to generating synthetic datasets using Isaac Sim with domain randomization techniques. Synthetic datasets are crucial for training perception systems in robotics, especially when real-world data is scarce, expensive, or dangerous to collect. Domain randomization helps bridge the gap between synthetic and real-world data by training models on diverse simulated conditions.

## Learning Objectives

By the end of this tutorial, you will be able to:
- Set up Isaac Sim for synthetic dataset generation
- Implement domain randomization techniques
- Generate various types of synthetic data (RGB, depth, segmentation)
- Validate and assess the quality of generated datasets
- Integrate synthetic datasets with machine learning pipelines

## Prerequisites

Before starting this tutorial, ensure you have:
- Isaac Sim LTS installed and configured
- Basic understanding of computer vision and machine learning
- Knowledge of your target perception task (object detection, segmentation, etc.)
- A clear understanding of the real-world conditions you want to simulate

## Understanding Synthetic Dataset Generation

### Why Synthetic Data?

Synthetic dataset generation offers several advantages:
- **Cost-Effective**: No need for expensive real-world data collection
- **Safe Testing**: Test dangerous scenarios without risk
- **Perfect Annotations**: Automatic ground truth generation
- **Controlled Conditions**: Systematically vary environmental parameters
- **Infinite Data**: Generate as much data as needed

### Domain Randomization Benefits

Domain randomization addresses the sim-to-real gap by:
- **Increasing Robustness**: Models become invariant to domain-specific features
- **Reducing Overfitting**: Prevents models from relying on simulation artifacts
- **Improving Generalization**: Better performance on real-world data
- **Expanding Training Diversity**: Exposure to wide variety of conditions

## Setting Up Dataset Generation Environment

### 1. Install Required Extensions

First, ensure you have the necessary Isaac Sim extensions enabled:

1. Go to **Window** → **Extensions**
2. Enable the following extensions:
   - **Isaac Sensors**: For sensor simulation
   - **Isaac Synthetic Data**: For automatic annotation generation
   - **Isaac Domain Randomization**: For domain randomization capabilities
   - **Isaac ROS Bridge**: If integrating with ROS workflows

### 2. Create Dataset Generation Scene

Create a new scene optimized for dataset generation:

1. **File** → **New Stage** to create a new scene
2. Save the stage as `dataset_generation_scene`
3. Add the following elements to your scene:

#### Environment Setup
- **Ground Plane**: Large enough to accommodate random object placement
- **Dome Light**: For environment lighting
- **Additional Lights**: To create diverse lighting conditions
- **Background Objects**: Diverse objects for realistic scenes

#### Robot/Camera Setup
- **Camera**: Position and configure your primary camera
- **Sensor Mount**: If using a robot platform
- **Multiple Cameras**: For multi-view datasets if needed

### 3. Configure Synthetic Data Extensions

Enable and configure synthetic data generation:

```python
# Python code to enable synthetic data extensions
import omni
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Enable synthetic data extensions
omni.kit.commands.execute("ExtensionManagerEnable", extension_id="omni.isaac.synthetic_utils")
omni.kit.commands.execute("ExtensionManagerEnable", extension_id="omni.isaac.sensor")

# Initialize synthetic data helper
synthetic_data = SyntheticDataHelper()
```

## Implementing Domain Randomization

### 1. Lighting Randomization

Configure randomization for lighting conditions:

#### Dome Light Randomization
```python
# Example Python code for dome light randomization
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import UsdLux, Gf
import random

def randomize_dome_light(light_path):
    light_prim = get_prim_at_path(light_path)
    dome_light = UsdLux.DomeLight(light_prim)

    # Randomize intensity (e.g., 500 to 2000)
    intensity = random.uniform(500.0, 2000.0)
    dome_light.CreateIntensityAttr(intensity)

    # Randomize color temperature (e.g., 3000K to 8000K)
    color_temp = random.uniform(3000.0, 8000.0)
    # Convert color temperature to RGB (simplified)
    dome_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
```

#### Directional Light Randomization
```python
def randomize_directional_light(light_path):
    light_prim = get_prim_at_path(light_path)
    distant_light = UsdLux.DistantLight(light_prim)

    # Randomize direction
    x = random.uniform(-1.0, 1.0)
    y = random.uniform(-1.0, 1.0)
    z = random.uniform(-1.0, 0.0)  # Always pointing somewhat downward
    direction = Gf.Vec3f(x, y, z).GetNormalized()

    # Randomize intensity
    intensity = random.uniform(5000.0, 15000.0)

    distant_light.CreateIntensityAttr(intensity)
```

### 2. Material Randomization

Randomize material properties for objects in your scene:

#### Color Randomization
```python
def randomize_material_color(material_path):
    material_prim = get_prim_at_path(material_path)

    # Generate random color
    r = random.uniform(0.1, 1.0)
    g = random.uniform(0.1, 1.0)
    b = random.uniform(0.1, 1.0)

    # Apply to material's diffuse color
    # This is a simplified example - actual implementation depends on material type
    pass
```

#### Texture Randomization
```python
def randomize_material_textures(material_path, texture_library):
    material_prim = get_prim_at_path(material_path)

    # Randomly select a texture from the library
    random_texture = random.choice(texture_library)

    # Apply the texture to the material
    # Implementation depends on material system used
    pass
```

### 3. Object Placement Randomization

Randomize the placement and properties of objects in your scene:

```python
def randomize_object_placement(objects_list, area_bounds):
    for obj in objects_list:
        # Random position within bounds
        x = random.uniform(area_bounds['min_x'], area_bounds['max_x'])
        y = random.uniform(area_bounds['min_y'], area_bounds['max_y'])
        z = random.uniform(area_bounds['min_z'], area_bounds['max_z'])

        # Random rotation
        rot_x = random.uniform(0, 360)
        rot_y = random.uniform(0, 360)
        rot_z = random.uniform(0, 360)

        # Apply transforms to the object
        # Implementation depends on your object management system
        pass
```

### 4. Camera Parameter Randomization

Randomize camera parameters to simulate different sensor characteristics:

```python
def randomize_camera_parameters(camera_path):
    camera_prim = get_prim_at_path(camera_path)

    # Randomize focal length (for variable focal length cameras)
    focal_length = random.uniform(18.0, 85.0)  # in mm equivalent

    # Randomize sensor noise parameters
    noise_level = random.uniform(0.001, 0.01)

    # Randomize distortion parameters
    k1 = random.uniform(-0.1, 0.1)
    k2 = random.uniform(-0.05, 0.05)

    # Apply these parameters to the camera
    pass
```

## Configuring Synthetic Data Generation

### 1. Setting Up Sensor Outputs

Configure your camera to generate multiple types of synthetic data:

#### RGB Images
```python
# Enable RGB image capture
def setup_rgb_capture(camera_path, output_directory):
    # Configure camera to capture RGB images
    # Set up file naming convention
    # Specify output format and resolution
    pass
```

#### Depth Maps
```python
# Enable depth map generation
def setup_depth_capture(camera_path, output_directory):
    # Configure depth sensor
    # Set depth range and precision
    # Specify output format
    pass
```

#### Semantic Segmentation
```python
# Enable semantic segmentation masks
def setup_semantic_segmentation(camera_path, output_directory):
    # Configure semantic segmentation
    # Set up class labels and color mapping
    # Ensure all objects have semantic labels
    pass
```

#### Instance Segmentation
```python
# Enable instance segmentation masks
def setup_instance_segmentation(camera_path, output_directory):
    # Configure instance segmentation
    # Ensure each object has unique instance ID
    # Set up instance label mapping
    pass
```

### 2. Annotation Generation

Set up automatic annotation generation:

#### Bounding Boxes
```python
# Generate 2D bounding boxes
def setup_bounding_box_annotations(camera_path, output_directory):
    # Configure 2D bounding box generation
    # Set up format (COCO, Pascal VOC, etc.)
    # Ensure all objects have class labels
    pass
```

#### 3D Annotations
```python
# Generate 3D bounding boxes and poses
def setup_3d_annotations(camera_path, output_directory):
    # Configure 3D annotation generation
    # Set up 6D pose estimation
    # Generate 3D bounding boxes
    pass
```

## Creating the Dataset Generation Pipeline

### 1. Scene Configuration Script

Create a Python script to manage the entire generation process:

```python
# dataset_generation_pipeline.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import random
import os
from datetime import datetime

class DatasetGenerator:
    def __init__(self, output_directory):
        self.output_directory = output_directory
        self.world = World(stage_units_in_meters=1.0)
        self.current_sample_id = 0

        # Create output directories
        os.makedirs(os.path.join(output_directory, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(output_directory, "depth"), exist_ok=True)
        os.makedirs(os.path.join(output_directory, "seg"), exist_ok=True)
        os.makedirs(os.path.join(output_directory, "annotations"), exist_ok=True)

    def randomize_scene(self):
        """Apply domain randomization to the scene"""
        # Randomize lighting
        self.randomize_lighting()

        # Randomize materials
        self.randomize_materials()

        # Randomize object positions
        self.randomize_object_positions()

        # Randomize camera parameters
        self.randomize_camera_params()

    def capture_data(self):
        """Capture all synthetic data for current scene state"""
        # Capture RGB image
        self.capture_rgb()

        # Capture depth map
        self.capture_depth()

        # Capture segmentation
        self.capture_segmentation()

        # Generate annotations
        self.generate_annotations()

        # Increment sample ID
        self.current_sample_id += 1

    def generate_dataset(self, num_samples):
        """Main function to generate the dataset"""
        for i in range(num_samples):
            print(f"Generating sample {i+1}/{num_samples}")

            # Randomize the scene
            self.randomize_scene()

            # Wait for scene to stabilize
            for _ in range(2):  # Run a few physics steps
                self.world.step(render=True)

            # Capture data
            self.capture_data()

            # Reset scene for next sample if needed
            # (this depends on your specific requirements)

    def randomize_lighting(self):
        # Implementation of lighting randomization
        pass

    def randomize_materials(self):
        # Implementation of material randomization
        pass

    def randomize_object_positions(self):
        # Implementation of object position randomization
        pass

    def randomize_camera_params(self):
        # Implementation of camera parameter randomization
        pass

    def capture_rgb(self):
        # Implementation of RGB image capture
        pass

    def capture_depth(self):
        # Implementation of depth map capture
        pass

    def capture_segmentation(self):
        # Implementation of segmentation capture
        pass

    def generate_annotations(self):
        # Implementation of annotation generation
        pass

# Usage example
if __name__ == "__main__":
    generator = DatasetGenerator("./synthetic_dataset")
    generator.generate_dataset(num_samples=1000)
```

### 2. Advanced Randomization Techniques

#### Procedural Environment Generation
```python
def generate_random_environment():
    """Generate random indoor/outdoor environments"""
    # Randomly select environment type
    env_type = random.choice(["indoor", "outdoor", "warehouse", "street"])

    # Generate environment based on type
    if env_type == "indoor":
        return generate_indoor_environment()
    elif env_type == "outdoor":
        return generate_outdoor_environment()
    # ... other environment types
```

#### Weather Randomization
```python
def randomize_weather_conditions():
    """Randomize weather conditions"""
    # Randomize fog density
    fog_density = random.uniform(0.0, 0.1)

    # Randomize atmospheric properties
    atmosphere_haze = random.uniform(0.0, 0.05)

    # Randomize precipitation (if supported)
    precipitation = random.uniform(0.0, 0.2)

    # Apply weather effects
    apply_atmospheric_effects(fog_density, atmosphere_haze, precipitation)
```

## Quality Validation and Assessment

### 1. Statistical Validation

Compare synthetic and real data distributions:

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

def validate_dataset_quality(synthetic_data, real_data):
    """Validate synthetic dataset quality against real data"""

    # Compare color distributions
    syn_colors = extract_color_histogram(synthetic_data)
    real_colors = extract_color_histogram(real_data)

    # Calculate statistical distance
    color_distance = stats.wasserstein_distance(syn_colors, real_colors)

    # Compare texture properties
    syn_textures = extract_texture_features(synthetic_data)
    real_textures = extract_texture_features(real_data)

    # Compare edge distributions
    syn_edges = extract_edge_features(synthetic_data)
    real_edges = extract_edge_features(real_data)

    return {
        'color_distance': color_distance,
        'texture_similarity': calculate_similarity(syn_textures, real_textures),
        'edge_similarity': calculate_similarity(syn_edges, real_edges)
    }
```

### 2. Model Performance Validation

Test model performance on both synthetic and real data:

```python
def validate_model_transfer(synthetic_model, real_data, real_labels):
    """Validate model trained on synthetic data on real data"""

    # Get predictions on real data
    predictions = synthetic_model.predict(real_data)

    # Calculate performance metrics
    accuracy = calculate_accuracy(predictions, real_labels)
    precision = calculate_precision(predictions, real_labels)
    recall = calculate_recall(predictions, real_labels)

    return {
        'accuracy': accuracy,
        'precision': precision,
        'recall': recall
    }
```

## Advanced Domain Randomization Strategies

### 1. Curriculum Randomization

Start with less randomization and gradually increase:

```python
class CurriculumRandomizer:
    def __init__(self):
        self.current_level = 0
        self.max_level = 5

    def get_randomization_params(self, sample_number, total_samples):
        # Calculate curriculum level based on training progress
        progress = sample_number / total_samples
        self.current_level = int(progress * self.max_level)

        # Adjust randomization intensity based on level
        intensity = min(progress * 2, 1.0)  # Cap at 100%

        return self.get_params_for_level(self.current_level, intensity)

    def get_params_for_level(self, level, intensity):
        # Define randomization parameters for each level
        base_params = {
            'light_intensity_range': (500, 2000),
            'color_variance': 0.1,
            'texture_randomization': 0.3
        }

        # Scale parameters based on level and intensity
        scaled_params = {}
        for key, value in base_params.items():
            if isinstance(value, tuple):
                # For ranges, scale the range width
                center = (value[0] + value[1]) / 2
                width = (value[1] - value[0]) * intensity
                scaled_params[key] = (center - width/2, center + width/2)
            else:
                # For single values, scale the value
                scaled_params[key] = value * intensity

        return scaled_params
```

### 2. Adversarial Randomization

Use adversarial techniques to find challenging scenarios:

```python
def adversarial_randomization(model, current_scene_params):
    """Find scene parameters that challenge the model the most"""

    # Define scene parameters as variables to optimize
    scene_variables = {
        'lighting': current_scene_params['lighting'],
        'materials': current_scene_params['materials'],
        'objects': current_scene_params['objects']
    }

    # Calculate model confidence on current scene
    current_confidence = evaluate_model_confidence(model, current_scene_params)

    # Perturb parameters to minimize model confidence
    adversarial_params = perturb_for_adversarial_effect(
        scene_variables,
        model,
        current_confidence
    )

    return adversarial_params
```

## Integration with Machine Learning Pipelines

### 1. Dataset Format Conversion

Convert synthetic data to standard formats:

```python
def convert_to_coco_format(synthetic_data_dir, output_coco_file):
    """Convert synthetic dataset to COCO format"""

    coco_format = {
        "info": {
            "description": "Synthetic Dataset with Domain Randomization",
            "version": "1.0",
            "year": datetime.now().year
        },
        "licenses": [],
        "images": [],
        "annotations": [],
        "categories": []
    }

    # Process each image in the dataset
    for image_file in os.listdir(os.path.join(synthetic_data_dir, "rgb")):
        # Add image info
        image_info = {
            "id": len(coco_format["images"]),
            "file_name": image_file,
            "width": image_width,
            "height": image_height
        }
        coco_format["images"].append(image_info)

        # Process annotations for this image
        annotation_file = image_file.replace(".png", "_annotations.json")
        annotations = load_annotations(os.path.join(synthetic_data_dir, "annotations", annotation_file))

        for ann in annotations:
            ann["id"] = len(coco_format["annotations"])
            ann["image_id"] = image_info["id"]
            coco_format["annotations"].append(ann)

    # Save COCO format dataset
    with open(output_coco_file, 'w') as f:
        json.dump(coco_format, f)
```

### 2. Training Pipeline Integration

Integrate synthetic data with your training pipeline:

```python
def train_with_synthetic_data(model, synthetic_dataset_path, real_dataset_path, epochs):
    """Train model using both synthetic and real data"""

    # Load synthetic dataset
    syn_loader = create_dataloader(synthetic_dataset_path, batch_size=32)

    # Load real dataset
    real_loader = create_dataloader(real_dataset_path, batch_size=32)

    # Training loop
    for epoch in range(epochs):
        # Train on synthetic data
        for batch_idx, (syn_data, syn_targets) in enumerate(syn_loader):
            syn_loss = model.train_step(syn_data, syn_targets)

        # Validate on real data
        if epoch % 5 == 0:  # Validate every 5 epochs
            real_accuracy = validate_on_real_data(model, real_loader)
            print(f"Epoch {epoch}: Real data accuracy: {real_accuracy}")
```

## Best Practices and Tips

### 1. Randomization Strategy

#### Physics-Aware Randomization
- Ensure randomization parameters remain physically plausible
- Consider real-world constraints when defining parameter ranges
- Validate that randomized scenes still make physical sense

#### Correlation-Aware Randomization
- Randomize correlated parameters together (e.g., lighting and shadows)
- Consider how different randomization factors interact
- Avoid creating unrealistic combinations

### 2. Performance Optimization

#### Efficient Randomization
- Cache expensive computations where possible
- Use efficient sampling methods
- Parallelize randomization where appropriate

#### Memory Management
- Stream data to disk rather than keeping everything in memory
- Use appropriate data formats to minimize storage
- Implement proper cleanup procedures

### 3. Quality Control

#### Automated Validation
- Implement automated checks for data quality
- Monitor for unrealistic or impossible scenarios
- Track statistics to ensure randomization is working as expected

#### Human-in-the-Loop Validation
- Periodically review generated samples manually
- Collect feedback from domain experts
- Adjust randomization parameters based on validation results

## Troubleshooting Common Issues

### Issue 1: Poor Real-World Transfer
**Symptoms**: Model performs well on synthetic data but poorly on real data
**Solutions**:
- Increase domain randomization diversity
- Add more realistic noise models
- Include real-world data in training (sim + real training)
- Analyze the specific failure modes

### Issue 2: Unrealistic Synthetic Data
**Symptoms**: Generated data looks obviously synthetic
**Solutions**:
- Reduce the range of randomization parameters
- Focus on realistic parameter combinations
- Add subtle details that match real-world conditions
- Use reference images to guide randomization

### Issue 3: Performance Bottlenecks
**Symptoms**: Slow dataset generation, high memory usage
**Solutions**:
- Optimize scene complexity
- Use level-of-detail models
- Implement efficient randomization algorithms
- Consider distributed generation across multiple machines

### Issue 4: Insufficient Diversity
**Symptoms**: Generated data lacks diversity, models overfit
**Solutions**:
- Increase parameter ranges for randomization
- Add more randomization factors
- Implement curriculum or adversarial randomization
- Ensure balanced representation of different scenarios

## Advanced Topics

### 1. Neural Domain Randomization

#### Learned Randomization
```python
import torch
import torch.nn as nn

class RandomizationNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        # Network to learn optimal randomization parameters
        self.network = nn.Sequential(
            nn.Linear(10, 64),  # Input: current scene state
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 20)   # Output: randomization parameters
        )

    def forward(self, scene_state):
        return self.network(scene_state)
```

### 2. Active Learning Integration

Use active learning to identify important scenarios:

```python
def active_learning_guided_generation(model, uncertainty_threshold=0.1):
    """Generate data for scenarios where model is most uncertain"""

    while True:
        # Generate random scene
        scene_params = generate_random_scene()

        # Get model prediction and uncertainty
        prediction, uncertainty = model.predict_with_uncertainty(scene_params)

        # If uncertainty is high, add this scene to dataset
        if uncertainty > uncertainty_threshold:
            capture_scene_data(scene_params)
```

## Evaluation Metrics

### 1. Dataset Quality Metrics

#### Diversity Metrics
```python
def calculate_dataset_diversity(dataset):
    """Calculate diversity metrics for synthetic dataset"""

    # Feature-based diversity
    features = extract_features_from_dataset(dataset)
    diversity_score = calculate_feature_space_coverage(features)

    # Statistical diversity
    color_diversity = calculate_color_space_diversity(dataset)
    texture_diversity = calculate_texture_diversity(dataset)

    return {
        'feature_diversity': diversity_score,
        'color_diversity': color_diversity,
        'texture_diversity': texture_diversity
    }
```

### 2. Transfer Performance Metrics

#### Sim-to-Real Gap Measurement
```python
def measure_sim_to_real_gap(syn_model_performance, real_model_performance):
    """Measure the sim-to-real performance gap"""

    gap = abs(syn_model_performance - real_model_performance)

    return {
        'absolute_gap': gap,
        'relative_gap': gap / syn_model_performance if syn_model_performance != 0 else float('inf'),
        'performance_ratio': real_model_performance / syn_model_performance if syn_model_performance != 0 else 0
    }
```

## Summary

This tutorial provided a comprehensive guide to generating synthetic datasets with domain randomization in Isaac Sim. The key components include:

1. **Environment Setup**: Configuring Isaac Sim with necessary extensions
2. **Randomization Techniques**: Implementing lighting, material, and object randomization
3. **Data Generation**: Capturing RGB, depth, segmentation, and annotation data
4. **Quality Validation**: Assessing dataset quality and model transfer performance
5. **Best Practices**: Following proven strategies for effective domain randomization

The combination of photorealistic rendering, accurate physics simulation, and systematic domain randomization in Isaac Sim provides a powerful platform for generating high-quality synthetic datasets that can effectively train robust perception systems for robotics applications.

## References

1. NVIDIA Isaac Sim Documentation: [Synthetic Data Generation](https://docs.omniverse.nvidia.com/isaacsim/latest/synthetic_data_generation.html)
2. Tobin, J., et al. (2017). Domain randomization for transferring deep neural networks from simulation to the real world. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.
3. NVIDIA Isaac Sim Documentation: [Domain Randomization](https://docs.omniverse.nvidia.com/isaacsim/latest/domain_randomization.html)
4. Peng, X. B., et al. (2020). Learning dexterous in-hand manipulation. *The International Journal of Robotics Research*.