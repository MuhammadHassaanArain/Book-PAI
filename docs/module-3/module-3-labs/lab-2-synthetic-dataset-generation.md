---
sidebar_position: 2
---

# Lab 2: Synthetic Dataset Generation for Object Detection

## Objective

In this lab, students will learn to generate synthetic datasets using NVIDIA Isaac Sim with domain randomization techniques. Students will create diverse training data for object detection models and understand how synthetic data can bridge the reality gap in computer vision applications.

## Prerequisites

- Completed Lab 1: Isaac Sim Installation & Scene Setup
- Basic understanding of computer vision concepts
- ROS 2 Humble installed
- Isaac Sim with ROS 2 bridge configured

## Estimated Time

3-4 hours

## Lab Overview

This lab focuses on generating synthetic datasets using Isaac Sim's powerful rendering capabilities and domain randomization features. Students will create a diverse set of images with annotations for object detection tasks.

## Step 1: Scene Setup for Dataset Generation

### Create a New Scene
1. Open Isaac Sim
2. Create a new scene: File → New Scene
3. Save the scene as `synthetic_dataset_scene.usd`

### Set Up the Environment
1. Add a ground plane: Create → Mesh → Plane (scale to 10x10 units)
2. Add objects for detection:
   - Create → Mesh → Cube (add 2-3 cubes of different sizes)
   - Create → Mesh → Sphere (add 2-3 spheres)
   - Create → Mesh → Cylinder (add 1-2 cylinders)
3. Apply different materials to each object using the Material Gallery

### Configure Semantic Segmentation
1. Select each object and assign semantic labels:
   - Select cube → Right-click → Assign → New Material → Semantic Label
   - Assign labels: "cube", "sphere", "cylinder"
2. Verify semantic labels are correctly assigned in the Stage panel

## Step 2: Camera Setup for Data Capture

### Create and Configure Camera
1. Create a camera: Create → Camera → Camera
2. Position the camera at [0, -5, 2] with rotation [15, 0, 0]
3. Set camera properties:
   - Focal Length: 24mm
   - Focus Distance: 5.0
   - F-Stop: 2.8

### Configure Camera Sensors
1. In the Property panel, expand "Camera" properties
2. Set resolution to 640x480 or 1280x720
3. Enable RGB, Depth, and Semantic segmentation outputs

## Step 3: Implement Domain Randomization

### Randomize Lighting Conditions
1. Create a dome light: Create → Light → Dome Light
2. Add randomization graph for dome light:
   - Window → Extensions → Isaac → Random RGL
   - Create a randomization graph for dome light intensity and color
   - Set intensity range: 0.5 to 2.0
   - Set color temperature range: 4000K to 8000K

### Randomize Object Properties
1. Create randomization graph for object materials:
   - Randomize diffuse color within realistic ranges
   - Randomize metallic and roughness values
   - Randomize object positions within bounds
   - Randomize object rotations

### Randomize Camera Parameters
1. Add randomization for camera:
   - Position jitter (±0.2m in X, Y, Z)
   - Rotation jitter (±5° in X, Y, Z)
   - Focal length variation (20mm to 35mm)

## Step 4: Configure Synthetic Data Generation

### Set Up USD RGL (Robotics Graph Language)
1. Create RGL nodes for data generation:
   - Range Image node for depth data
   - Segmentation node for semantic data
   - Detection node for bounding box generation
2. Connect nodes to form the data generation pipeline

### Configure Annotation Generation
1. Set up bounding box generation for each labeled object
2. Configure annotation format (COCO, YOLO, or Pascal VOC)
3. Enable automatic annotation file generation

## Step 5: Execute Dataset Generation

### Write Python Script for Data Generation
Create a Python script to automate the data generation process:

```python
import omni
import carb
from omni.isaac.core import World
from omni.isaac.core.utils import viewports, stage, camera as camera_utils
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np
import cv2
import os

# Initialize Isaac Sim
world = World(stage_units_in_meters=1.0)

# Set up synthetic data helper
synthetic_data_helper = SyntheticDataHelper()

# Create output directory
output_dir = "/path/to/output/dataset"
os.makedirs(output_dir, exist_ok=True)

# Generate multiple frames with randomization
for i in range(1000):  # Generate 1000 frames
    # Apply randomization
    # (This would include randomizing object positions, lighting, etc.)

    # Render frame
    world.step(render=True)

    # Capture RGB, Depth, and Semantic data
    rgb_data = synthetic_data_helper.get_rgb_data()
    depth_data = synthetic_data_helper.get_depth_data()
    semantic_data = synthetic_data_helper.get_semantic_segmentation()

    # Save data
    cv2.imwrite(f"{output_dir}/rgb_{i:04d}.png", rgb_data)
    np.save(f"{output_dir}/depth_{i:04d}.npy", depth_data)
    cv2.imwrite(f"{output_dir}/semantic_{i:04d}.png", semantic_data)

    # Generate and save annotations
    # (Bounding boxes, class labels, etc.)

print("Dataset generation completed!")
```

### Run the Data Generation Script
1. Save the script as `generate_dataset.py`
2. Run in Isaac Sim's scripting window or external Python environment
3. Monitor the generation process for any errors

## Step 6: Validate Generated Dataset

### Check Data Quality
1. Verify that RGB images have realistic appearance
2. Check that depth maps are accurate
3. Confirm semantic segmentation labels are correct
4. Validate bounding box annotations

### Analyze Dataset Statistics
1. Calculate dataset statistics (mean, std, etc.)
2. Check class distribution
3. Verify annotation accuracy
4. Assess diversity of generated samples

## Step 7: Export Dataset in Standard Format

### Convert to COCO Format
```python
import json
import os
from PIL import Image

def convert_to_coco_format(dataset_path, output_path):
    # COCO format structure
    coco_format = {
        "info": {},
        "licenses": [],
        "images": [],
        "annotations": [],
        "categories": []
    }

    # Process each image and annotation
    # (Implementation details for converting synthetic data to COCO format)

    # Save as JSON
    with open(output_path, 'w') as f:
        json.dump(coco_format, f)
```

### Prepare Training Data
1. Organize data into train/validation/test splits
2. Create appropriate directory structure
3. Generate dataset configuration files

## Lab Assessment

### Knowledge Check Questions
1. What is domain randomization and why is it important for synthetic data generation?
2. How does semantic segmentation assist in generating training data for object detection?
3. What are the advantages of synthetic data over real-world data for training?

### Practical Assessment
- Successfully generate 500+ synthetic images with annotations
- Demonstrate domain randomization effects on dataset diversity
- Validate dataset quality and annotation accuracy
- Create proper train/validation splits

### Deliverables
1. Generated dataset (minimum 500 annotated images)
2. Python script for dataset generation
3. Analysis report on dataset quality and diversity
4. Comparison of different domain randomization settings

## Advanced Extensions

### Multi-Object Scenarios
- Create scenes with multiple objects in various configurations
- Implement occlusion handling
- Generate complex interaction scenarios

### Dynamic Scene Generation
- Add moving objects to scenes
- Generate temporal sequences for video-based detection
- Implement realistic motion patterns

## Troubleshooting

### Common Issues
- **Memory errors**: Reduce batch size or scene complexity
- **Annotation errors**: Verify semantic labels are properly assigned
- **Performance issues**: Optimize scene complexity and rendering settings
- **Randomization not working**: Check RGL graph connections

## References

1. NVIDIA Isaac Sim Synthetic Data Generation: https://docs.omniverse.nvidia.com/isaacsim/latest/features/synthetic_data/index.html
2. Domain Randomization in Robotics: https://arxiv.org/abs/1703.06907
3. COCO Dataset Format: https://cocodataset.org/#format-data