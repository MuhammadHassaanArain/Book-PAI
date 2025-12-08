# Data Model: Digital Twin Module for Physical AI Textbook

## Overview
This document defines the key data models for Module 2: Digital Twin (Gazebo & Unity). It captures the entities identified in the feature specification with their attributes, relationships, and validation rules.

## Core Entities

### 1. Digital Twin Model
**Description**: Complete virtual replica of a physical robot, including physical properties, sensor configuration, and visual representation.

**Attributes**:
- `id` (string): Unique identifier for the digital twin
- `name` (string): Human-readable name of the robot
- `physical_properties` (object): Mass, dimensions, material properties
  - `mass` (float): Robot mass in kg
  - `dimensions` (object): Length, width, height in meters
  - `material` (string): Physical material properties
- `sensor_config` (array): List of sensor configurations
- `visual_representation` (object): Visual model data for Unity
  - `model_path` (string): Path to 3D model
  - `textures` (array): Texture file paths
  - `materials` (array): Material definitions

**Relationships**:
- Contains many `Sensor Simulation` instances
- Connected to one `Simulation Environment`

**Validation Rules**:
- `id` must be unique within the system
- `physical_properties.mass` must be positive
- `sensor_config` must contain at least one sensor
- All paths must exist in the file system

### 2. Simulation Environment
**Description**: Virtual world where the digital twin operates, including terrain, obstacles, lighting, and other objects.

**Attributes**:
- `id` (string): Unique identifier for the environment
- `name` (string): Human-readable name of the environment
- `terrain` (object): Terrain configuration
  - `type` (string): Indoor, outdoor, specific terrain type
  - `materials` (array): Surface materials
  - `lighting` (object): Light sources and properties
- `objects` (array): Static and dynamic objects in the environment
- `physics_config` (object): Environment-specific physics parameters

**Relationships**:
- Contains many `Digital Twin Model` instances
- Connected to many `Sensor Simulation` instances (through the twins)

**Validation Rules**:
- `id` must be unique within the system
- `terrain.type` must be a valid terrain type
- `objects` array must not exceed performance limits (e.g., 1000 objects)

### 3. Sensor Simulation
**Description**: Virtual sensors that produce realistic data streams mimicking real-world sensors (LiDAR, cameras, IMUs).

**Attributes**:
- `id` (string): Unique identifier for the sensor
- `type` (string): Sensor type (LiDAR, depth_camera, RGB_camera, IMU)
- `position` (object): Position relative to parent model
  - `x`, `y`, `z` (float): Cartesian coordinates
- `orientation` (object): Orientation relative to parent model
  - `roll`, `pitch`, `yaw` (float): Euler angles in radians
- `parameters` (object): Sensor-specific parameters
  - `range` (float): For LiDAR, range in meters
  - `fov` (float): Field of view in degrees
  - `resolution` (object): Width and height in pixels
  - `noise_model` (object): Noise characteristics
- `ros2_topic` (string): ROS 2 topic name for data output

**Relationships**:
- Belongs to one `Digital Twin Model`
- Connected to `ROS 2 Interface` for data streaming

**Validation Rules**:
- `type` must be one of the supported sensor types
- `position` coordinates must be within reasonable bounds
- `fov` must be between 0 and 360 degrees
- `ros2_topic` must follow ROS 2 naming conventions

### 4. Physics Configuration
**Description**: Parameters that control physical behavior in simulation (gravity, friction, mass, damping, etc.).

**Attributes**:
- `id` (string): Unique identifier for the configuration
- `gravity` (object): Gravity vector
  - `x`, `y`, `z` (float): Gravity components in m/s²
- `solver_iterations` (integer): Number of iterations for physics solver
- `cfm` (float): Constraint Force Mixing parameter
- `erp` (float): Error Reduction Parameter
- `contact_surface_params` (object): Contact surface parameters
  - `mu` (float): Friction coefficient
  - `bounce` (float): Bounciness factor
  - `bounce_vel` (float): Minimum velocity for bouncing

**Relationships**:
- Applied to `Digital Twin Model` and `Simulation Environment`

**Validation Rules**:
- `gravity` components should sum to approximately 9.8 m/s²
- `solver_iterations` must be between 10 and 1000
- `cfm` must be positive (typically 1e-9 to 1e-3)
- `erp` must be between 0 and 1

### 5. ROS 2 Interface
**Description**: Communication layer between simulation and external ROS 2 nodes for sensor data and control commands.

**Attributes**:
- `id` (string): Unique identifier for the interface
- `node_name` (string): ROS 2 node name
- `sensor_topics` (array): List of sensor data topics
- `control_topics` (array): List of control command topics
- `tf_frames` (array): List of coordinate frames
- `services` (array): List of available services

**Relationships**:
- Connected to many `Sensor Simulation` instances
- Connected to `Digital Twin Model` for control

**Validation Rules**:
- `node_name` must follow ROS 2 naming conventions
- Topic names in `sensor_topics` must be unique
- All topics and services must be properly formatted

## State Transitions

### Digital Twin Model States
- `CREATED`: Model defined but not yet loaded in simulation
- `LOADED`: Model loaded in Gazebo simulation
- `ACTIVE`: Model running with physics enabled
- `VISUALIZED`: Model synchronized with Unity visualization
- `DEACTIVATED`: Model paused or stopped

### Sensor Simulation States
- `CONFIGURED`: Sensor parameters set but not active
- `ACTIVE`: Sensor producing data
- `STREAMING`: Sensor data being published to ROS 2
- `ERROR`: Sensor experiencing issues

## Relationships Summary

```
Simulation Environment
        |
        | contains
        |
Digital Twin Model
        |
        | has many
        |
Sensor Simulation
        |
        | connects to
        |
ROS 2 Interface
```

## Validation Rules Summary

### Physics Validation
- All physics parameters must result in stable simulation
- Gravity settings must be physically reasonable
- Contact parameters must prevent simulation instability

### Sensor Validation
- Sensor parameters must be within realistic bounds
- Sensor placement must not cause self-intersection
- Data output must match expected ROS 2 message formats

### Performance Validation
- Simulation must maintain real-time performance (1x speed)
- Multi-robot scenarios must not exceed resource limits
- Unity visualization must maintain 30+ FPS

### Educational Validation
- All configurations must be reproducible with provided documentation
- Examples must work with default hardware specifications
- Content must align with learning objectives from feature spec