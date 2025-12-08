# Lab 1: Configure Gazebo Physics for a Humanoid Robot

## Overview

In this lab, you will learn how to configure physics parameters in Gazebo for humanoid robot simulation. You will explore different physics engines, adjust gravity settings, tune solver parameters, and validate contact dynamics. This hands-on experience will prepare you for creating realistic digital twins of humanoid robots.

## Learning Objectives

By the end of this lab, you will be able to:
1. Configure different physics engines (ODE, Bullet, DART) in Gazebo
2. Adjust gravity parameters for different scenarios
3. Tune solver parameters for optimal performance and stability
4. Configure contact properties for realistic interactions
5. Validate physics simulation using the physics manager
6. Create and test a humanoid robot model with proper physics properties

## Prerequisites

Before starting this lab, ensure you have:
- ROS 2 Humble or Iron installed
- Gazebo (Fortress or Harmonic) installed
- Basic knowledge of URDF and ROS 2 concepts
- Completed Chapter 1 of Module 2

## Lab Environment Setup

### 1. Verify Installation
```bash
# Check if Gazebo is installed
gz --version

# Check if ROS 2 is sourced
printenv | grep ROS_DISTRO

# Navigate to your workspace
cd ~/book_pai_ws
source install/setup.bash
```

### 2. Verify Required Files
Check that the following files exist in your workspace:
- `src/urdf/humanoid_models/physics_humanoid.urdf` (humanoid robot model)
- `simulation-assets/gazebo/worlds/physics_demo.world` (physics demo world)
- `src/python/gazebo_controllers/physics_manager.py` (physics manager script)
- `simulation-assets/ros2/launch/physics_demo.launch.py` (launch file)

## Part 1: Launch Physics Demo Environment

### 1.1 Launch the Physics Demo
```bash
# Launch the physics demo environment
ros2 launch book_pai physics_demo.launch.py
```

### 1.2 Verify Environment
- Confirm that Gazebo has started with the physics_demo.world
- Verify that the physics_humanoid robot model is loaded
- Check that the physics manager node is running

### 1.3 Explore the World
- Use the Gazebo GUI to examine the physics_demo.world
- Identify the different objects used to demonstrate physics concepts
- Note the initial physics parameters (gravity, solver settings, etc.)

## Part 2: Physics Engine Configuration

### 2.1 Check Current Physics Properties
```bash
# Get current physics properties
ros2 service call /gazebo/get_physics_properties gazebo_msgs/srv/GetPhysicsProperties
```

### 2.2 Experiment with Different Physics Engines
1. In the physics_demo.world file, change the physics engine type:
```xml
<physics type="ode">  <!-- Try "bullet" or "dart" -->
```

2. Restart the simulation with the new engine:
```bash
# Stop the current launch
Ctrl+C

# Launch again with modified world
ros2 launch book_pai physics_demo.launch.py
```

3. Compare the behavior of objects with different engines:
   - Note differences in stability
   - Observe contact behavior
   - Record performance differences (real-time factor)

### 2.3 Document Engine Differences
Create a table comparing the behavior of different physics engines:

| Engine | Stability | Contact Behavior | Performance (RTF) | Best Use Cases |
|--------|-----------|------------------|-------------------|----------------|
| ODE    |           |                  |                   |                |
| Bullet |           |                  |                   |                |
| DART   |           |                  |                   |                |

## Part 3: Gravity Configuration

### 3.1 Test Default Gravity
1. Observe the behavior of objects in the physics_demo.world with default Earth gravity (9.8 m/s²)
2. Note how the bouncy sphere, falling box, and pendulum behave

### 3.2 Adjust Gravity Using Physics Manager
```python
# Open a new terminal and run Python commands
python3

# Import ROS 2 Python client libraries
import rclpy
from src.python.gazebo_controllers.physics_manager import PhysicsManager

# Initialize ROS 2
rclpy.init()

# Create physics manager
pm = PhysicsManager()

# Change gravity to Moon gravity
pm.set_gravity(x=0.0, y=0.0, z=-1.62)

# Change gravity to zero (for space simulation)
pm.set_gravity(x=0.0, y=0.0, z=0.0)

# Restore Earth gravity
pm.set_gravity(x=0.0, y=0.0, z=-9.8)

# Exit Python
exit()
```

### 3.3 Validate Gravity Effects
1. After changing gravity, observe how object behavior changes:
   - Falling objects acceleration
   - Pendulum oscillation period
   - Bouncing ball height

2. Record observations for each gravity setting

## Part 4: Solver Parameter Tuning

### 4.1 Adjust Solver Parameters
```python
# In a new Python session
import rclpy
from src.python.gazebo_controllers.physics_manager import PhysicsManager

rclpy.init()
pm = PhysicsManager()

# Set conservative parameters (stable but slow)
pm.set_solver_parameters(
    time_step=0.001,
    max_update_rate=1000.0,
    iters=200,
    sor=1.3
)

# Set performance-oriented parameters (faster but less stable)
pm.set_solver_parameters(
    time_step=0.002,
    max_update_rate=500.0,
    iters=50,
    sor=1.2
)

# Set balanced parameters
pm.set_solver_parameters(
    time_step=0.001,
    max_update_rate=1000.0,
    iters=100,
    sor=1.3
)
```

### 4.2 Test Different Parameter Sets
1. Apply each parameter set and observe the effects on:
   - Simulation stability
   - Object interactions
   - Real-time factor (RTF)
   - Computational performance

2. Record your observations in a table:

| Parameter Set | Time Step | Iterations | SOR | Stability | RTF | Performance |
|---------------|-----------|------------|-----|-----------|-----|-------------|
| Conservative  |           |            |     |           |     |             |
| Performance   |           |            |     |           |     |             |
| Balanced      |           |            |     |           |     |             |

## Part 5: Contact Parameter Configuration

### 5.1 Adjust Contact Parameters
```python
# In Python
import rclpy
from src.python.gazebo_controllers.physics_manager import PhysicsManager

rclpy.init()
pm = PhysicsManager()

# Set parameters for high friction (sticky contacts)
pm.set_contact_parameters(
    cfm=1e-7,
    erp=0.8,
    contact_surface_layer=0.002,
    contact_max_correcting_vel=50.0
)

# Set parameters for low friction (slippery contacts)
pm.set_contact_parameters(
    cfm=1e-4,
    erp=0.1,
    contact_surface_layer=0.001,
    contact_max_correcting_vel=200.0
)

# Set balanced contact parameters
pm.set_contact_parameters(
    cfm=1e-6,
    erp=0.2,
    contact_surface_layer=0.001,
    contact_max_correcting_vel=100.0
)
```

### 5.2 Test Contact Behavior
1. After changing contact parameters, observe:
   - How objects slide vs. grip
   - Stability of stacked objects
   - Bouncing behavior
   - Penetration between objects

2. Document the effects of different contact parameter sets

## Part 6: Humanoid Robot Physics Configuration

### 6.1 Examine Humanoid Robot Model
1. Open the `src/urdf/humanoid_models/physics_humanoid.urdf` file
2. Identify the physics properties for each link:
   - Mass values
   - Inertial properties
   - Joint limits and dynamics
   - Gazebo-specific parameters

### 6.2 Test Humanoid Stability
1. Spawn the humanoid robot in Gazebo (if not already present)
2. Observe its stability with default physics parameters
3. Try adjusting parameters to improve stability:
   - Increase solver iterations for better contact handling
   - Adjust ERP/CFM for better foot-ground contact
   - Modify joint damping for more realistic movement

### 6.3 Validate Humanoid Physics
```python
# Use the physics validation script
python3 tests/gazebo/physics_validation.py
```

## Part 7: Preset Configuration and Application

### 7.1 Use Physics Presets
The physics manager includes several presets for common scenarios:

```python
import rclpy
from src.python.gazebo_controllers.physics_manager import PhysicsManager

rclpy.init()
pm = PhysicsManager()

# Apply stable preset (good for humanoid robots)
pm.apply_preset('stable')

# Apply fast preset (good for performance testing)
pm.apply_preset('fast')

# Apply moon gravity preset
pm.apply_preset('moon_gravity')

# Apply zero gravity preset
pm.apply_preset('zero_gravity')

# Apply default preset
pm.apply_preset('default')
```

### 7.2 Evaluate Preset Performance
1. Apply each preset and observe the simulation behavior
2. Record which preset works best for humanoid robot simulation
3. Note the parameter values for each preset by examining the physics manager code

## Part 8: Performance Optimization

### 8.1 Monitor Performance Metrics
1. In Gazebo GUI, enable statistics display (View → Statistics)
2. Monitor real-time factor (RTF)
3. Observe physics update rate and visual update rate

### 8.2 Optimize for Humanoid Simulation
Based on your observations, determine the optimal parameter set for humanoid robot simulation that balances:
- Stability (no object penetration, stable contacts)
- Performance (RTF close to 1.0)
- Realism (physically plausible behavior)

## Lab Report

### Questions to Answer:

1. Which physics engine performed best for humanoid robot simulation? Why?
2. What gravity settings would be appropriate for simulating a humanoid robot on the Moon?
3. How do solver parameters affect the stability of humanoid robot contacts?
4. What contact parameters provide the best balance between stability and performance?
5. Which preset configuration worked best for humanoid robot simulation?

### Deliverables:

1. Completed observation tables from Parts 2, 4, and 5
2. Answers to the lab questions
3. Recommended physics parameter configuration for humanoid robot simulation
4. Screenshot of the physics_demo.world with the humanoid robot

## Troubleshooting Tips

- If Gazebo fails to start, check that no other Gazebo instances are running
- If physics parameters don't change, ensure the physics manager node is running
- If objects behave erratically, try more conservative solver parameters
- If simulation is too slow, reduce solver iterations or increase time step (within stability limits)

## Conclusion

This lab provided hands-on experience with configuring physics parameters in Gazebo for humanoid robot simulation. You learned how different parameters affect simulation stability, performance, and realism. The skills gained in this lab are essential for creating accurate digital twins of humanoid robots.

## References

1. Gazebo Tutorials. (2021). Physics. http://gazebosim.org/tutorials?tut=physics
2. Open Robotics. (2021). Gazebo User Guide. https://classic.gazebosim.org/userguide/
3. Siciliano, B., & Khatib, O. (Eds.). (2016). Springer Handbook of Robotics. Springer.