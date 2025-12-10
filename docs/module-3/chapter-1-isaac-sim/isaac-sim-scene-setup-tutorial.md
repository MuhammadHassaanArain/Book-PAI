---
title: Isaac Sim Basic Scene Setup with Humanoid Robot Tutorial
sidebar_position: 8
description: Step-by-step tutorial for creating your first Isaac Sim scene with a humanoid robot
---

# Isaac Sim Basic Scene Setup with Humanoid Robot Tutorial

## Overview

This tutorial will guide you through creating your first Isaac Sim scene with a humanoid robot. You'll learn how to set up a basic environment, import a humanoid robot model, configure basic sensors, and run a simple simulation. This foundational knowledge is essential for more advanced robotics development.

## Learning Objectives

By the end of this tutorial, you will be able to:
- Create a new Isaac Sim scene with basic environment
- Import and configure a humanoid robot model
- Set up basic sensors (camera and IMU)
- Run a simple simulation with robot movement
- Verify that the simulation is working correctly

## Prerequisites

Before starting this tutorial, ensure you have:
- Isaac Sim LTS installed and running (see Installation Guide)
- Basic understanding of 3D environments and robotics concepts
- A humanoid robot URDF file (or use one of Isaac Sim's sample robots)

## Step 1: Launch Isaac Sim and Create New Scene

### 1.1 Launch Isaac Sim
1. Open a terminal and navigate to your Isaac Sim directory:
   ```bash
   cd ~/isaac-sim
   ./isaac-sim.sh
   ```

2. Wait for Isaac Sim to fully load (this may take a minute)

### 1.2 Create a New Scene
1. In the menu bar, go to **File** → **New Stage** (or press `Ctrl+N`)
2. Save the new stage by going to **File** → **Save Stage As...**
3. Name your stage `humanoid_scene_01` and save it in your workspace

### 1.3 Set Up Basic Environment
1. In the **Create** menu (or press `Shift+A`), search for "Ground Plane"
2. Add a ground plane to your scene
3. In the **Property** window, adjust the ground plane size to 10x10 meters
4. Add a **Dome Light** for basic illumination:
   - In the **Create** menu, search for "Dome Light"
   - Select the dome light in the **Viewport** or **Stage** window
   - In the **Property** window, set the intensity to 3000 and color to white

## Step 2: Import Humanoid Robot Model

### 2.1 Choose a Humanoid Robot
For this tutorial, we'll use one of Isaac Sim's sample humanoid robots. Isaac Sim includes several sample robots that you can use:

1. In the **Content** window (usually on the right), navigate to **Isaac/Robots**
2. Look for humanoid robot models like "Unitree" or similar
3. If you have your own URDF robot, you can import it using the URDF Import extension

### 2.2 Import Sample Humanoid Robot
1. In the **Content** window, navigate to `Isaac/Robots/Unitree/Go1/urdf/go1_instanceable.usd`
2. Drag and drop the robot into the viewport
3. Position the robot on the ground plane:
   - Select the robot in the **Stage** window or **Viewport**
   - Use the transform gizmo to position it at coordinates (0, 0, 0.5) - slightly above the ground
   - Use the **Property** window to set the translation to (0, 0, 0.5)

### 2.3 Verify Robot Import
1. In the **Stage** window, expand the robot hierarchy to see all links and joints
2. Select different links to verify they are properly connected
3. Check the **Property** window to see robot properties like mass, joint limits, etc.

## Step 3: Configure Basic Sensors

### 3.1 Add RGB Camera
1. With the robot selected, right-click and choose **Add** → **Physics** → **RigidBodyAPI** (if not already present)
2. Right-click on the robot's head link (or chest link) and choose **Add** → **Camera**
3. In the **Property** window for the camera:
   - Set **Resolution Width** to 640
   - Set **Resolution Height** to 480
   - Set **Focal Length** to 24.0
   - Set **Focus Distance** to 10.0

### 3.2 Add IMU Sensor
1. Right-click on the robot's main body link
2. Choose **Add** → **Isaac Sensors** → **Isaac IMU**
3. In the **Property** window for the IMU:
   - Set **Sensor Position** to (0, 0, 0) relative to the link
   - Configure noise parameters as needed for your simulation

### 3.3 Add LiDAR (Optional)
1. Right-click on the robot's head or front link
2. Choose **Add** → **Isaac Sensors** → **Isaac Lidar**
3. Configure LiDAR parameters:
   - Set **Range** to 10.0 meters
   - Set **Resolution** to 0.25 degrees
   - Set **Channels** to 16 (or desired number)

## Step 4: Set Up Basic Environment Objects

### 4.1 Add Simple Obstacles
1. In the **Create** menu, add a **Cube** primitive
2. Position it at (2, 0, 0.5) with a size of (0.5, 0.5, 0.5)
3. Add another cube at (-2, 1, 0.5)
4. You can also add **Cylinder** or **Sphere** objects as obstacles

### 4.2 Add Lighting Configuration
1. Select the **Dome Light** and adjust its properties:
   - Set **Texture** to a simple environment map (if available)
   - Adjust **Tint** to slightly warm color for realism
2. Add a **Distant Light** to create shadows:
   - Create → Light → Distant Light
   - Position it to create realistic shadows
   - Adjust intensity to complement the dome light

## Step 5: Configure Physics and Simulation

### 5.1 Set Physics Properties
1. Select the ground plane
2. In the **Property** window, add **Physics Material**:
   - Add → Physics → Physics Material
   - Set **Static Friction** to 0.5
   - Set **Dynamic Friction** to 0.5
   - Set **Restitution** to 0.1

### 5.2 Configure Robot Physics
1. For each robot link that should participate in physics:
   - Select the link in the **Stage** window
   - Add → Physics → RigidBodyAPI
   - Add → Physics → CollisionAPI
   - Choose appropriate collision shapes (usually **Convex Hull** or **Box**)

### 5.3 Set Simulation Parameters
1. In the **Window** menu, go to **Physics** → **Physics Settings**
2. Adjust the following parameters:
   - **Substeps per frame**: 8 (for stable humanoid simulation)
   - **Solver Type**: TGS (recommended for robotics)
   - **Velocity Iterations**: 4
   - **Position Iterations**: 4

## Step 6: Set Up ROS 2 Bridge (Optional)

### 6.1 Enable ROS 2 Bridge Extension
1. Go to **Window** → **Extensions**
2. Search for "ROS" and enable **Isaac ROS Bridge**
3. Restart Isaac Sim if prompted

### 6.2 Configure ROS Bridge
1. In the **Stage** window, right-click in empty space
2. Go to **Isaac ROS** → **ROS Context**
3. Add **ROS Bridge Node** to your scene
4. Connect your robot's joints to ROS topics using the appropriate bridge extensions

## Step 7: Run the Simulation

### 7.1 Initial Simulation Check
1. Press the **Play** button (spacebar) to start the simulation
2. Verify that:
   - The robot maintains its initial position (if no controllers are active)
   - Physics objects behave naturally
   - Sensors are active and producing data (check in the **Sensors** window)

### 7.2 Test Robot Movement (If Controllers Available)
1. If you have ROS controllers set up, publish joint commands to move the robot
2. If using keyboard controls (if configured), try moving the robot around
3. Observe how the robot interacts with the environment

### 7.3 Camera View
1. In the **Viewport**, you can switch between different cameras
2. If you added a robot camera, you can view the scene from the robot's perspective
3. Use the **View** menu to toggle different camera views

## Step 8: Validate Simulation Behavior

### 8.1 Physics Validation
1. Add a small sphere above the robot and let it fall
2. Verify it collides properly with the robot and ground
3. Check that the humanoid robot maintains balance (if properly configured)

### 8.2 Sensor Validation
1. Check that your camera is producing images
2. Verify IMU is publishing data
3. If using LiDAR, verify point cloud generation

### 8.3 Performance Check
1. Monitor the **Viewport** frame rate
2. Check that the simulation runs smoothly
3. If performance is poor, consider reducing scene complexity

## Step 9: Save and Document Your Scene

### 9.1 Save the Scene
1. Go to **File** → **Save Stage** (or `Ctrl+S`)
2. Consider creating a backup copy with a different name

### 9.2 Document Your Setup
1. Create a README file describing your scene configuration
2. Note down important parameters and settings
3. Document any special configurations for future reference

## Troubleshooting Common Issues

### Issue 1: Robot Falls Through Ground
**Symptoms**: Robot falls through the ground plane
**Solutions**:
- Verify that both the ground plane and robot have collision properties
- Check that the robot's base link is positioned correctly above the ground
- Ensure physics properties are properly configured

### Issue 2: Poor Performance
**Symptoms**: Slow simulation, low frame rate
**Solutions**:
- Reduce the complexity of visual meshes
- Lower physics substeps if stability allows
- Close other applications to free up system resources
- Check that your GPU drivers are up to date

### Issue 3: Sensor Not Working
**Symptoms**: Sensors don't produce data or produce incorrect data
**Solutions**:
- Verify sensor is properly attached to a robot link
- Check sensor parameters in the Property window
- Ensure Isaac Sim's sensor extensions are enabled
- Check for any error messages in the console

### Issue 4: ROS Bridge Not Connecting
**Symptoms**: ROS topics not available or no communication
**Solutions**:
- Verify ROS 2 installation and environment setup
- Check that Isaac Sim ROS bridge extension is enabled
- Ensure ROS bridge nodes are properly added to the scene
- Check ROS domain ID settings

## Advanced Configuration Options

### 1. Adding More Complex Environments
- Import more complex scenes using USD files
- Use Isaac Sim's environment assets
- Create custom environments with multiple rooms or outdoor areas

### 2. Advanced Sensor Configurations
- Configure multiple camera setups (stereo vision)
- Add specialized sensors (force/torque, GPS, etc.)
- Configure sensor noise models to match real hardware

### 3. Domain Randomization Setup
- Set up material randomization for diverse training data
- Configure lighting randomization
- Add texture randomization for improved sim-to-real transfer

## Best Practices

### 1. Scene Organization
- Use meaningful names for objects in your scene
- Organize objects hierarchically in the Stage window
- Group related objects together for easier management

### 2. Performance Optimization
- Use appropriate level of detail for meshes
- Optimize collision geometries
- Use instancing for repeated objects
- Consider occlusion culling for large scenes

### 3. Documentation
- Document your scene setup and configurations
- Keep notes on important parameters
- Version control your scene files
- Create tutorials for complex setups

## Next Steps

After completing this tutorial, you can:
1. **Experiment with Robot Controllers**: Connect your robot to ROS controllers for movement
2. **Add More Sensors**: Configure additional sensors for perception tasks
3. **Create Complex Scenarios**: Build more complex environments for testing
4. **Generate Synthetic Data**: Use your scene for synthetic dataset generation
5. **Implement Domain Randomization**: Add randomization for sim-to-real transfer

## Summary

This tutorial provided a comprehensive introduction to creating your first Isaac Sim scene with a humanoid robot. You learned how to set up the basic environment, import a robot model, configure sensors, and run a simulation. These foundational skills are essential for more advanced robotics development in Isaac Sim.

The combination of Isaac Sim's photorealistic rendering, accurate physics simulation, and ROS integration provides a powerful platform for developing and testing robotics algorithms before deployment to real hardware.

## References

1. NVIDIA Isaac Sim Documentation: [Isaac Sim User Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_intro.html)
2. NVIDIA Isaac Sim Documentation: [Robot Import Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_robot_import.html)
3. ROS 2 Documentation: [ROS 2 with Isaac Sim](https://nvidia-isaac-ros.github.io/concepts/isaac_sim/index.html)
4. NVIDIA Isaac Sim Documentation: [Sensors in Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_sensors.html)