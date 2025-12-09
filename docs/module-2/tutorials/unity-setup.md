# Unity Setup Tutorial

## Overview

This tutorial guides you through setting up Unity for robotics visualization and digital twin applications. We'll cover Unity installation, robotics-specific configuration, and integration with ROS 2 for creating high-fidelity visualizations of robotic systems.

## Prerequisites

- Windows 10/11, macOS 10.14+, or Ubuntu 20.04+
- Unity Hub installed (recommended)
- Minimum 8GB RAM (16GB+ recommended for robotics applications)
- Dedicated GPU with DirectX 11/Vulkan support
- Internet connection for package downloads

## Installation

### Step 1: Install Unity Hub

Unity Hub is the recommended way to manage Unity installations:

1. Download Unity Hub from https://unity.com/download
2. Run the installer and follow the setup wizard
3. Sign in with your Unity ID (free account) or create one
4. Unity Hub provides centralized management of Unity versions and projects

### Step 2: Install Unity Editor

Install Unity LTS (Long Term Support) version:

1. Open Unity Hub
2. Click "Installs" tab
3. Click "Add" to add a new Unity version
4. Select "2022.3.x LTS" (most recent LTS version)
5. In the installer, select these modules:
   - Windows Build Support (IL2CPP) [or Mac/Linux as needed]
   - Universal Windows Platform Build Support (IL2CPP)
   - Android Build Support (if needed)
   - Visual Studio Tools for Unity
6. Install Unity

### Step 3: Install Required Packages

Launch Unity and install robotics-specific packages:

1. Open Unity Hub
2. Click "Projects" tab
3. Click "New" to create a new project
4. Select "3D (Built-in Render Pipeline)" template
5. Name your project "RoboticsVisualization"
6. Click "Create"

## Project Configuration

### Step 4: Configure Project Settings

After creating your project, configure it for robotics applications:

1. Go to `Edit > Project Settings`
2. In "Player" settings:
   - Set Company Name and Product Name
   - Under "Resolution and Presentation":
     - Uncheck "Resizable Window"
     - Set default screen resolution (e.g., 1920x1080)
   - Under "XR Settings":
     - Enable "Virtual Reality Supported" if needed for VR HRI
     - Add "OpenVR" or "Windows MR" as needed

3. In "Graphics" settings:
   - For robotics visualization, Built-in Render Pipeline is sufficient
   - For high-quality rendering, consider upgrading to URP or HDRP later

4. In "Physics" settings:
   - Set Gravity to (0, -9.81, 0) to match real world
   - Adjust Default Solver Iterations (higher for stability, lower for performance)

### Step 5: Install Robotics-Specific Packages

Open the Package Manager to install robotics packages:

1. Go to `Window > Package Manager`
2. In the dropdown, select "Unity Registry"
3. Install these packages:
   - **Ros TCP Connector**: For ROS 2 communication
   - **XR Interaction Toolkit**: For VR/AR HRI (if needed)
   - **Unity Robotics Tools**: For robotics-specific utilities
   - **ProBuilder**: For rapid environment prototyping

## Robotics Integration Setup

### Step 6: Set Up ROS 2 Communication

Install and configure ROS 2 communication tools:

1. In Package Manager, install "ROS TCP Connector"
2. Create a new folder: `Assets/Scripts/ROS`
3. Add the ROS TCP Connection Manager to your scene:
   - Create an empty GameObject: `GameObject > Create Empty`
   - Name it "ROSConnection"
   - Add "ROS TCP Connection" component from the package
   - Configure the connection settings:
     - Host: "127.0.0.1" (localhost)
     - Port: "10000" (default ROS TCP port)

### Step 7: Create Basic Robotics Scene

Set up a basic scene structure for robotics:

```csharp
// Create this script in Assets/Scripts/ROS/RobotController.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotController : MonoBehaviour
{
    [SerializeField]
    private string topicName = "/unity_robot_pose";

    private ROSConnection ros;

    void Start()
    {
        // Get reference to ROS connection object
        ros = ROSConnection.GetOrCreateInstance();

        // Register topic
        ros.RegisterPublisher<UInt64Msg>(topicName);
    }

    void Update()
    {
        // Publish robot position periodically
        if (Time.time % 0.1f < Time.deltaTime) // Every 0.1 seconds
        {
            var positionMsg = new UInt64Msg();
            positionMsg.data = (ulong)Time.time;

            ros.Publish(topicName, positionMsg);
        }
    }
}
```

### Step 8: Configure Coordinate System

Set up proper coordinate system conversion for robotics:

```csharp
// Create this script in Assets/Scripts/Utilities/CoordinateConverter.cs
using UnityEngine;

public class CoordinateConverter : MonoBehaviour
{
    // Convert from ROS/Gazebo coordinate system to Unity
    // ROS/Gazebo: X-forward, Y-left, Z-up
    // Unity: X-right, Y-up, Z-forward
    public static Vector3 RosToUnityPosition(Vector3 rosPos)
    {
        return new Vector3(-rosPos.y, rosPos.z, rosPos.x);
    }

    public static Vector3 UnityToRosPosition(Vector3 unityPos)
    {
        return new Vector3(unityPos.z, -unityPos.x, unityPos.y);
    }

    public static Quaternion RosToUnityRotation(Quaternion rosRot)
    {
        // Convert quaternion from ROS to Unity coordinate system
        return new Quaternion(-rosRot.z, rosRot.x, -rosRot.y, rosRot.w);
    }

    public static Quaternion UnityToRosRotation(Quaternion unityRot)
    {
        // Convert quaternion from Unity to ROS coordinate system
        return new Quaternion(-unityRot.y, unityRot.z, -unityRot.x, unityRot.w);
    }

    public static Vector3 RosToUnityVector(Vector3 rosVector)
    {
        return new Vector3(-rosVector.y, rosVector.z, rosVector.x);
    }
}
```

## High-Fidelity Visualization Setup

### Step 9: Configure Rendering Settings

For high-quality robotics visualization:

1. Go to `Edit > Project Settings > Graphics`
2. If using Built-in Render Pipeline, consider these optimizations:
   - Increase "Realtime GI Bounces" to 2-3 for better lighting
   - Set "Pixel Light Count" to 4 for multiple light sources
   - Enable "GPU Skinning" for animated robots

3. For enhanced visual quality, consider switching to URP:
   - Go to Package Manager
   - Install "Universal Render Pipeline"
   - Create URP Asset: `Assets > Create > Rendering > Universal Render Pipeline > Pipeline Asset`
   - In Player Settings, under Graphics, set Scriptable Render Pipeline Settings to your new URP asset

### Step 10: Set Up Lighting System

Configure realistic lighting for robotics environments:

1. Select the "Directional Light" in your scene
2. Configure for realistic outdoor lighting:
   - Set Intensity to 1.0
   - Set Color to a warm white (e.g., RGB: 255, 244, 228)
   - Set Rotation to (50, -30, 0) for natural sun position
   - Enable "Shadows" (Hard or Soft as needed)

3. Add additional lights for indoor environments:
   - Create Point Lights for ceiling fixtures
   - Create Spot Lights for focused illumination
   - Configure light cookies for realistic light patterns

### Step 11: Import Robotics Assets

Set up a proper folder structure for robotics assets:

```
Assets/
├── Models/
│   ├── Robots/
│   ├── Environments/
│   └── Props/
├── Materials/
├── Scripts/
│   ├── ROS/
│   ├── Robotics/
│   └── Utilities/
├── Scenes/
└── Prefabs/
```

1. Create the folder structure
2. Import robot models in FBX or OBJ format
3. Create material folders for different robot parts
4. Set up prefabs for commonly used robot configurations

## Performance Optimization

### Step 12: Configure Performance Settings

Optimize Unity for real-time robotics visualization:

1. Go to `Edit > Project Settings > Quality`
2. Set up different quality levels:
   - **Ultra**: For high-end systems with maximum visual quality
   - **High**: Good balance for most robotics applications
   - **Medium**: For systems with performance constraints

3. Configure specific settings for robotics:
   - Anti-aliasing: FXAA or MSAA as needed
   - Shadow resolution: Medium to High (balance quality vs. performance)
   - Shadow distance: 50-100m for robotics environments
   - Texture Quality: Full-size or Half depending on performance needs

### Step 13: Set Up Level of Detail (LOD)

Create LOD groups for complex robot models:

1. Select a robot model with multiple parts
2. Add "LOD Group" component: `Component > Rendering > LOD Group`
3. Create 2-3 LOD levels:
   - LOD 0: High detail (full geometry, high-res textures)
   - LOD 1: Medium detail (simplified geometry, medium textures)
   - LOD 2: Low detail (lowest poly count, low-res textures)
4. Set transition distances based on typical viewing distances

## VR/AR Setup (Optional)

### Step 14: Configure VR Support

If you need VR for immersive HRI:

1. Install "XR Interaction Toolkit" from Package Manager
2. Go to `Edit > Project Settings > XR Plug-in Management`
3. Enable your target platform (OpenVR for SteamVR, Oculus for Quest, etc.)
4. Install the respective XR plug-in from Package Manager
5. Add XR Origin to your scene:
   - `GameObject > XR > XR Origin (VR)`
   - Replace the main camera with the XROrigin camera

## Testing Your Setup

### Step 15: Create Test Scene

Create a simple test scene to verify your setup:

1. Create a new scene: `File > New Scene`
2. Set up the scene hierarchy:
   ```
   Main Camera
   Directional Light
   RobotModel (imported robot prefab)
   Environment (simple floor/walls)
   ROSConnection (empty with ROS TCP connector)
   ```

3. Add a simple test script to verify functionality:

```csharp
// Create in Assets/Scripts/Test/RosTest.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RosTest : MonoBehaviour
{
    public string topicName = "/test_pose";
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);
    }

    void Update()
    {
        // Send a test message every 2 seconds
        if (Time.time % 2.0f < Time.deltaTime)
        {
            var poseMsg = new PoseMsg();
            poseMsg.position.x = transform.position.x;
            poseMsg.position.y = transform.position.y;
            poseMsg.position.z = transform.position.z;

            ros.Publish(topicName, poseMsg);
        }
    }
}
```

4. Attach this script to an empty GameObject
5. Run the scene and verify no errors appear in Console

## Troubleshooting

### Common Issues and Solutions

#### Issue 1: Unity Crashes on Startup
**Symptoms**: Unity editor crashes immediately after opening
**Solutions**:
1. Update graphics drivers
2. Check system requirements are met
3. Try running Unity with `-force-opengl` command line argument
4. Clear Unity cache: `%appdata%\..\LocalLow\Unity\` (Windows) or `~/Library/Application Support/Unity/` (Mac)

#### Issue 2: ROS Connection Fails
**Symptoms**: Cannot connect to ROS 2 network
**Solutions**:
1. Verify ROS 2 is running: `ros2 topic list`
2. Check firewall settings block port 10000
3. Ensure ROS_DOMAIN_ID matches between Unity and ROS 2
4. Verify IP address settings in ROS TCP Connector

#### Issue 3: Poor Performance
**Symptoms**: Low frame rates or stuttering
**Solutions**:
1. Reduce shadow resolution
2. Lower anti-aliasing quality
3. Use simpler materials/shaders
4. Enable occlusion culling for large environments
5. Use object pooling for frequently instantiated objects

#### Issue 4: Coordinate System Issues
**Symptoms**: Robot appears rotated or positioned incorrectly
**Solutions**:
1. Use the coordinate converter script provided
2. Verify Gazebo and Unity coordinate system alignment
3. Check rotation offsets on imported models

## Performance Optimization Tips

### Step 16: Optimize for Robotics Applications

1. **Use Object Pooling**: For frequently instantiated/destructed objects like sensor points
2. **Implement Frustum Culling**: Only render objects visible to cameras
3. **Use Occlusion Culling**: Hide objects not visible due to obstruction
4. **Optimize Draw Calls**: Batch similar objects and materials
5. **Use Appropriate Poly Counts**: Balance visual quality with performance

## Integration with Gazebo

### Step 17: Prepare for Gazebo Integration

1. Create a synchronization script that receives robot state from Gazebo via ROS 2
2. Implement interpolation to smooth state transitions
3. Set up proper coordinate system conversion
4. Configure timing synchronization between Unity and Gazebo

```csharp
// Example synchronization script
using UnityEngine;
using System.Collections.Generic;

public class GazeboUnitySync : MonoBehaviour
{
    public float syncFrequency = 60.0f; // Hz
    public bool enableInterpolation = true;
    public float interpolationTime = 0.1f;

    private float lastSyncTime = 0f;
    private Dictionary<string, TransformState> gazeboStates = new Dictionary<string, TransformState>();
    private Dictionary<string, TransformState> unityStates = new Dictionary<string, TransformState>();

    [System.Serializable]
    public class TransformState
    {
        public Vector3 position;
        public Quaternion rotation;
        public float timestamp;
    }

    void Update()
    {
        float currentTime = Time.time;
        if (currentTime - lastSyncTime >= 1.0f/syncFrequency)
        {
            SyncWithGazebo();
            lastSyncTime = currentTime;
        }
    }

    void SyncWithGazebo()
    {
        // This would receive state from Gazebo via ROS 2
        // For now, we'll simulate receiving state
        ReceiveStateFromGazebo();
        UpdateUnityObjects();
    }

    void ReceiveStateFromGazebo()
    {
        // In implementation, this receives state from ROS 2
        // For simulation:
        foreach (var robotName in GetRobotNames())
        {
            var state = GetSimulatedState(robotName);
            gazeboStates[robotName] = state;
        }
    }

    void UpdateUnityObjects()
    {
        foreach (var kvp in gazeboStates)
        {
            string robotName = kvp.Key;
            TransformState targetState = kvp.Value;

            GameObject robot = GameObject.Find(robotName);
            if (robot != null)
            {
                if (enableInterpolation)
                {
                    robot.transform.position = Vector3.Lerp(
                        robot.transform.position,
                        targetState.position,
                        Time.deltaTime / interpolationTime
                    );

                    robot.transform.rotation = Quaternion.Slerp(
                        robot.transform.rotation,
                        targetState.rotation,
                        Time.deltaTime / interpolationTime
                    );
                }
                else
                {
                    robot.transform.position = targetState.position;
                    robot.transform.rotation = targetState.rotation;
                }
            }
        }
    }

    List<string> GetRobotNames()
    {
        // Return list of robot names in the scene
        List<string> names = new List<string>();
        GameObject[] robots = GameObject.FindGameObjectsWithTag("Robot");
        foreach (GameObject robot in robots)
        {
            names.Add(robot.name);
        }
        return names;
    }

    TransformState GetSimulatedState(string robotName)
    {
        // Simulate receiving state from Gazebo
        float time = Time.time;
        return new TransformState
        {
            position = new Vector3(
                Mathf.Sin(time) * 2f,
                0.5f,
                Mathf.Cos(time) * 2f
            ),
            rotation = Quaternion.Euler(0, time * 45f, 0),
            timestamp = Time.time
        };
    }
}
```

## Next Steps

Congratulations! You've successfully set up Unity for robotics visualization. Now you can:

1. Continue with the ROS 2 integration tutorial
2. Create detailed robot models and environments
3. Implement physics synchronization with Gazebo
4. Develop human-robot interaction interfaces
5. Set up synthetic data generation pipelines

## Resources

- Unity Manual: https://docs.unity3d.com/Manual/index.html
- Unity Robotics Repository: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- ROS TCP Connector: https://github.com/Unity-Technologies/ROS-TCP-Connector
- Unity Learn Robotics: https://learn.unity.com/search?facets=category%3A%22xr-and-robotics%22

## Summary

This tutorial covered the complete setup of Unity for robotics applications, including:
- Unity installation and project configuration
- Robotics-specific package installation
- Coordinate system setup for robotics
- Rendering and performance optimization
- VR/AR setup for immersive HRI
- Testing and troubleshooting
- Integration preparation for Gazebo

You now have a Unity environment properly configured for robotics visualization and digital twin applications.