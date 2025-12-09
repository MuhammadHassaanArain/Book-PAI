# Lab 4: Visualize Robot in Unity and Simulate HRI

## Objective

In this lab, you will learn to visualize a robot in Unity, synchronize physics between Gazebo and Unity, and simulate human-robot interaction scenarios. You will create a Unity project that integrates with your Gazebo simulation and implements HRI interfaces.

## Prerequisites

- Basic Unity knowledge
- Completed previous labs
- Unity LTS installed
- ROS 2 bridge setup (rosbridge_suite)
- Basic understanding of VR/AR concepts

## Lab Tasks

### Task 1: Set Up Unity Project Structure

Create a Unity project with the proper structure for robotics visualization:

```csharp
// RobotVisualizationManager.cs
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class RobotVisualizationManager : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string robotName = "robot1";
    public GameObject robotPrefab;
    public Transform gazeboOrigin;
    public Transform unityOrigin;

    [Header("Synchronization Settings")]
    public float syncFrequency = 60.0f; // Hz
    public bool enableInterpolation = true;
    public float interpolationTime = 0.1f;

    [Header("Coordinate System")]
    public bool convertCoordinates = true;
    public Vector3 coordinateScale = new Vector3(1, 1, 1);

    private GameObject robotInstance;
    private Dictionary<string, Vector3> gazeboPositions = new Dictionary<string, Vector3>();
    private Dictionary<string, Quaternion> gazeboRotations = new Dictionary<string, Quaternion>();
    private Dictionary<string, Vector3> unityPositions = new Dictionary<string, Vector3>();
    private Dictionary<string, Quaternion> unityRotations = new Dictionary<string, Quaternion>();

    void Start()
    {
        InitializeRobotVisualization();
        StartCoroutine(SynchronizationLoop());
    }

    void InitializeRobotVisualization()
    {
        // Instantiate robot prefab
        if (robotPrefab != null)
        {
            robotInstance = Instantiate(robotPrefab, Vector3.zero, Quaternion.identity);
            robotInstance.name = robotName;

            // Set up initial transforms
            if (gazeboOrigin != null)
            {
                robotInstance.transform.position = ConvertGazeboToUnityPosition(gazeboOrigin.position);
                robotInstance.transform.rotation = ConvertGazeboToUnityRotation(gazeboOrigin.rotation);
            }
        }
        else
        {
            Debug.LogError("Robot prefab not assigned!");
        }
    }

    IEnumerator SynchronizationLoop()
    {
        float syncInterval = 1.0f / syncFrequency;

        while (true)
        {
            // In a real implementation, this would receive data from Gazebo via ROS
            ReceiveRobotStateFromGazebo();

            // Update Unity visualization
            UpdateRobotVisualization();

            yield return new WaitForSeconds(syncInterval);
        }
    }

    void ReceiveRobotStateFromGazebo()
    {
        // Simulate receiving robot state from Gazebo
        // In practice, this would connect to ROS via rosbridge or similar
        if (robotInstance != null)
        {
            // Simulate receiving position and rotation from Gazebo
            Vector3 gazeboPos = SimulateGazeboPosition();
            Quaternion gazeboRot = SimulateGazeboRotation();

            gazeboPositions[robotName] = gazeboPos;
            gazeboRotations[robotName] = gazeboRot;
        }
    }

    void UpdateRobotVisualization()
    {
        if (robotInstance == null || !gazeboPositions.ContainsKey(robotName)) return;

        Vector3 targetPosition = gazeboPositions[robotName];
        Quaternion targetRotation = gazeboRotations[robotName];

        if (convertCoordinates)
        {
            targetPosition = ConvertGazeboToUnityPosition(targetPosition);
            targetRotation = ConvertGazeboToUnityRotation(targetRotation);
        }

        if (enableInterpolation)
        {
            // Interpolate to target position
            robotInstance.transform.position = Vector3.Lerp(
                robotInstance.transform.position,
                targetPosition,
                Time.deltaTime / interpolationTime
            );

            robotInstance.transform.rotation = Quaternion.Slerp(
                robotInstance.transform.rotation,
                targetRotation,
                Time.deltaTime / interpolationTime
            );
        }
        else
        {
            // Direct assignment
            robotInstance.transform.position = targetPosition;
            robotInstance.transform.rotation = targetRotation;
        }

        // Store current Unity state
        unityPositions[robotName] = robotInstance.transform.position;
        unityRotations[robotName] = robotInstance.transform.rotation;
    }

    Vector3 ConvertGazeboToUnityPosition(Vector3 gazeboPos)
    {
        // Gazebo: X-forward, Y-left, Z-up
        // Unity: X-right, Y-up, Z-forward
        return new Vector3(-gazeboPos.y, gazeboPos.z, gazeboPos.x) * coordinateScale;
    }

    Quaternion ConvertGazeboToUnityRotation(Quaternion gazeboRot)
    {
        // Convert rotation quaternion from Gazebo to Unity coordinate system
        return new Quaternion(-gazeboRot.z, gazeboRot.x, -gazeboRot.y, gazeboRot.w);
    }

    Vector3 SimulateGazeboPosition()
    {
        // Simulate getting position from Gazebo
        // In reality, this would come from ROS topic
        float time = Time.time;
        return new Vector3(
            Mathf.Sin(time * 0.5f) * 2f,
            0.5f,
            Mathf.Cos(time * 0.5f) * 2f
        );
    }

    Quaternion SimulateGazeboRotation()
    {
        // Simulate getting rotation from Gazebo
        float time = Time.time;
        return Quaternion.Euler(
            Mathf.Sin(time) * 30f,
            time * 45f,
            Mathf.Cos(time * 1.5f) * 20f
        );
    }
}
```

### Task 2: Implement Physics Synchronization

Create a physics synchronization system between Gazebo and Unity:

```csharp
// PhysicsSynchronization.cs
using UnityEngine;
using System.Collections.Generic;

public class PhysicsSynchronization : MonoBehaviour
{
    [Header("Synchronization Configuration")]
    public float syncFrequency = 60.0f;
    public float maxSyncLatency = 0.1f;
    public bool enableInterpolation = true;
    public bool enableExtrapolation = true;

    [Header("Physics Objects")]
    public GameObject[] synchronizedObjects;
    public Transform gazeboOrigin;
    public Transform unityOrigin;

    [Header("Synchronization Settings")]
    public float positionThreshold = 0.01f;
    public float rotationThreshold = 0.01f;
    public float velocityThreshold = 0.01f;

    private Dictionary<string, PhysicsState> gazeboStates = new Dictionary<string, PhysicsState>();
    private Dictionary<string, PhysicsState> unityStates = new Dictionary<string, PhysicsState>();
    private Dictionary<string, PhysicsState> interpolatedStates = new Dictionary<string, PhysicsState>();

    [System.Serializable]
    public class PhysicsState
    {
        public Vector3 position = Vector3.zero;
        public Quaternion rotation = Quaternion.identity;
        public Vector3 velocity = Vector3.zero;
        public Vector3 angularVelocity = Vector3.zero;
        public float timestamp = 0f;

        public PhysicsState Clone()
        {
            return new PhysicsState
            {
                position = this.position,
                rotation = this.rotation,
                velocity = this.velocity,
                angularVelocity = this.angularVelocity,
                timestamp = this.timestamp
            };
        }
    }

    void Start()
    {
        InitializeSynchronizer();
        StartCoroutine(SynchronizationLoop());
    }

    void InitializeSynchronizer()
    {
        // Initialize state dictionaries
        foreach (GameObject obj in synchronizedObjects)
        {
            string objName = obj.name;
            gazeboStates[objName] = new PhysicsState();
            unityStates[objName] = new PhysicsState();
            interpolatedStates[objName] = new PhysicsState();

            // Store initial Unity state
            StoreUnityState(objName, obj);
        }

        // Set up coordinate system conversion
        SetupCoordinateConversion();
    }

    void SetupCoordinateConversion()
    {
        // Coordinate system conversion setup
        // Gazebo: X-forward, Y-left, Z-up
        // Unity: X-right, Y-up, Z-forward
    }

    IEnumerator SynchronizationLoop()
    {
        float syncInterval = 1.0f / syncFrequency;

        while (true)
        {
            // Receive states from Gazebo
            ReceiveGazeboStates();

            // Update Unity objects based on Gazebo states
            UpdateUnityObjects();

            // Send Unity states to Gazebo (if needed for bidirectional sync)
            SendUnityStates();

            yield return new WaitForSeconds(syncInterval);
        }
    }

    void ReceiveGazeboStates()
    {
        // In a real implementation, this would receive state data from Gazebo
        // via ROS, TCP, or other communication protocol
        foreach (GameObject obj in synchronizedObjects)
        {
            string objName = obj.name;

            // Simulate receiving state from Gazebo
            PhysicsState gazeboState = ReceiveStateFromGazebo(objName);
            if (gazeboState != null)
            {
                gazeboStates[objName] = gazeboState;

                if (enableInterpolation)
                {
                    // Calculate interpolated state if needed
                    CalculateInterpolatedState(objName, gazeboState);
                }
            }
        }
    }

    void UpdateUnityObjects()
    {
        foreach (GameObject obj in synchronizedObjects)
        {
            string objName = obj.name;

            if (gazeboStates.ContainsKey(objName))
            {
                PhysicsState targetState = enableInterpolation && interpolatedStates.ContainsKey(objName)
                    ? interpolatedStates[objName]
                    : gazeboStates[objName];

                if (ShouldUpdateObject(objName, targetState))
                {
                    ApplyStateToUnityObject(obj, targetState);
                }
            }
        }
    }

    void SendUnityStates()
    {
        // Send Unity states back to Gazebo for bidirectional synchronization
        foreach (GameObject obj in synchronizedObjects)
        {
            string objName = obj.name;
            PhysicsState unityState = GetUnityState(obj);

            // Send to Gazebo
            SendStateToGazebo(objName, unityState);
        }
    }

    PhysicsState ReceiveStateFromGazebo(string objectName)
    {
        // Simulate receiving state from Gazebo
        // In reality, this would interface with ROS/Gazebo communication
        PhysicsState state = new PhysicsState();

        // Get current state from a simulated Gazebo connection
        state.position = GetSimulatedGazeboPosition(objectName);
        state.rotation = GetSimulatedGazeboRotation(objectName);
        state.velocity = GetSimulatedGazeboVelocity(objectName);
        state.angularVelocity = GetSimulatedGazeboAngularVelocity(objectName);
        state.timestamp = Time.time;

        return state;
    }

    void SendStateToGazebo(string objectName, PhysicsState state)
    {
        // Simulate sending state to Gazebo
        // In reality, this would interface with ROS/Gazebo communication
        Debug.Log($"Sending state to Gazebo for {objectName}: pos={state.position}, rot={state.rotation.eulerAngles}");
    }

    void CalculateInterpolatedState(string objectName, PhysicsState gazeboState)
    {
        if (!unityStates.ContainsKey(objectName)) return;

        PhysicsState unityState = unityStates[objectName];
        float interpolationFactor = CalculateInterpolationFactor(unityState, gazeboState);

        PhysicsState interpolated = new PhysicsState
        {
            position = Vector3.Lerp(unityState.position, gazeboState.position, interpolationFactor),
            rotation = Quaternion.Slerp(unityState.rotation, gazeboState.rotation, interpolationFactor),
            velocity = Vector3.Lerp(unityState.velocity, gazeboState.velocity, interpolationFactor),
            angularVelocity = Vector3.Lerp(unityState.angularVelocity, gazeboState.angularVelocity, interpolationFactor),
            timestamp = Mathf.Lerp(unityState.timestamp, gazeboState.timestamp, interpolationFactor)
        };

        interpolatedStates[objectName] = interpolated;
    }

    float CalculateInterpolationFactor(PhysicsState unityState, PhysicsState gazeboState)
    {
        float timeDiff = gazeboState.timestamp - unityState.timestamp;
        float targetInterval = 1.0f / syncFrequency;

        if (timeDiff <= 0) return 1.0f; // Use latest state if no time difference
        if (timeDiff >= targetInterval) return 1.0f; // Use latest state if too far behind

        return timeDiff / targetInterval;
    }

    bool ShouldUpdateObject(string objectName, PhysicsState targetState)
    {
        if (!unityStates.ContainsKey(objectName)) return true;

        PhysicsState currentState = unityStates[objectName];

        // Check if position change is significant
        float posDiff = Vector3.Distance(currentState.position, targetState.position);
        if (posDiff > positionThreshold) return true;

        // Check if rotation change is significant
        float rotDiff = Quaternion.Angle(currentState.rotation, targetState.rotation);
        if (rotDiff > rotationThreshold) return true;

        return false;
    }

    void ApplyStateToUnityObject(GameObject obj, PhysicsState state)
    {
        // Apply position and rotation with coordinate system conversion
        obj.transform.position = ConvertGazeboToUnityPosition(state.position);
        obj.transform.rotation = ConvertGazeboToUnityRotation(state.rotation);

        // Apply velocity to Rigidbody if it exists
        Rigidbody rb = obj.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.velocity = ConvertGazeboToUnityVector(state.velocity);
            rb.angularVelocity = ConvertGazeboToUnityVector(state.angularVelocity);
        }

        // Store the applied state
        string objName = obj.name;
        unityStates[objName] = state;
    }

    PhysicsState GetUnityState(GameObject obj)
    {
        PhysicsState state = new PhysicsState
        {
            position = ConvertUnityToGazeboPosition(obj.transform.position),
            rotation = ConvertUnityToGazeboRotation(obj.transform.rotation),
            timestamp = Time.time
        };

        Rigidbody rb = obj.GetComponent<Rigidbody>();
        if (rb != null)
        {
            state.velocity = ConvertUnityToGazeboVector(rb.velocity);
            state.angularVelocity = ConvertUnityToGazeboVector(rb.angularVelocity);
        }

        return state;
    }

    void StoreUnityState(string objectName, GameObject obj)
    {
        if (unityStates.ContainsKey(objectName))
        {
            unityStates[objectName] = GetUnityState(obj);
        }
    }

    // Coordinate system conversion methods
    Vector3 ConvertGazeboToUnityPosition(Vector3 gazeboPos)
    {
        // Gazebo: X-forward, Y-left, Z-up
        // Unity: X-right, Y-up, Z-forward
        return new Vector3(-gazeboPos.y, gazeboPos.z, gazeboPos.x);
    }

    Vector3 ConvertUnityToGazeboPosition(Vector3 unityPos)
    {
        // Reverse conversion
        return new Vector3(unityPos.z, -unityPos.x, unityPos.y);
    }

    Quaternion ConvertGazeboToUnityRotation(Quaternion gazeboRot)
    {
        // Convert rotation quaternion from Gazebo to Unity coordinate system
        return new Quaternion(-gazeboRot.z, gazeboRot.x, -gazeboRot.y, gazeboRot.w);
    }

    Quaternion ConvertUnityToGazeboRotation(Quaternion unityRot)
    {
        // Reverse conversion
        return new Quaternion(-unityRot.y, unityRot.z, -unityRot.x, unityRot.w);
    }

    Vector3 ConvertGazeboToUnityVector(Vector3 gazeboVec)
    {
        // Convert vector from Gazebo to Unity coordinate system
        return new Vector3(-gazeboVec.y, gazeboVec.z, gazeboVec.x);
    }

    Vector3 ConvertUnityToGazeboVector(Vector3 unityVec)
    {
        // Convert vector from Unity to Gazebo coordinate system
        return new Vector3(unityVec.z, -unityVec.x, unityVec.y);
    }

    Vector3 GetSimulatedGazeboPosition(string objectName)
    {
        // Simulate getting position from Gazebo
        GameObject obj = FindGameObjectByName(objectName);
        if (obj != null)
        {
            // Simulate some movement for demonstration
            float time = Time.time;
            return new Vector3(
                Mathf.Sin(time) * 2f,
                1f,
                Mathf.Cos(time) * 2f
            );
        }
        return Vector3.zero;
    }

    Quaternion GetSimulatedGazeboRotation(string objectName)
    {
        // Simulate getting rotation from Gazebo
        float time = Time.time;
        return Quaternion.Euler(
            Mathf.Sin(time * 2) * 30f,
            time * 45f,
            Mathf.Cos(time * 1.5f) * 20f
        );
    }

    Vector3 GetSimulatedGazeboVelocity(string objectName)
    {
        // Simulate getting velocity from Gazebo
        float time = Time.time;
        return new Vector3(
            Mathf.Cos(time) * 0.5f,
            0f,
            -Mathf.Sin(time) * 0.5f
        );
    }

    Vector3 GetSimulatedGazeboAngularVelocity(string objectName)
    {
        // Simulate getting angular velocity from Gazebo
        return new Vector3(0.5f, 0.3f, 0.2f);
    }

    GameObject FindGameObjectByName(string name)
    {
        foreach (GameObject obj in synchronizedObjects)
        {
            if (obj.name == name)
                return obj;
        }
        return null;
    }
}
```

### Task 3: Implement Human-Robot Interaction Interface

Create an HRI interface for teleoperation and interaction:

```csharp
// HRIInterface.cs
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System.Collections;
using System.Collections.Generic;

public class HRIInterface : MonoBehaviour
{
    [Header("UI Configuration")]
    public Canvas hriCanvas;
    public GameObject controlPanel;
    public GameObject statusPanel;
    public GameObject commandPanel;

    [Header("Robot Status Elements")]
    public Text robotNameText;
    public Text robotStateText;
    public Text batteryLevelText;
    public Slider batterySlider;
    public Image robotStatusIndicator;

    [Header("Command Elements")]
    public Button moveForwardButton;
    public Button moveBackwardButton;
    public Button turnLeftButton;
    public Button turnRightButton;
    public Button stopButton;
    public Button emergencyStopButton;

    [Header("Interaction Settings")]
    public float commandTimeout = 5.0f;
    public bool enableHapticFeedback = true;
    public float hapticIntensity = 0.5f;

    private Dictionary<string, bool> commandStates = new Dictionary<string, bool>();
    private Coroutine activeCommandCoroutine;

    void Start()
    {
        InitializeUI();
        SetupEventHandlers();
        UpdateRobotStatus();
    }

    void InitializeUI()
    {
        if (hriCanvas == null)
        {
            hriCanvas = GetComponent<Canvas>();
        }

        if (hriCanvas == null)
        {
            hriCanvas = gameObject.AddComponent<Canvas>();
            hriCanvas.renderMode = RenderMode.ScreenSpaceOverlay;
        }

        SetupControlPanel();
        SetupStatusPanel();
        SetupCommandPanel();
    }

    void SetupControlPanel()
    {
        if (controlPanel == null) return;

        // Create control panel layout
        RectTransform rectTransform = controlPanel.GetComponent<RectTransform>();
        rectTransform.anchorMin = new Vector2(0.02f, 0.02f);
        rectTransform.anchorMax = new Vector2(0.3f, 0.3f);
        rectTransform.pivot = new Vector2(0, 1);
    }

    void SetupStatusPanel()
    {
        if (statusPanel == null) return;

        // Create status panel layout
        RectTransform rectTransform = statusPanel.GetComponent<RectTransform>();
        rectTransform.anchorMin = new Vector2(0.7f, 0.7f);
        rectTransform.anchorMax = new Vector2(0.98f, 0.98f);
        rectTransform.pivot = new Vector2(1, 1);
    }

    void SetupCommandPanel()
    {
        if (commandPanel == null) return;

        // Create command panel layout
        RectTransform rectTransform = commandPanel.GetComponent<RectTransform>();
        rectTransform.anchorMin = new Vector2(0.4f, 0.02f);
        rectTransform.anchorMax = new Vector2(0.6f, 0.3f);
        rectTransform.pivot = new Vector2(0.5f, 1);
    }

    void SetupEventHandlers()
    {
        if (moveForwardButton != null)
            moveForwardButton.onClick.AddListener(() => SendCommand("move_forward"));

        if (moveBackwardButton != null)
            moveBackwardButton.onClick.AddListener(() => SendCommand("move_backward"));

        if (turnLeftButton != null)
            turnLeftButton.onClick.AddListener(() => SendCommand("turn_left"));

        if (turnRightButton != null)
            turnRightButton.onClick.AddListener(() => SendCommand("turn_right"));

        if (stopButton != null)
            stopButton.onClick.AddListener(() => SendCommand("stop"));

        if (emergencyStopButton != null)
            emergencyStopButton.onClick.AddListener(() => SendCommand("emergency_stop"));
    }

    public void SendCommand(string command)
    {
        if (commandStates.ContainsKey(command) && commandStates[command])
        {
            Debug.Log($"Command '{command}' already active, ignoring");
            return;
        }

        commandStates[command] = true;

        if (activeCommandCoroutine != null)
        {
            StopCoroutine(activeCommandCoroutine);
        }

        activeCommandCoroutine = StartCoroutine(ExecuteCommandWithTimeout(command));
    }

    IEnumerator ExecuteCommandWithTimeout(string command)
    {
        // Visual feedback for command execution
        HighlightActiveCommand(command, true);

        // Send command to robot (this would interface with ROS/robot control)
        bool success = SendCommandToRobot(command);

        if (success)
        {
            yield return new WaitForSeconds(commandTimeout);
        }
        else
        {
            Debug.LogError($"Command '{command}' failed to execute");
        }

        // Reset command state
        if (commandStates.ContainsKey(command))
        {
            commandStates[command] = false;
        }

        HighlightActiveCommand(command, false);
    }

    bool SendCommandToRobot(string command)
    {
        // In a real implementation, this would send the command to the robot
        // via ROS, TCP, or other communication protocol
        Debug.Log($"Sending command to robot: {command}");

        // Simulate command success
        return true;
    }

    void HighlightActiveCommand(string command, bool active)
    {
        Button button = null;

        switch (command)
        {
            case "move_forward":
                button = moveForwardButton;
                break;
            case "move_backward":
                button = moveBackwardButton;
                break;
            case "turn_left":
                button = turnLeftButton;
                break;
            case "turn_right":
                button = turnRightButton;
                break;
            case "stop":
                button = stopButton;
                break;
            case "emergency_stop":
                button = emergencyStopButton;
                break;
        }

        if (button != null)
        {
            ColorBlock colors = button.colors;
            colors.pressedColor = active ? Color.red : colors.normalColor;
            button.colors = colors;
        }
    }

    public void UpdateRobotStatus(string robotName = "Robot1", string state = "Ready", float batteryLevel = 85.5f)
    {
        if (robotNameText != null)
            robotNameText.text = $"Robot: {robotName}";

        if (robotStateText != null)
            robotStateText.text = $"State: {state}";

        if (batteryLevelText != null)
            batteryLevelText.text = $"Battery: {batteryLevel:F1}%";

        if (batterySlider != null)
            batterySlider.value = batteryLevel / 100f;

        if (robotStatusIndicator != null)
        {
            robotStatusIndicator.color = state.ToLower() == "error" ? Color.red : Color.green;
        }
    }

    void UpdateRobotStatus()
    {
        // Simulate updating robot status
        UpdateRobotStatus();
    }

    void Update()
    {
        // Update status periodically
        if (Time.time % 2.0f < Time.deltaTime) // Update every 2 seconds
        {
            UpdateRobotStatus();
        }
    }
}
```

### Task 4: Create Synthetic Data Generation System

Implement a system for generating synthetic data in Unity:

```csharp
// SyntheticDataManager.cs
using UnityEngine;
using System.Collections;
using System.IO;
using System.Collections.Generic;

public class SyntheticDataManager : MonoBehaviour
{
    [Header("Synthetic Data Configuration")]
    public string outputDirectory = "SyntheticData";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public int antiAliasing = 1;
    public bool generateRGB = true;
    public bool generateDepth = true;
    public bool generateSegmentation = true;
    public bool generateNormals = true;

    [Header("Dataset Configuration")]
    public int numberOfScenes = 1000;
    public int imagesPerScene = 10;
    public float sceneVariationInterval = 5.0f;

    [Header("Object Configuration")]
    public GameObject[] objectsToRandomize;
    public Material[] materialsToApply;
    public Light[] lightsToAdjust;
    public Color[] segmentationColors;

    private RenderTexture rgbTexture;
    private RenderTexture depthTexture;
    private RenderTexture segmentationTexture;
    private RenderTexture normalTexture;

    void Start()
    {
        CreateRenderTextures();
        StartCoroutine(GenerateDataset());
    }

    void CreateRenderTextures()
    {
        // Create RGB texture
        if (generateRGB)
        {
            rgbTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
            rgbTexture.antiAliasing = antiAliasing;
            rgbTexture.Create();
        }

        // Create depth texture
        if (generateDepth)
        {
            depthTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.RFloat);
            depthTexture.Create();
        }

        // Create segmentation texture
        if (generateSegmentation)
        {
            segmentationTexture = new RenderTexture(imageWidth, imageHeight, 0, RenderTextureFormat.ARGB32);
            segmentationTexture.Create();
        }

        // Create normal texture
        if (generateNormals)
        {
            normalTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
            normalTexture.Create();
        }
    }

    IEnumerator GenerateDataset()
    {
        for (int scene = 0; scene < numberOfScenes; scene++)
        {
            // Randomize scene configuration
            RandomizeSceneConfiguration();

            for (int img = 0; img < imagesPerScene; img++)
            {
                // Move camera to random position
                MoveCameraToRandomPosition();

                // Capture all data types
                if (generateRGB)
                    CaptureRGBImage(scene, img);

                if (generateDepth)
                    CaptureDepthImage(scene, img);

                if (generateSegmentation)
                    CaptureSegmentationMask(scene, img);

                if (generateNormals)
                    CaptureNormalMap(scene, img);

                // Save metadata
                SaveMetadata(scene, img);

                yield return null; // Wait for next frame
            }

            // Randomize scene after interval
            yield return new WaitForSeconds(sceneVariationInterval);
        }

        Debug.Log("Dataset generation completed!");
    }

    void RandomizeSceneConfiguration()
    {
        // Randomize object positions
        foreach (var obj in objectsToRandomize)
        {
            Vector3 randomPos = new Vector3(
                Random.Range(-10f, 10f),
                Random.Range(0.1f, 5f),
                Random.Range(-10f, 10f)
            );
            obj.transform.position = randomPos;

            // Randomize object rotation
            obj.transform.rotation = Quaternion.Euler(
                Random.Range(0f, 360f),
                Random.Range(0f, 360f),
                Random.Range(0f, 360f)
            );

            // Randomize material
            Renderer renderer = obj.GetComponent<Renderer>();
            if (renderer != null && materialsToApply.Length > 0)
            {
                Material randomMaterial = materialsToApply[Random.Range(0, materialsToApply.Length)];
                renderer.material = randomMaterial;
            }
        }

        // Randomize lighting
        foreach (var light in lightsToAdjust)
        {
            light.intensity = Random.Range(0.5f, 2f);
            light.color = Random.ColorHSV(0f, 1f, 0.5f, 1f, 0.5f, 1f);

            // Randomize light position and rotation
            light.transform.position = new Vector3(
                Random.Range(-5f, 5f),
                Random.Range(3f, 8f),
                Random.Range(-5f, 5f)
            );
        }
    }

    void MoveCameraToRandomPosition()
    {
        Vector3 randomOffset = new Vector3(
            Random.Range(-8f, 8f),
            Random.Range(1f, 6f),
            Random.Range(-8f, 8f)
        );
        transform.position = randomOffset;

        // Look at a random point in the scene
        Vector3 lookTarget = new Vector3(
            Random.Range(-3f, 3f),
            Random.Range(1f, 3f),
            Random.Range(-3f, 3f)
        );
        transform.LookAt(lookTarget);
    }

    void CaptureRGBImage(int scene, int img)
    {
        string filename = Path.Combine(
            outputDirectory,
            "rgb",
            $"scene_{scene:0000}_img_{img:0000}.png"
        );

        CaptureCameraImage(Camera.main, filename);
    }

    void CaptureDepthImage(int scene, int img)
    {
        string filename = Path.Combine(
            outputDirectory,
            "depth",
            $"scene_{scene:0000}_img_{img:0000}.exr"
        );

        CaptureDepthData(Camera.main, filename);
    }

    void CaptureSegmentationMask(int scene, int img)
    {
        string filename = Path.Combine(
            outputDirectory,
            "segmentation",
            $"scene_{scene:0000}_img_{img:0000}.png"
        );

        CaptureSegmentationData(Camera.main, filename);
    }

    void CaptureNormalMap(int scene, int img)
    {
        string filename = Path.Combine(
            outputDirectory,
            "normals",
            $"scene_{scene:0000}_img_{img:0000}.png"
        );

        CaptureNormalData(Camera.main, filename);
    }

    void CaptureCameraImage(Camera cam, string filename)
    {
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;

        cam.Render();

        Texture2D image = new Texture2D(cam.targetTexture.width, cam.targetTexture.height, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        image.Apply();

        RenderTexture.active = currentRT;

        // Create directory if it doesn't exist
        Directory.CreateDirectory(Path.GetDirectoryName(filename));

        // Save image
        byte[] bytes = image.EncodeToPNG();
        File.WriteAllBytes(filename, bytes);

        DestroyImmediate(image);
    }

    void CaptureDepthData(Camera cam, string filename)
    {
        // Render depth data
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;
        cam.Render();

        Texture2D depthTexture2D = new Texture2D(cam.targetTexture.width, cam.targetTexture.height, TextureFormat.RFloat, false);
        depthTexture2D.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        depthTexture2D.Apply();

        RenderTexture.active = currentRT;

        // Convert to proper depth format and save
        Directory.CreateDirectory(Path.GetDirectoryName(filename));

        // For depth data, we typically save as EXR for higher precision
        byte[] bytes = depthTexture2D.EncodeToEXR();
        File.WriteAllBytes(filename, bytes);

        DestroyImmediate(depthTexture2D);
    }

    void CaptureSegmentationData(Camera cam, string filename)
    {
        // For segmentation, we render objects with unique colors
        // This requires special shaders or materials for each object class
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;
        cam.Render();

        Texture2D segmentationTexture2D = new Texture2D(cam.targetTexture.width, cam.targetTexture.height, TextureFormat.RGB24, false);
        segmentationTexture2D.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        segmentationTexture2D.Apply();

        RenderTexture.active = currentRT;

        Directory.CreateDirectory(Path.GetDirectoryName(filename));

        byte[] bytes = segmentationTexture2D.EncodeToPNG();
        File.WriteAllBytes(filename, bytes);

        DestroyImmediate(segmentationTexture2D);
    }

    void CaptureNormalData(Camera cam, string filename)
    {
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;
        cam.Render();

        Texture2D normalTexture2D = new Texture2D(cam.targetTexture.width, cam.targetTexture.height, TextureFormat.RGB24, false);
        normalTexture2D.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        normalTexture2D.Apply();

        RenderTexture.active = currentRT;

        Directory.CreateDirectory(Path.GetDirectoryName(filename));

        byte[] bytes = normalTexture2D.EncodeToPNG();
        File.WriteAllBytes(filename, bytes);

        DestroyImmediate(normalTexture2D);
    }

    void SaveMetadata(int scene, int img)
    {
        string metadataPath = Path.Combine(
            outputDirectory,
            "metadata",
            $"scene_{scene:0000}_img_{img:0000}.json"
        );

        Directory.CreateDirectory(Path.GetDirectoryName(metadataPath));

        // Create metadata object
        SyntheticDataMetadata metadata = new SyntheticDataMetadata
        {
            sceneId = scene,
            imageId = img,
            timestamp = System.DateTime.UtcNow.ToString("o"),
            cameraPosition = transform.position,
            cameraRotation = transform.rotation.eulerAngles,
            objects = GetObjectMetadata()
        };

        string json = JsonUtility.ToJson(metadata, true);
        File.WriteAllText(metadataPath, json);
    }

    ObjectMetadata[] GetObjectMetadata()
    {
        ObjectMetadata[] objects = new ObjectMetadata[objectsToRandomize.Length];

        for (int i = 0; i < objectsToRandomize.Length; i++)
        {
            GameObject obj = objectsToRandomize[i];
            objects[i] = new ObjectMetadata
            {
                name = obj.name,
                position = obj.transform.position,
                rotation = obj.transform.rotation.eulerAngles,
                scale = obj.transform.localScale,
                material = obj.GetComponent<Renderer>()?.material?.name ?? "Unknown"
            };
        }

        return objects;
    }
}

[System.Serializable]
public class SyntheticDataMetadata
{
    public int sceneId;
    public int imageId;
    public string timestamp;
    public Vector3 cameraPosition;
    public Vector3 cameraRotation;
    public ObjectMetadata[] objects;
}

[System.Serializable]
public class ObjectMetadata
{
    public string name;
    public Vector3 position;
    public Vector3 rotation;
    public Vector3 scale;
    public string material;
}
```

## Lab Exercises

1. **Robot Visualization**: Implement a complete robot model visualization in Unity
2. **Physics Synchronization**: Create a stable synchronization system between Gazebo and Unity
3. **HRI Interface**: Design and implement an intuitive human-robot interaction interface
4. **Synthetic Data**: Generate a synthetic dataset with proper annotations
5. **VR Integration**: Extend the visualization to VR for immersive HRI

## Validation Steps

1. Set up the Unity project with proper robotics integration
2. Verify physics synchronization accuracy
3. Test HRI interface responsiveness
4. Validate synthetic data quality
5. Ensure stable performance across all systems

## Expected Outcomes

- Functional Unity visualization of Gazebo robot
- Stable physics synchronization
- Intuitive HRI interface
- Synthetic data generation pipeline
- Integrated system ready for HRI testing

## References

1. Unity Technologies. (2023). Unity Robotics Integration. Unity Technologies.
2. Open Source Robotics Foundation. (2023). Gazebo to Unity Bridge. http://gazebosim.org/tutorials?tut=ros_unity_bridge
3. ROS-Industrial Consortium. (2023). Unity Integration Best Practices. https://ros-industrial.github.io/industrial_training/