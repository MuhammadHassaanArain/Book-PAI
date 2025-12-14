# HRI Simulation Tutorial

## Overview

This tutorial covers the implementation of Human-Robot Interaction (HRI) simulation in Unity integrated with Gazebo physics. We'll explore creating realistic human-robot interaction scenarios, implementing intuitive interfaces, and validating HRI systems in digital twin environments. The focus is on creating immersive, safe, and effective human-robot collaboration systems.

## Introduction to Human-Robot Interaction

### HRI Fundamentals

Human-Robot Interaction encompasses the design, development, and evaluation of robots that interact with humans. Key aspects include:

1. **Safety**: Ensuring human safety during interaction
2. **Intuitiveness**: Making interactions natural and predictable
3. **Effectiveness**: Achieving task goals efficiently
4. **Trust**: Building confidence in robot capabilities
5. **Acceptance**: Making robots socially acceptable

### HRI Design Principles

Effective HRI systems follow these principles:

1. **Transparency**: Robots should clearly communicate their intentions
2. **Predictability**: Robot behavior should be consistent and understandable
3. **Responsiveness**: Robots should react appropriately to human actions
4. **Adaptability**: Systems should adapt to different users and contexts
5. **Safety**: Interaction should never compromise human safety

## Unity Implementation

### Setting Up HRI Environment

Create the basic HRI scene structure:

```csharp
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.InputSystem;
using System.Collections;
using System.Collections.Generic;

public class HRISystemManager : MonoBehaviour
{
    [Header("HRI Configuration")]
    public float interactionDistance = 2.0f;  // Maximum interaction distance
    public float safetyDistance = 0.5f;       // Minimum safe distance
    public float attentionSpan = 5.0f;        // How long robot pays attention
    public bool enableHapticFeedback = true;
    public float hapticIntensity = 0.5f;

    [Header("Human Detection")]
    public LayerMask humanLayer = 1 << 9;     // Human layer
    public float detectionRadius = 5.0f;
    public float detectionUpdateRate = 10.0f; // Hz

    [Header("Interaction Modes")]
    public bool enableVoiceCommands = true;
    public bool enableGestureRecognition = true;
    public bool enableTouchInteraction = true;
    public bool enableProximityDetection = true;

    [Header("Safety Parameters")]
    public float maximumApproachSpeed = 0.5f; // m/s
    public float emergencyStopDistance = 0.3f; // m
    public bool enableCollisionAvoidance = true;

    // Private variables
    private List<GameObject> detectedHumans = new List<GameObject>();
    private Dictionary<GameObject, float> attentionTimers = new Dictionary<GameObject, float>();
    private Dictionary<GameObject, bool> isAttending = new Dictionary<GameObject, bool>();
    private GameObject currentInteractionTarget = null;
    private float lastDetectionTime = 0f;
    private bool isInteracting = false;

    void Start()
    {
        InitializeHRIEnvironment();
        StartCoroutine(HRIDetectionLoop());
        StartCoroutine(InteractionManagementLoop());
    }

    void InitializeHRIEnvironment()
    {
        // Initialize HRI systems
        SetupInteractionZones();
        ConfigureSafetySystems();
        InitializeSensors();

        Debug.Log("HRI System initialized successfully");
    }

    void SetupInteractionZones()
    {
        // Create visualization for interaction zones
        GameObject interactionZone = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        interactionZone.name = "InteractionZone";
        interactionZone.transform.SetParent(transform);
        interactionZone.transform.localScale = new Vector3(interactionDistance * 2, 0.1f, interactionDistance * 2);
        interactionZone.transform.localPosition = Vector3.zero;
        interactionZone.transform.localRotation = Quaternion.Euler(90, 0, 0);

        Renderer zoneRenderer = interactionZone.GetComponent<Renderer>();
        if (zoneRenderer != null)
        {
            zoneRenderer.material.color = new Color(0, 1, 0, 0.2f); // Semi-transparent green
            zoneRenderer.enabled = true;
        }

        // Make it non-interactive
        Destroy(interactionZone.GetComponent<Collider>());
    }

    void ConfigureSafetySystems()
    {
        // Setup safety monitoring
        SafetySystem safetySystem = GetComponent<SafetySystem>();
        if (safetySystem != null)
        {
            safetySystem.Initialize(safetyDistance, emergencyStopDistance, maximumApproachSpeed);
        }
    }

    void InitializeSensors()
    {
        // Initialize proximity sensors, cameras, etc.
        // This would interface with actual robot sensors in real implementation
    }

    IEnumerator HRIDetectionLoop()
    {
        float detectionInterval = 1.0f / detectionUpdateRate;

        while (true)
        {
            DetectHumans();
            UpdateSafetySystems();

            yield return new WaitForSeconds(detectionInterval);
        }
    }

    void DetectHumans()
    {
        // Use sphere casting to detect humans
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, detectionRadius, humanLayer);

        detectedHumans.Clear();
        foreach (Collider collider in hitColliders)
        {
            GameObject human = collider.gameObject;
            detectedHumans.Add(human);

            // Update attention tracking
            if (!attentionTimers.ContainsKey(human))
            {
                attentionTimers[human] = 0f;
                isAttending[human] = false;
            }

            // Check distance and react appropriately
            float distance = Vector3.Distance(transform.position, human.transform.position);

            if (distance <= interactionDistance)
            {
                if (!isAttending[human])
                {
                    StartAttentionToHuman(human);
                }

                attentionTimers[human] = attentionSpan; // Renew attention

                // React to human presence
                ReactToHumanPresence(human, distance);
            }
            else if (isAttending.ContainsKey(human) && isAttending[human])
            {
                EndAttentionToHuman(human);
            }
        }

        // Decrease attention timers
        List<GameObject> humansToRemove = new List<GameObject>();
        foreach (var kvp in attentionTimers)
        {
            kvp.Value -= Time.deltaTime / detectionInterval;
            if (kvp.Value <= 0)
            {
                humansToRemove.Add(kvp.Key);
            }
        }

        foreach (GameObject human in humansToRemove)
        {
            EndAttentionToHuman(human);
            attentionTimers.Remove(human);
            isAttending.Remove(human);
        }
    }

    void StartAttentionToHuman(GameObject human)
    {
        isAttending[human] = true;

        // Play attention animation
        AnimateAttentionStart(human);

        // Provide feedback to human
        ProvideVisualFeedback(human, "attention_acquired");

        Debug.Log($"Started attention to human: {human.name}");
    }

    void EndAttentionToHuman(GameObject human)
    {
        isAttending[human] = false;

        // Play attention end animation
        AnimateAttentionEnd(human);

        // Check if this was the interaction target
        if (currentInteractionTarget == human)
        {
            EndInteraction();
        }

        Debug.Log($"Ended attention to human: {human.name}");
    }

    void ReactToHumanPresence(GameObject human, float distance)
    {
        // Determine appropriate reaction based on distance
        if (distance < safetyDistance)
        {
            // Too close - trigger safety response
            TriggerSafetyResponse(human);
        }
        else if (distance < interactionDistance * 0.7f) // 70% of interaction distance
        {
            // Close enough for interaction
            if (currentInteractionTarget == null)
            {
                StartInteraction(human);
            }
        }
        else if (distance > interactionDistance * 0.8f && currentInteractionTarget == human)
        {
            // Moved away from interaction distance
            EndInteraction();
        }
    }

    void StartInteraction(GameObject human)
    {
        if (isInteracting) return; // Already interacting

        currentInteractionTarget = human;
        isInteracting = true;

        // Notify interaction start
        OnInteractionStarted(human);

        // Play interaction start animation
        PlayInteractionAnimation("greeting");

        Debug.Log($"Started interaction with: {human.name}");
    }

    void EndInteraction()
    {
        if (!isInteracting) return;

        // Notify interaction end
        OnInteractionEnded(currentInteractionTarget);

        // Play interaction end animation
        PlayInteractionAnimation("farewell");

        currentInteractionTarget = null;
        isInteracting = false;

        Debug.Log($"Ended interaction with: {currentInteractionTarget?.name}");
    }

    void OnInteractionStarted(GameObject human)
    {
        // Event handler for interaction start
        // This could trigger state changes, UI updates, etc.

        // Example: Show interaction UI
        ShowInteractionUI(human);

        // Example: Start listening for commands
        if (enableVoiceCommands)
        {
            StartVoiceRecognition();
        }

        if (enableGestureRecognition)
        {
            StartGestureRecognition();
        }
    }

    void OnInteractionEnded(GameObject human)
    {
        // Event handler for interaction end
        // Clean up interaction resources

        // Example: Hide interaction UI
        HideInteractionUI();

        // Example: Stop listening for commands
        StopVoiceRecognition();
        StopGestureRecognition();
    }

    void TriggerSafetyResponse(GameObject human)
    {
        // Trigger safety protocols
        SafetySystem safetySystem = GetComponent<SafetySystem>();
        if (safetySystem != null)
        {
            safetySystem.TriggerSafetyProtocol(human);
        }

        // Emergency stop if needed
        EmergencyStop();

        Debug.LogWarning($"Safety triggered: Human {human.name} too close!");
    }

    void UpdateSafetySystems()
    {
        // Update safety monitoring
        SafetySystem safetySystem = GetComponent<SafetySystem>();
        if (safetySystem != null)
        {
            safetySystem.UpdateSafetyStatus(detectedHumans);
        }
    }

    void PlayInteractionAnimation(string animationType)
    {
        // Play appropriate animation based on interaction type
        // This would interface with Unity's animation system
        Animator animator = GetComponent<Animator>();
        if (animator != null)
        {
            animator.SetTrigger(animationType);
        }
    }

    void ProvideVisualFeedback(GameObject human, string feedbackType)
    {
        // Provide visual feedback to human
        switch (feedbackType)
        {
            case "attention_acquired":
                // Flash attention indicator
                StartCoroutine(FlashAttentionIndicator());
                break;
            case "interaction_ready":
                // Show interaction ready indicator
                ShowInteractionIndicator(true);
                break;
        }
    }

    IEnumerator FlashAttentionIndicator()
    {
        // Example: Flash a light or change material color
        Renderer renderer = GetComponent<Renderer>();
        if (renderer != null)
        {
            Color originalColor = renderer.material.color;
            Color attentionColor = Color.blue;

            for (int i = 0; i < 3; i++)
            {
                renderer.material.color = attentionColor;
                yield return new WaitForSeconds(0.2f);
                renderer.material.color = originalColor;
                yield return new WaitForSeconds(0.2f);
            }
        }
    }

    void ShowInteractionIndicator(bool show)
    {
        // Show or hide interaction indicator
        GameObject indicator = GameObject.Find("InteractionIndicator");
        if (indicator != null)
        {
            indicator.SetActive(show);
        }
    }

    void ShowInteractionUI(GameObject human)
    {
        // Show interaction UI elements
        GameObject interactionUI = GameObject.Find("InteractionUI");
        if (interactionUI != null)
        {
            interactionUI.SetActive(true);

            // Update UI with human information
            UpdateInteractionUI(human);
        }
    }

    void HideInteractionUI()
    {
        GameObject interactionUI = GameObject.Find("InteractionUI");
        if (interactionUI != null)
        {
            interactionUI.SetActive(false);
        }
    }

    void UpdateInteractionUI(GameObject human)
    {
        // Update UI with relevant information
        // This would update text, buttons, etc. based on the human
    }

    void StartVoiceRecognition()
    {
        // Initialize voice recognition system
        VoiceRecognitionSystem voiceSystem = GetComponent<VoiceRecognitionSystem>();
        if (voiceSystem != null)
        {
            voiceSystem.StartListening();
        }
    }

    void StopVoiceRecognition()
    {
        VoiceRecognitionSystem voiceSystem = GetComponent<VoiceRecognitionSystem>();
        if (voiceSystem != null)
        {
            voiceSystem.StopListening();
        }
    }

    void StartGestureRecognition()
    {
        // Initialize gesture recognition system
        GestureRecognitionSystem gestureSystem = GetComponent<GestureRecognitionSystem>();
        if (gestureSystem != null)
        {
            gestureSystem.StartRecognition();
        }
    }

    void StopGestureRecognition()
    {
        GestureRecognitionSystem gestureSystem = GetComponent<GestureRecognitionSystem>();
        if (gestureSystem != null)
        {
            gestureSystem.StopRecognition();
        }
    }

    void EmergencyStop()
    {
        // Implement emergency stop procedure
        // This would stop all robot motion and activate safety protocols
        Debug.LogWarning("Emergency stop activated!");
    }

    void OnDrawGizmosSelected()
    {
        // Draw interaction and safety zones in editor
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, safetyDistance);

        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, interactionDistance);

        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(transform.position, detectionRadius);
    }
}
```

### Human Controller Implementation

Create a human controller for realistic human behavior:

```csharp
using UnityEngine;
using UnityEngine.AI;
using System.Collections;

public class HumanController : MonoBehaviour
{
    [Header("Movement Configuration")]
    public float walkingSpeed = 1.2f;
    public float runningSpeed = 3.0f;
    public float turnSpeed = 120f;
    public float stoppingDistance = 0.5f;

    [Header("Behavior Configuration")]
    public float attentionSpan = 5.0f;  // How long to pay attention to robot
    public float reactionTime = 0.2f;   // Reaction time to robot actions
    public bool isIdleWhenNotInteracting = true;

    [Header("Interaction Preferences")]
    public float preferredInteractionDistance = 1.0f;  // Preferred distance to robot
    public float comfortZoneRadius = 0.8f;  // Comfort zone around human
    public bool prefersFaceToFace = true;  // Prefers facing robot

    [Header("Visual Feedback")]
    public GameObject eyeIndicator;  // Visual indicator for attention
    public GameObject speechBubble;  // Speech bubble for communication

    // Private variables
    private NavMeshAgent navAgent;
    private Animator animator;
    private float attentionTimer = 0f;
    private bool isAttendingToRobot = false;
    private Transform targetRobot;
    private Vector3 originalPosition;
    private Quaternion originalRotation;

    void Start()
    {
        InitializeHumanController();
        StoreOriginalPosition();
    }

    void InitializeHumanController()
    {
        // Get required components
        navAgent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        if (navAgent != null)
        {
            navAgent.speed = walkingSpeed;
            navAgent.stoppingDistance = stoppingDistance;
        }

        // Initialize state
        attentionTimer = attentionSpan;
        isAttendingToRobot = false;

        Debug.Log($"Human controller {gameObject.name} initialized");
    }

    void StoreOriginalPosition()
    {
        originalPosition = transform.position;
        originalRotation = transform.rotation;
    }

    void Update()
    {
        UpdateAttentionSystem();
        UpdateBehavior();
    }

    void UpdateAttentionSystem()
    {
        // Update attention timer
        if (isAttendingToRobot)
        {
            attentionTimer -= Time.deltaTime;
            if (attentionTimer <= 0)
            {
                EndAttentionToRobot();
            }
        }
    }

    void UpdateBehavior()
    {
        // Update human behavior based on interaction state
        if (isAttendingToRobot && targetRobot != null)
        {
            MaintainInteractionPosture();
        }
        else if (isIdleWhenNotInteracting)
        {
            MaintainIdleBehavior();
        }
    }

    public void AttendToRobot(Transform robot, float duration = -1)
    {
        // Start attending to robot
        targetRobot = robot;
        isAttendingToRobot = true;
        attentionTimer = duration > 0 ? duration : attentionSpan;

        // Play attention animation
        if (animator != null)
        {
            animator.SetBool("IsAttending", true);
            animator.SetFloat("AttentionIntensity", 1.0f);
        }

        // Face the robot if preferred
        if (prefersFaceToFace && targetRobot != null)
        {
            StartCoroutine(DelayedLookAtRobot());
        }

        Debug.Log($"{gameObject.name} is now attending to robot");
    }

    IEnumerator DelayedLookAtRobot()
    {
        // Wait for reaction time before turning
        yield return new WaitForSeconds(reactionTime);

        if (targetRobot != null)
        {
            Vector3 direction = (targetRobot.position - transform.position).normalized;
            direction.y = 0; // Keep rotation on horizontal plane
            Quaternion targetRotation = Quaternion.LookRotation(direction);
            transform.rotation = Quaternion.RotateTowards(
                transform.rotation,
                targetRotation,
                turnSpeed * Time.deltaTime
            );
        }
    }

    public void EndAttentionToRobot()
    {
        // End attention to robot
        isAttendingToRobot = false;
        targetRobot = null;
        attentionTimer = attentionSpan;

        // Stop attention animation
        if (animator != null)
        {
            animator.SetBool("IsAttending", false);
            animator.SetFloat("AttentionIntensity", 0.0f);
        }

        Debug.Log($"{gameObject.name} ended attention to robot");
    }

    void MaintainInteractionPosture()
    {
        // Maintain appropriate posture during interaction
        if (targetRobot != null)
        {
            // Adjust position to maintain preferred distance
            float currentDistance = Vector3.Distance(transform.position, targetRobot.position);
            float targetDistance = preferredInteractionDistance;

            if (Mathf.Abs(currentDistance - targetDistance) > 0.2f)
            {
                // Adjust position
                Vector3 direction = (targetRobot.position - transform.position).normalized;
                Vector3 targetPosition = targetRobot.position - direction * targetDistance;

                if (navAgent != null)
                {
                    navAgent.SetDestination(targetPosition);
                }
            }

            // Maintain face-to-face orientation if preferred
            if (prefersFaceToFace)
            {
                Vector3 lookDirection = (targetRobot.position - transform.position).normalized;
                lookDirection.y = 0; // Keep rotation on horizontal plane
                Quaternion targetRotation = Quaternion.LookRotation(lookDirection);
                transform.rotation = Quaternion.RotateTowards(
                    transform.rotation,
                    targetRotation,
                    turnSpeed * Time.deltaTime
                );
            }
        }
    }

    void MaintainIdleBehavior()
    {
        // Maintain idle behavior when not interacting
        if (animator != null)
        {
            // Play idle animations or random movements
            if (Random.value < 0.01f) // Small chance per frame for idle movement
            {
                PlayIdleMovement();
            }
        }
    }

    void PlayIdleMovement()
    {
        // Play small idle movements
        if (animator != null)
        {
            int idleMove = Random.Range(0, 3);
            switch (idleMove)
            {
                case 0:
                    animator.SetTrigger("IdleGlance");
                    break;
                case 1:
                    animator.SetTrigger("IdleShiftWeight");
                    break;
                case 2:
                    animator.SetTrigger("IdleAdjustClothing");
                    break;
            }
        }
    }

    public void ReactToRobotAction(string action)
    {
        // React to robot's actions
        if (animator != null)
        {
            switch (action)
            {
                case "approach":
                    animator.SetTrigger("Surprised");
                    break;
                case "wave":
                    StartCoroutine(DelayedWaveResponse());
                    break;
                case "stop":
                    animator.SetTrigger("Acknowledge");
                    break;
                case "greet":
                    StartCoroutine(DelayedGreetingResponse());
                    break;
                default:
                    animator.SetTrigger("Notice");
                    break;
            }
        }
    }

    IEnumerator DelayedWaveResponse()
    {
        yield return new WaitForSeconds(reactionTime);
        if (animator != null)
        {
            animator.SetTrigger("WaveBack");
        }
    }

    IEnumerator DelayedGreetingResponse()
    {
        yield return new WaitForSeconds(reactionTime);
        if (animator != null)
        {
            animator.SetTrigger("GreetBack");
        }
    }

    public bool IsInComfortZone(Vector3 position)
    {
        // Check if a position is within human's comfort zone
        float distance = Vector3.Distance(position, transform.position);
        return distance <= comfortZoneRadius;
    }

    public float GetDistanceToRobot(Transform robot)
    {
        if (robot != null)
        {
            return Vector3.Distance(transform.position, robot.position);
        }
        return float.MaxValue;
    }

    public Vector3 GetPreferredInteractionPosition(Transform robot)
    {
        // Calculate preferred position relative to robot
        if (robot != null)
        {
            Vector3 direction = (transform.position - robot.position).normalized;
            return robot.position + direction * preferredInteractionDistance;
        }
        return transform.position;
    }

    public void ResetToOriginalPosition()
    {
        // Reset human to original position
        if (navAgent != null)
        {
            navAgent.Warp(originalPosition);
            transform.rotation = originalRotation;
        }
        else
        {
            transform.position = originalPosition;
            transform.rotation = originalRotation;
        }

        EndAttentionToRobot();
    }

    void OnDrawGizmosSelected()
    {
        // Draw comfort zone gizmo
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, comfortZoneRadius);

        // Draw preferred interaction distance
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, preferredInteractionDistance);
    }
}
```

## Voice Command System

### Voice Recognition Integration

Implement voice command processing:

```csharp
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class VoiceCommandSystem : MonoBehaviour
{
    [Header("Voice Recognition Configuration")]
    public bool enableVoiceRecognition = true;
    public float commandTimeout = 5.0f;
    public float minimumConfidence = 0.7f;

    [Header("Command Configuration")]
    public List<VoiceCommand> voiceCommands = new List<VoiceCommand>();

    [Header("Feedback Configuration")]
    public bool enableAudioFeedback = true;
    public bool enableVisualFeedback = true;
    public GameObject feedbackIndicator;

    // Private variables
    private Dictionary<string, System.Action> commandActions = new Dictionary<string, System.Action>();
    private bool isListening = false;
    private float lastCommandTime = 0f;

    [System.Serializable]
    public class VoiceCommand
    {
        public string commandPhrase;
        public string[] alternativePhrases;
        public System.Action action;
        public float confidenceThreshold = 0.8f;
    }

    void Start()
    {
        if (enableVoiceRecognition)
        {
            InitializeVoiceCommands();
            SetupCommandMappings();
        }
    }

    void InitializeVoiceCommands()
    {
        // Define voice commands with their actions
        voiceCommands.Add(new VoiceCommand {
            commandPhrase = "move forward",
            alternativePhrases = new string[] { "go forward", "forward", "move ahead" },
            action = () => ProcessMoveCommand("forward"),
            confidenceThreshold = 0.7f
        });

        voiceCommands.Add(new VoiceCommand {
            commandPhrase = "move backward",
            alternativePhrases = new string[] { "go back", "backward", "reverse" },
            action = () => ProcessMoveCommand("backward"),
            confidenceThreshold = 0.7f
        });

        voiceCommands.Add(new VoiceCommand {
            commandPhrase = "turn left",
            alternativePhrases = new string[] { "rotate left", "left turn" },
            action = () => ProcessTurnCommand("left"),
            confidenceThreshold = 0.7f
        });

        voiceCommands.Add(new VoiceCommand {
            commandPhrase = "turn right",
            alternativePhrases = new string[] { "rotate right", "right turn" },
            action = () => ProcessTurnCommand("right"),
            confidenceThreshold = 0.7f
        });

        voiceCommands.Add(new VoiceCommand {
            commandPhrase = "stop",
            alternativePhrases = new string[] { "halt", "freeze", "cease" },
            action = () => ProcessStopCommand(),
            confidenceThreshold = 0.6f
        });

        voiceCommands.Add(new VoiceCommand {
            commandPhrase = "hello robot",
            alternativePhrases = new string[] { "hi robot", "hello", "hi" },
            action = () => ProcessGreetingCommand(),
            confidenceThreshold = 0.6f
        });

        voiceCommands.Add(new VoiceCommand {
            commandPhrase = "follow me",
            alternativePhrases = new string[] { "come with me", "follow" },
            action = () => ProcessFollowCommand(),
            confidenceThreshold = 0.75f
        });

        voiceCommands.Add(new VoiceCommand {
            commandPhrase = "return to base",
            alternativePhrases = new string[] { "go home", "return", "back to base" },
            action = () => ProcessReturnHomeCommand(),
            confidenceThreshold = 0.8f
        });
    }

    void SetupCommandMappings()
    {
        // Create mapping from phrases to actions
        foreach (VoiceCommand cmd in voiceCommands)
        {
            commandActions[cmd.commandPhrase.ToLower()] = cmd.action;

            // Add alternative phrases
            if (cmd.alternativePhrases != null)
            {
                foreach (string altPhrase in cmd.alternativePhrases)
                {
                    commandActions[altPhrase.ToLower()] = cmd.action;
                }
            }
        }
    }

    public void StartListening()
    {
        if (!enableVoiceRecognition || isListening) return;

        isListening = true;
        lastCommandTime = Time.time;

        // In a real implementation, this would start the voice recognition system
        // For now, we'll simulate with Unity's input system or external library
        StartCoroutine(ListeningCoroutine());
    }

    public void StopListening()
    {
        isListening = false;
    }

    IEnumerator ListeningCoroutine()
    {
        while (isListening)
        {
            // Check for voice input (in real implementation, this would interface with voice recognition)
            if (Time.time - lastCommandTime > commandTimeout)
            {
                // Timeout - restart listening
                lastCommandTime = Time.time;
            }

            // Simulate voice recognition
            SimulateVoiceRecognition();

            yield return new WaitForSeconds(0.1f); // Check every 100ms
        }
    }

    void SimulateVoiceRecognition()
    {
        // This is a simulation - in real implementation, you'd use a voice recognition library
        // For demonstration, we'll use keyboard input to simulate voice commands

        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            ProcessVoiceCommand("move forward");
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            ProcessVoiceCommand("turn left");
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            ProcessVoiceCommand("turn right");
        }
        else if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            ProcessVoiceCommand("stop");
        }
        else if (Input.GetKeyDown(KeyCode.Alpha5))
        {
            ProcessVoiceCommand("hello robot");
        }
        else if (Input.GetKeyDown(KeyCode.Alpha6))
        {
            ProcessVoiceCommand("follow me");
        }
    }

    public void ProcessVoiceCommand(string command)
    {
        if (!enableVoiceRecognition) return;

        lastCommandTime = Time.time;

        // Normalize command string
        string normalizedCommand = command.ToLower().Trim();

        // Check for exact match first
        if (commandActions.ContainsKey(normalizedCommand))
        {
            commandActions[normalizedCommand]?.Invoke();
            ProvideCommandFeedback(command, true);
            return;
        }

        // Check for partial matches in alternatives
        foreach (var cmd in voiceCommands)
        {
            if (normalizedCommand.Contains(cmd.commandPhrase.ToLower()) ||
                (cmd.alternativePhrases != null &&
                 System.Linq.Enumerable.Any(cmd.alternativePhrases, alt => normalizedCommand.Contains(alt.ToLower()))))
            {
                cmd.action?.Invoke();
                ProvideCommandFeedback(command, true);
                return;
            }
        }

        // Command not recognized
        ProvideCommandFeedback(command, false);
        Debug.Log($"Unrecognized voice command: {command}");
    }

    void ProvideCommandFeedback(string command, bool success)
    {
        if (enableVisualFeedback && feedbackIndicator != null)
        {
            StartCoroutine(ShowCommandFeedback(success));
        }

        if (enableAudioFeedback)
        {
            // Play audio feedback
            PlayAudioFeedback(success);
        }

        string status = success ? "SUCCESS" : "FAILED";
        Debug.Log($"Voice command '{command}' processed: {status}");
    }

    IEnumerator ShowCommandFeedback(bool success)
    {
        if (feedbackIndicator != null)
        {
            Renderer feedbackRenderer = feedbackIndicator.GetComponent<Renderer>();
            Material originalMaterial = null;
            Color originalColor = Color.white;

            if (feedbackRenderer != null)
            {
                originalMaterial = feedbackRenderer.material;
                originalColor = originalMaterial.color;
                originalMaterial.color = success ? Color.green : Color.red;
            }

            feedbackIndicator.SetActive(true);
            yield return new WaitForSeconds(1.0f);
            feedbackIndicator.SetActive(false);

            if (originalMaterial != null)
            {
                originalMaterial.color = originalColor;
            }
        }
    }

    void PlayAudioFeedback(bool success)
    {
        // In a real implementation, this would play audio feedback
        // For now, we'll just log it
        string soundName = success ? "command_success" : "command_failed";
        Debug.Log($"Playing audio feedback: {soundName}");
    }

    void ProcessMoveCommand(string direction)
    {
        // Handle move commands
        RobotController robotController = FindObjectOfType<RobotController>();
        if (robotController != null)
        {
            switch (direction)
            {
                case "forward":
                    robotController.MoveForward();
                    break;
                case "backward":
                    robotController.MoveBackward();
                    break;
            }
        }
    }

    void ProcessTurnCommand(string direction)
    {
        // Handle turn commands
        RobotController robotController = FindObjectOfType<RobotController>();
        if (robotController != null)
        {
            switch (direction)
            {
                case "left":
                    robotController.TurnLeft();
                    break;
                case "right":
                    robotController.TurnRight();
                    break;
            }
        }
    }

    void ProcessStopCommand()
    {
        // Handle stop command
        RobotController robotController = FindObjectOfType<RobotController>();
        if (robotController != null)
        {
            robotController.Stop();
        }
    }

    void ProcessGreetingCommand()
    {
        // Handle greeting command
        RobotController robotController = FindObjectOfType<RobotController>();
        if (robotController != null)
        {
            robotController.Greet();
        }

        // Provide visual feedback to human
        if (HRIManager.Instance != null)
        {
            HRIManager.Instance.RespondToGreeting();
        }
    }

    void ProcessFollowCommand()
    {
        // Handle follow command
        RobotController robotController = FindObjectOfType<RobotController>();
        HumanController humanController = FindObjectOfType<HumanController>();

        if (robotController != null && humanController != null)
        {
            robotController.StartFollowing(humanController.transform);
        }
    }

    void ProcessReturnHomeCommand()
    {
        // Handle return home command
        RobotController robotController = FindObjectOfType<RobotController>();
        if (robotController != null)
        {
            robotController.ReturnHome();
        }
    }

    void OnDisable()
    {
        StopListening();
    }
}
```

## Gesture Recognition System

### Hand Gesture Recognition

Implement gesture recognition for natural interaction:

```csharp
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class GestureRecognitionSystem : MonoBehaviour
{
    [Header("Gesture Recognition Configuration")]
    public bool enableGestureRecognition = true;
    public float gestureDetectionRange = 3.0f;
    public float gestureHoldDuration = 0.5f;
    public float gestureRecognitionThreshold = 0.8f;

    [Header("Supported Gestures")]
    public List<GestureDefinition> supportedGestures = new List<GestureDefinition>();

    [Header("Gesture Feedback")]
    public bool enableVisualFeedback = true;
    public GameObject gestureFeedbackIndicator;

    // Private variables
    private bool isRecognizing = false;
    private Dictionary<string, GestureDefinition> gestureMap = new Dictionary<string, GestureDefinition>();
    private List<GestureSequence> gestureSequences = new List<GestureSequence>();

    [System.Serializable]
    public class GestureDefinition
    {
        public string gestureName;
        public GestureType gestureType;
        public List<Vector3> gesturePath;  // For path-based gestures
        public float activationThreshold = 0.8f;
        public System.Action action;
    }

    public enum GestureType
    {
        Wave,
        Point,
        Beckon,  // Come here
        Stop,
        ThumbsUp,
        ThumbsDown,
        PeaceSign,
        Fist,
        OpenHand
    }

    [System.Serializable]
    public class GestureSequence
    {
        public List<Vector3> positions;
        public float startTime;
        public float endTime;
        public bool isActive;

        public GestureSequence()
        {
            positions = new List<Vector3>();
            startTime = Time.time;
            isActive = true;
        }

        public void AddPosition(Vector3 pos)
        {
            positions.Add(pos);
        }

        public void Complete()
        {
            endTime = Time.time;
            isActive = false;
        }

        public float GetDuration()
        {
            return endTime - startTime;
        }
    }

    void Start()
    {
        if (enableGestureRecognition)
        {
            InitializeGestureDefinitions();
            SetupGestureMapping();
        }
    }

    void InitializeGestureDefinitions()
    {
        // Define common gestures with their recognition parameters
        supportedGestures.Add(new GestureDefinition {
            gestureName = "wave",
            gestureType = GestureType.Wave,
            gesturePath = new List<Vector3>(), // Path will be learned/compared dynamically
            activationThreshold = 0.7f,
            action = () => ProcessWaveGesture()
        });

        supportedGestures.Add(new GestureDefinition {
            gestureName = "point",
            gestureType = GestureType.Point,
            gesturePath = new List<Vector3>(),
            activationThreshold = 0.75f,
            action = () => ProcessPointGesture()
        });

        supportedGestures.Add(new GestureDefinition {
            gestureName = "beckon",
            gestureType = GestureType.Beckon,
            gesturePath = new List<Vector3>(),
            activationThreshold = 0.8f,
            action = () => ProcessBeckonGesture()
        });

        supportedGestures.Add(new GestureDefinition {
            gestureName = "stop",
            gestureType = GestureType.Stop,
            gesturePath = new List<Vector3>(),
            activationThreshold = 0.7f,
            action = () => ProcessStopGesture()
        });

        supportedGestures.Add(new GestureDefinition {
            gestureName = "thumbs_up",
            gestureType = GestureType.ThumbsUp,
            gesturePath = new List<Vector3>(),
            activationThreshold = 0.85f,
            action = () => ProcessThumbsUpGesture()
        });

        supportedGestures.Add(new GestureDefinition {
            gestureName = "follow_me",
            gestureType = GestureType.Beckon,
            gesturePath = new List<Vector3>(),
            activationThreshold = 0.8f,
            action = () => ProcessFollowMeGesture()
        });
    }

    void SetupGestureMapping()
    {
        foreach (GestureDefinition gesture in supportedGestures)
        {
            gestureMap[gesture.gestureName.ToLower()] = gesture;
        }
    }

    public void StartRecognition()
    {
        if (!enableGestureRecognition || isRecognizing) return;

        isRecognizing = true;
        StartCoroutine(GestureRecognitionCoroutine());
    }

    public void StopRecognition()
    {
        isRecognizing = false;
    }

    IEnumerator GestureRecognitionCoroutine()
    {
        while (isRecognizing)
        {
            // Simulate gesture detection (in real implementation, this would interface with hand tracking)
            SimulateGestureDetection();

            yield return new WaitForSeconds(0.05f); // 20 FPS gesture detection
        }
    }

    void SimulateGestureDetection()
    {
        // Simulate gesture detection using keyboard input for demonstration
        if (Input.GetKeyDown(KeyCode.G))
        {
            // Simulate a wave gesture
            SimulateWaveGesture();
        }
        else if (Input.GetKeyDown(KeyCode.H))
        {
            // Simulate a point gesture
            SimulatePointGesture();
        }
        else if (Input.GetKeyDown(KeyCode.J))
        {
            // Simulate a beckon gesture
            SimulateBeckonGesture();
        }
        else if (Input.GetKeyDown(KeyCode.K))
        {
            // Simulate a stop gesture
            SimulateStopGesture();
        }
    }

    void SimulateWaveGesture()
    {
        // Create a simulated gesture sequence
        GestureSequence seq = new GestureSequence();

        // Add simulated positions for waving motion
        for (int i = 0; i < 10; i++)
        {
            Vector3 pos = new Vector3(
                Mathf.Sin(i * 0.5f) * 0.1f,
                0.5f + Mathf.Cos(i * 0.3f) * 0.05f,
                0f
            );
            seq.AddPosition(pos);
        }

        seq.Complete();
        ProcessGestureSequence(seq, "wave");
    }

    void SimulatePointGesture()
    {
        GestureSequence seq = new GestureSequence();

        // Add simulated positions for pointing motion
        for (int i = 0; i < 5; i++)
        {
            Vector3 pos = new Vector3(0.5f, 0.3f, 0f);
            seq.AddPosition(pos);
        }

        seq.Complete();
        ProcessGestureSequence(seq, "point");
    }

    void SimulateBeckonGesture()
    {
        GestureSequence seq = new GestureSequence();

        // Add simulated positions for beckoning motion
        for (int i = 0; i < 8; i++)
        {
            Vector3 pos = new Vector3(
                0.3f + (i % 2) * 0.1f,  // Alternating position
                0.4f,
                0f
            );
            seq.AddPosition(pos);
        }

        seq.Complete();
        ProcessGestureSequence(seq, "beckon");
    }

    void SimulateStopGesture()
    {
        GestureSequence seq = new GestureSequence();

        // Add simulated positions for stop gesture (flat hand)
        for (int i = 0; i < 5; i++)
        {
            Vector3 pos = new Vector3(0.4f, 0.4f, 0f);
            seq.AddPosition(pos);
        }

        seq.Complete();
        ProcessGestureSequence(seq, "stop");
    }

    void ProcessGestureSequence(GestureSequence sequence, string expectedGesture = "")
    {
        if (sequence.positions.Count < 3) return; // Need at least 3 points for meaningful gesture

        // Analyze the gesture sequence
        string recognizedGesture = AnalyzeGesture(sequence);

        if (!string.IsNullOrEmpty(recognizedGesture))
        {
            // Check if this matches expected gesture or is a valid gesture
            if (string.IsNullOrEmpty(expectedGesture) ||
                recognizedGesture.ToLower().Contains(expectedGesture.ToLower()))
            {
                ExecuteGestureAction(recognizedGesture);
                ProvideGestureFeedback(recognizedGesture, true);
            }
        }
        else
        {
            ProvideGestureFeedback(expectedGesture, false);
        }
    }

    string AnalyzeGesture(GestureSequence sequence)
    {
        // Simple gesture analysis based on movement patterns
        if (sequence.positions.Count < 3) return "";

        // Calculate gesture metrics
        Vector3 start = sequence.positions[0];
        Vector3 end = sequence.positions[sequence.positions.Count - 1];
        Vector3 displacement = end - start;

        // Analyze movement characteristics
        float totalDistance = 0f;
        for (int i = 1; i < sequence.positions.Count; i++)
        {
            totalDistance += Vector3.Distance(sequence.positions[i-1], sequence.positions[i]);
        }

        // Calculate average velocity
        float duration = sequence.GetDuration();
        float avgVelocity = duration > 0 ? totalDistance / duration : 0;

        // Identify gesture based on movement pattern
        if (avgVelocity > 0.1f && totalDistance > 0.1f)
        {
            // Check for oscillating pattern (wave)
            float oscillationScore = CalculateOscillationScore(sequence.positions);
            if (oscillationScore > 0.6f)
            {
                return "wave";
            }

            // Check for directional movement (point)
            if (totalDistance > 0.15f && displacement.magnitude > 0.1f)
            {
                return "point";
            }
        }

        // Check for sustained position (stop/thumbs up)
        if (duration > 0.3f && displacement.magnitude < 0.05f)
        {
            return "stop";
        }

        return "";
    }

    float CalculateOscillationScore(List<Vector3> positions)
    {
        if (positions.Count < 4) return 0f;

        float oscillationScore = 0f;
        int oscillationCount = 0;

        for (int i = 1; i < positions.Count - 1; i++)
        {
            Vector3 prev = positions[i-1];
            Vector3 curr = positions[i];
            Vector3 next = positions[i+1];

            // Calculate direction changes
            Vector3 dir1 = (curr - prev).normalized;
            Vector3 dir2 = (next - curr).normalized;

            float dotProduct = Vector3.Dot(dir1, dir2);
            if (dotProduct < -0.5f) // Opposite directions indicate oscillation
            {
                oscillationCount++;
            }
        }

        return (float)oscillationCount / (positions.Count - 2);
    }

    void ExecuteGestureAction(string gestureName)
    {
        if (gestureMap.ContainsKey(gestureName.ToLower()))
        {
            gestureMap[gestureName.ToLower()].action?.Invoke();
        }
        else
        {
            // Try to find gesture by partial match
            foreach (var kvp in gestureMap)
            {
                if (kvp.Key.Contains(gestureName.ToLower()) || gestureName.ToLower().Contains(kvp.Key))
                {
                    kvp.Value.action?.Invoke();
                    break;
                }
            }
        }
    }

    void ProvideGestureFeedback(string gestureName, bool success)
    {
        if (enableVisualFeedback && gestureFeedbackIndicator != null)
        {
            StartCoroutine(ShowGestureFeedback(gestureName, success));
        }

        string status = success ? "RECOGNIZED" : "UNRECOGNIZED";
        Debug.Log($"Gesture '{gestureName}' {status}");
    }

    IEnumerator ShowGestureFeedback(string gestureName, bool success)
    {
        if (gestureFeedbackIndicator != null)
        {
            Renderer feedbackRenderer = gestureFeedbackIndicator.GetComponent<Renderer>();
            Material originalMaterial = null;
            Color originalColor = Color.white;

            if (feedbackRenderer != null)
            {
                originalMaterial = feedbackRenderer.material;
                originalColor = originalMaterial.color;
                originalMaterial.color = success ? Color.green : Color.red;
            }

            gestureFeedbackIndicator.SetActive(true);

            // Animate feedback
            float duration = 1.0f;
            float startTime = Time.time;
            while (Time.time - startTime < duration)
            {
                float progress = (Time.time - startTime) / duration;
                if (feedbackRenderer != null)
                {
                    float scale = 1.0f + Mathf.Sin(progress * Mathf.PI * 4) * 0.2f; // Pulsing effect
                    gestureFeedbackIndicator.transform.localScale = Vector3.one * scale;
                }
                yield return null;
            }

            gestureFeedbackIndicator.SetActive(false);
            gestureFeedbackIndicator.transform.localScale = Vector3.one;

            if (originalMaterial != null)
            {
                originalMaterial.color = originalColor;
            }
        }
    }

    void ProcessWaveGesture()
    {
        Debug.Log("Wave gesture recognized");

        // Respond to wave
        RobotController robot = FindObjectOfType<RobotController>();
        if (robot != null)
        {
            robot.ReactToWave();
        }

        // Provide feedback to human
        HumanController human = FindObjectOfType<HumanController>();
        if (human != null)
        {
            human.ReactToRobotAction("wave_acknowledge");
        }
    }

    void ProcessPointGesture()
    {
        Debug.Log("Point gesture recognized");

        // Process pointing gesture
        RobotController robot = FindObjectOfType<RobotController>();
        if (robot != null)
        {
            robot.ReactToPointing();
        }
    }

    void ProcessBeckonGesture()
    {
        Debug.Log("Beckon gesture recognized");

        // Process beckoning gesture (come here)
        RobotController robot = FindObjectOfType<RobotController>();
        HumanController human = FindObjectOfType<HumanController>();

        if (robot != null && human != null)
        {
            robot.MoveToHuman(human.transform);
        }
    }

    void ProcessStopGesture()
    {
        Debug.Log("Stop gesture recognized");

        // Process stop gesture
        RobotController robot = FindObjectOfType<RobotController>();
        if (robot != null)
        {
            robot.Stop();
        }
    }

    void ProcessThumbsUpGesture()
    {
        Debug.Log("Thumbs up gesture recognized");

        // Process thumbs up (approval/agreement)
        RobotController robot = FindObjectOfType<RobotController>();
        if (robot != null)
        {
            robot.ReactToApproval();
        }
    }

    void ProcessFollowMeGesture()
    {
        Debug.Log("Follow me gesture recognized");

        // Process follow me command
        RobotController robot = FindObjectOfType<RobotController>();
        HumanController human = FindObjectOfType<HumanController>();

        if (robot != null && human != null)
        {
            robot.StartFollowing(human.transform);
        }
    }

    void OnDisable()
    {
        StopRecognition();
    }
}
```

## Safety Systems for HRI

### Safety Monitoring and Collision Avoidance

Implement safety systems for human-robot interaction:

```csharp
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class HRISafetySystem : MonoBehaviour
{
    [Header("Safety Configuration")]
    public float safetyDistance = 0.5f;        // Minimum safe distance to human
    public float warningDistance = 1.0f;       // Distance to warn about proximity
    public float emergencyStopDistance = 0.2f; // Distance for emergency stop
    public float maximumApproachSpeed = 0.3f;  // Maximum speed when approaching human
    public bool enableCollisionAvoidance = true;
    public bool enableEmergencyProtocols = true;

    [Header("Safety Zones")]
    public bool visualizeSafetyZones = true;
    public Color safetyZoneColor = Color.red;
    public Color warningZoneColor = Color.yellow;

    [Header("Emergency Response")]
    public float emergencyStopDuration = 2.0f;
    public bool enableHapticFeedback = true;
    public float hapticIntensity = 0.8f;

    // Private variables
    private List<GameObject> humansInRange = new List<GameObject>();
    private Dictionary<GameObject, float> lastSafeTime = new Dictionary<GameObject, float>();
    private bool isEmergencyActive = false;
    private float emergencyEndTime = 0f;
    private GameObject emergencyTriggeredHuman = null;

    void Start()
    {
        InitializeSafetySystem();
    }

    void InitializeSafetySystem()
    {
        // Initialize safety monitoring
        StartCoroutine(SafetyMonitoringCoroutine());

        Debug.Log("HRI Safety System initialized");
    }

    IEnumerator SafetyMonitoringCoroutine()
    {
        while (true)
        {
            UpdateSafetyMonitoring();

            yield return new WaitForSeconds(0.1f); // 10 Hz safety check
        }
    }

    void UpdateSafetyMonitoring()
    {
        if (isEmergencyActive)
        {
            if (Time.time > emergencyEndTime)
            {
                EndEmergencyMode();
            }
            return; // Don't check safety during emergency
        }

        // Get all humans in the scene
        HumanController[] humans = FindObjectsOfType<HumanController>();
        humansInRange.Clear();

        foreach (HumanController human in humans)
        {
            if (human != null)
            {
                float distance = Vector3.Distance(transform.position, human.transform.position);

                if (distance <= warningDistance)
                {
                    humansInRange.Add(human.gameObject);

                    if (distance <= safetyDistance)
                    {
                        HandleSafetyViolation(human.gameObject, distance);
                    }
                    else if (distance <= warningDistance)
                    {
                        HandleProximityWarning(human.gameObject, distance);
                    }

                    lastSafeTime[human.gameObject] = Time.time;
                }
            }
        }
    }

    void HandleSafetyViolation(GameObject human, float distance)
    {
        Debug.LogWarning($"SAFETY VIOLATION: Human {human.name} too close! Distance: {distance:F3}m");

        // Trigger safety response
        if (enableEmergencyProtocols)
        {
            TriggerEmergencyResponse(human, distance);
        }
        else
        {
            // Just slow down or stop
            RobotController robot = FindObjectOfType<RobotController>();
            if (robot != null)
            {
                robot.SlowDownApproach();
            }
        }

        // Provide feedback to human
        HumanController humanCtrl = human.GetComponent<HumanController>();
        if (humanCtrl != null)
        {
            humanCtrl.ReactToRobotAction("safety_warning");
        }
    }

    void HandleProximityWarning(GameObject human, float distance)
    {
        Debug.Log($"PROXIMITY WARNING: Human {human.name} approaching. Distance: {distance:F3}m");

        // Provide gentle warning
        RobotController robot = FindObjectOfType<RobotController>();
        if (robot != null)
        {
            robot.AdjustApproachSpeed(distance);
        }

        // Visual feedback to human
        StartCoroutine(ShowProximityWarning(human));
    }

    void TriggerEmergencyResponse(GameObject human, float distance)
    {
        if (isEmergencyActive) return; // Already in emergency mode

        isEmergencyActive = true;
        emergencyEndTime = Time.time + emergencyStopDuration;
        emergencyTriggeredHuman = human;

        Debug.LogError($"EMERGENCY STOP TRIGGERED: Human {human.name} at {distance:F3}m!");

        // Stop robot immediately
        RobotController robot = FindObjectOfType<RobotController>();
        if (robot != null)
        {
            robot.EmergencyStop();
        }

        // Provide emergency feedback
        ProvideEmergencyFeedback();

        // Log safety incident
        LogSafetyIncident(human, distance);
    }

    void EndEmergencyMode()
    {
        isEmergencyActive = false;
        emergencyTriggeredHuman = null;

        Debug.Log("Emergency mode ended");

        // Resume normal operation
        RobotController robot = FindObjectOfType<RobotController>();
        if (robot != null)
        {
            robot.ResumeNormalOperation();
        }
    }

    void ProvideEmergencyFeedback()
    {
        // Visual emergency feedback
        StartCoroutine(ShowEmergencyVisualFeedback());

        // Audio emergency feedback
        PlayEmergencyAudio();

        // Haptic feedback if enabled
        if (enableHapticFeedback)
        {
            TriggerHapticFeedback();
        }
    }

    IEnumerator ShowEmergencyVisualFeedback()
    {
        // Create emergency visual indicator
        GameObject emergencyIndicator = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        emergencyIndicator.name = "EmergencyIndicator";
        emergencyIndicator.transform.position = transform.position + Vector3.up * 1.0f;
        emergencyIndicator.transform.localScale = Vector3.one * 0.3f;

        Renderer renderer = emergencyIndicator.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = Color.red;
        }

        // Make it blink rapidly
        float startTime = Time.time;
        while (Time.time - startTime < emergencyStopDuration)
        {
            renderer.enabled = !(Time.time * 10 % 2 < 1); // Fast blinking
            yield return null;
        }

        renderer.enabled = true;
        Destroy(emergencyIndicator, 1.0f); // Clean up after delay
    }

    void PlayEmergencyAudio()
    {
        // In a real implementation, this would play emergency audio
        Debug.Log("Playing emergency audio feedback");
    }

    void TriggerHapticFeedback()
    {
        // In a real implementation, this would trigger haptic feedback
        Debug.Log("Triggering haptic feedback");
    }

    IEnumerator ShowProximityWarning(GameObject human)
    {
        // Show warning indicator near the human
        GameObject warningIndicator = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        warningIndicator.name = "ProximityWarning";
        warningIndicator.transform.position = human.transform.position + Vector3.up * 1.5f;
        warningIndicator.transform.localScale = new Vector3(0.1f, 0.3f, 0.1f);

        Renderer renderer = warningIndicator.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = Color.yellow;
        }

        yield return new WaitForSeconds(2.0f);
        Destroy(warningIndicator);
    }

    void LogSafetyIncident(GameObject human, float distance)
    {
        string incidentLog = $"[{System.DateTime.Now:yyyy-MM-dd HH:mm:ss}] SAFETY_INCIDENT - Human: {human.name}, Distance: {distance:F3}m, Robot_Pos: {transform.position}";

        // In a real system, you'd log this to a file
        Debug.Log(incidentLog);
    }

    public bool IsHumanInSafetyZone(GameObject human)
    {
        if (human == null) return false;

        float distance = Vector3.Distance(transform.position, human.transform.position);
        return distance <= safetyDistance;
    }

    public bool IsHumanInWarningZone(GameObject human)
    {
        if (human == null) return false;

        float distance = Vector3.Distance(transform.position, human.transform.position);
        return distance <= warningDistance && distance > safetyDistance;
    }

    public List<GameObject> GetHumansInProximity()
    {
        return new List<GameObject>(humansInRange);
    }

    public bool IsInEmergencyMode()
    {
        return isEmergencyActive;
    }

    void OnDrawGizmosSelected()
    {
        if (!visualizeSafetyZones) return;

        // Draw safety zones
        Gizmos.color = safetyZoneColor;
        Gizmos.DrawWireSphere(transform.position, safetyDistance);

        Gizmos.color = warningZoneColor;
        Gizmos.DrawWireSphere(transform.position, warningDistance);

        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(transform.position, maximumApproachSpeed);
    }

    public float GetMinimumSafeDistance()
    {
        return safetyDistance;
    }

    public float GetWarningDistance()
    {
        return warningDistance;
    }

    public float GetEmergencyStopDistance()
    {
        return emergencyStopDistance;
    }

    public void OverrideSafetyDistance(float newDistance)
    {
        if (newDistance > 0)
        {
            safetyDistance = newDistance;
            Debug.Log($"Safety distance overridden to {newDistance:F3}m");
        }
    }

    public void ResetSafetySystem()
    {
        if (isEmergencyActive)
        {
            EndEmergencyMode();
        }

        humansInRange.Clear();
        lastSafeTime.Clear();

        Debug.Log("Safety system reset");
    }
}
```

## Validation and Testing

### HRI System Validation

Create validation tests for the HRI system:

```python
# hri_validation_tests.py
import unittest
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class HRISystemValidator(unittest.TestCase):
    def setUp(self):
        rospy.init_node('hri_validator', anonymous=True)

        # Publishers for testing
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        self.voice_cmd_pub = rospy.Publisher('/robot1/voice_command', String, queue_size=10)

        # Subscribers for validation
        self.lidar_sub = rospy.Subscriber('/robot1/scan', LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber('/robot1/odom', Odometry, self.odom_callback)

        # Test variables
        self.lidar_data = None
        self.odom_data = None
        self.test_start_time = rospy.Time.now()

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def odom_callback(self, msg):
        self.odom_data = msg

    def test_safety_zone_detection(self):
        """Test that safety zones are properly detected"""
        # Move robot toward obstacle
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.3  # Move forward slowly
        cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_vel)
        time.sleep(2.0)  # Move for 2 seconds

        # Check that robot stops when reaching safety zone
        # This would require checking if robot actually stopped
        self.assertIsNotNone(self.lidar_data, "Lidar data should be received")

        # Find minimum distance in scan
        if self.lidar_data and len(self.lidar_data.ranges) > 0:
            min_distance = min([r for r in self.lidar_data.ranges if r > 0])

            # With safety distance of 0.5m, robot should stop before reaching obstacle
            safety_margin = 0.1  # 10cm safety margin
            expected_min_distance = 0.5 - safety_margin

            self.assertGreater(min_distance, expected_min_distance,
                            f"Robot should maintain distance > {expected_min_distance}m from obstacles")

    def test_voice_command_recognition(self):
        """Test voice command recognition and execution"""
        # Send voice command
        voice_cmd = String()
        voice_cmd.data = "move forward"
        self.voice_cmd_pub.publish(voice_cmd)

        # Wait for response
        time.sleep(1.0)

        # Check if robot moved (compare odometry)
        initial_pos = self.odom_data.pose.pose.position if self.odom_data else None
        time.sleep(2.0)  # Wait for movement
        final_pos = self.odom_data.pose.pose.position if self.odom_data else None

        if initial_pos and final_pos:
            # Check if robot moved forward
            distance_moved = np.sqrt(
                (final_pos.x - initial_pos.x)**2 +
                (final_pos.y - initial_pos.y)**2
            )
            self.assertGreater(distance_moved, 0.1, "Robot should move forward after command")

    def test_proximity_reaction(self):
        """Test robot's reaction to human proximity"""
        # This test would require simulation of human presence
        # For now, we'll test the detection system

        # Verify that lidar data is being received
        self.assertIsNotNone(self.lidar_data, "Lidar data should be available for proximity detection")

        # Check that scan has valid data
        if self.lidar_data:
            self.assertGreater(len(self.lidar_data.ranges), 0, "Lidar should have range measurements")
            self.assertLess(self.lidar_data.range_min, self.lidar_data.range_max,
                          "Lidar range min should be less than max")

    def test_hri_performance(self):
        """Test HRI system performance metrics"""
        # Test response time
        start_time = rospy.Time.now()

        # Send command
        cmd = String()
        cmd.data = "stop"
        self.voice_cmd_pub.publish(cmd)

        # Measure response time
        response_start = rospy.Time.now()
        # Wait for robot to stop
        time.sleep(1.0)
        response_end = rospy.Time.now()

        response_time = (response_end - response_start).to_sec()
        max_acceptable_response_time = 2.0  # seconds

        self.assertLess(response_time, max_acceptable_response_time,
                       f"Response time {response_time}s should be < {max_acceptable_response_time}s")

    def tearDown(self):
        # Stop robot at the end of tests
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

def run_hri_validation():
    """Run comprehensive HRI validation"""
    import rostest
    rostest.rosrun('my_robot_description', 'hri_validator', HRISystemValidator)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('my_robot_description', 'hri_validator', HRISystemValidator)
```

## Performance Optimization

### Efficient HRI Processing

Optimize HRI systems for real-time performance:

```csharp
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;

public class OptimizedHRISystem : MonoBehaviour
{
    [Header("Performance Settings")]
    public int maxHumansToTrack = 10;
    public float detectionUpdateRate = 20.0f; // Hz
    public bool enableMultithreading = true;

    [Header("LOD Settings")]
    public float closeInteractionDistance = 1.0f;
    public float mediumInteractionDistance = 3.0f;
    public float farInteractionDistance = 8.0f;

    private NativeArray<float3> humanPositions;
    private NativeArray<bool> humanInteractionStates;
    private NativeArray<float> humanAttentionTimers;
    private int humanCount = 0;

    private float lastDetectionTime = 0f;
    private bool[] interactionLODLevels = new bool[10]; // Track LOD for each human

    void Start()
    {
        InitializeNativeArrays();
        StartCoroutine(OptimizedDetectionCoroutine());
    }

    void InitializeNativeArrays()
    {
        humanPositions = new NativeArray<float3>(maxHumansToTrack, Allocator.Persistent);
        humanInteractionStates = new NativeArray<bool>(maxHumansToTrack, Allocator.Persistent);
        humanAttentionTimers = new NativeArray<float>(maxHumansToTrack, Allocator.Persistent);
    }

    IEnumerator OptimizedDetectionCoroutine()
    {
        float detectionInterval = 1.0f / detectionUpdateRate;

        while (true)
        {
            if (Time.time - lastDetectionTime >= detectionInterval)
            {
                if (enableMultithreading)
                {
                    ProcessHumansWithJobs();
                }
                else
                {
                    ProcessHumansStandard();
                }

                lastDetectionTime = Time.time;
            }

            yield return null;
        }
    }

    void ProcessHumansWithJobs()
    {
        // Get all human positions
        GameObject[] humanObjects = GameObject.FindGameObjectsWithTag("Human");
        humanCount = Mathf.Min(humanObjects.Length, maxHumansToTrack);

        // Copy positions to native array
        for (int i = 0; i < humanCount; i++)
        {
            humanPositions[i] = (float3)humanObjects[i].transform.position;
        }

        // Create job to process human interactions
        var interactionJob = new HumanInteractionJob
        {
            robotPosition = (float3)transform.position,
            humanPositions = humanPositions,
            interactionStates = humanInteractionStates,
            attentionTimers = humanAttentionTimers,
            humanCount = humanCount,
            closeDistance = closeInteractionDistance,
            mediumDistance = mediumInteractionDistance,
            farDistance = farInteractionDistance,
            deltaTime = Time.deltaTime
        };

        JobHandle handle = interactionJob.Schedule(humanCount, 8); // 8 humans per job
        handle.Complete(); // Wait for completion

        // Update Unity objects based on job results
        UpdateHumanObjectsFromJobResults(humanObjects);
    }

    void UpdateHumanObjectsFromJobResults(GameObject[] humanObjects)
    {
        for (int i = 0; i < Mathf.Min(humanCount, humanObjects.Length); i++)
        {
            // Update interaction state based on job results
            bool isInteracting = humanInteractionStates[i];
            float attentionTimer = humanAttentionTimers[i];

            HumanController humanCtrl = humanObjects[i].GetComponent<HumanController>();
            if (humanCtrl != null)
            {
                if (isInteracting)
                {
                    humanCtrl.AttendToRobot(transform, attentionTimer);
                }
                else
                {
                    humanCtrl.EndAttentionToRobot();
                }
            }
        }
    }

    void ProcessHumansStandard()
    {
        GameObject[] humanObjects = GameObject.FindGameObjectsWithTag("Human");
        humanCount = Mathf.Min(humanObjects.Length, maxHumansToTrack);

        for (int i = 0; i < humanCount; i++)
        {
            GameObject human = humanObjects[i];
            float distance = Vector3.Distance(transform.position, human.transform.position);

            bool shouldInteract = distance <= closeInteractionDistance;
            float attentionTime = shouldInteract ? 5.0f : 0.1f;

            HumanController humanCtrl = human.GetComponent<HumanController>();
            if (humanCtrl != null)
            {
                if (shouldInteract)
                {
                    humanCtrl.AttendToRobot(transform, attentionTime);
                }
                else
                {
                    humanCtrl.EndAttentionToRobot();
                }
            }

            // Apply LOD based on distance
            ApplyLODToHuman(human, distance);
        }
    }

    void ApplyLODToHuman(GameObject human, float distance)
    {
        // Apply different levels of detail based on distance
        if (distance <= closeInteractionDistance)
        {
            // Full detail interaction
            SetHumanLODLevel(human, 0); // Highest detail
        }
        else if (distance <= mediumInteractionDistance)
        {
            // Medium detail
            SetHumanLODLevel(human, 1);
        }
        else if (distance <= farInteractionDistance)
        {
            // Low detail
            SetHumanLODLevel(human, 2);
        }
        else
        {
            // Minimal processing
            SetHumanLODLevel(human, 3); // Lowest detail
        }
    }

    void SetHumanLODLevel(GameObject human, int lodLevel)
    {
        // In a real implementation, this would adjust the human model's detail level
        // For example, changing mesh complexity, animation detail, or sensor update frequency
    }

    public void AddHumanForTracking(GameObject human)
    {
        if (humanCount < maxHumansToTrack)
        {
            humanPositions[humanCount] = (float3)human.transform.position;
            humanInteractionStates[humanCount] = false;
            humanAttentionTimers[humanCount] = 0f;
            humanCount++;
        }
    }

    public int GetTrackedHumanCount()
    {
        return humanCount;
    }

    void OnDestroy()
    {
        if (humanPositions.IsCreated) humanPositions.Dispose();
        if (humanInteractionStates.IsCreated) humanInteractionStates.Dispose();
        if (humanAttentionTimers.IsCreated) humanAttentionTimers.Dispose();
    }
}

public struct HumanInteractionJob : IJobParallelFor
{
    [ReadOnly] public float3 robotPosition;
    [ReadOnly] public NativeArray<float3> humanPositions;
    public NativeArray<bool> interactionStates;
    public NativeArray<float> attentionTimers;
    public int humanCount;
    public float closeDistance;
    public float mediumDistance;
    public float farDistance;
    public float deltaTime;

    public void Execute(int index)
    {
        if (index >= humanCount) return;

        float3 humanPos = humanPositions[index];
        float distance = math.length(humanPos - robotPosition);

        // Determine interaction state based on distance
        bool shouldInteract = distance <= closeDistance;
        interactionStates[index] = shouldInteract;

        // Update attention timer
        if (shouldInteract)
        {
            attentionTimers[index] = math.max(0, attentionTimers[index] - deltaTime);
        }
        else
        {
            attentionTimers[index] = 0f;
        }
    }
}
```

## Troubleshooting Common Issues

### Common HRI Issues and Solutions

1. **Performance Issues**: Reduce update rates or implement LOD systems
2. **Safety Violations**: Adjust safety parameters and implement proper monitoring
3. **Recognition Failures**: Improve sensor quality and recognition algorithms
4. **Synchronization Problems**: Ensure proper timing and state management
5. **Integration Issues**: Verify ROS 2 topic mappings and message formats

## Best Practices

### HRI Design Best Practices

1. **User-Centered Design**: Always design with human capabilities and limitations in mind
2. **Safety-First Approach**: Prioritize human safety in all interactions
3. **Clear Feedback**: Provide immediate and clear feedback for all interactions
4. **Consistent Behavior**: Ensure robot behavior is predictable and consistent
5. **Gradual Introduction**: Introduce new capabilities gradually to build trust

### Performance Best Practices

1. **Efficient Detection**: Use optimized algorithms for human detection
2. **Selective Processing**: Only process relevant humans within interaction range
3. **Asynchronous Operations**: Use coroutines for non-critical operations
4. **Memory Management**: Properly dispose of resources and manage memory
5. **Profiling**: Regularly profile performance to identify bottlenecks

## Summary

This tutorial covered the complete implementation of Human-Robot Interaction systems in Unity integrated with ROS 2. We explored:

1. **HRI Fundamentals**: Principles and design considerations
2. **Unity Implementation**: Creating interactive systems and interfaces
3. **Voice Recognition**: Implementing natural language interfaces
4. **Gesture Recognition**: Enabling intuitive gesture-based control
5. **Safety Systems**: Implementing comprehensive safety protocols
6. **Performance Optimization**: Optimizing for real-time operation
7. **Validation**: Testing and validating HRI systems

The HRI system enables natural, safe, and effective interaction between humans and robots in simulation environments, providing the foundation for developing and testing advanced human-robot collaboration scenarios.

## References

1. Goodrich, M. A., & Schultz, A. C. (2007). Human-robot interaction: A survey. Foundations and Trends in Human-Computer Interaction.
2. ISO 13482:2014. Robots and robotic devices — Safety requirements for personal care robots.
3. Mataric, M. J., & Scassellati, B. (2007). Socially assistive robotics. In Foundations and Trends in Robotics.

## Exercises

1. Implement a multimodal HRI system combining voice, gestures, and touch
2. Create a safety validation framework for HRI systems
3. Develop adaptive HRI that learns user preferences
4. Design HRI interfaces for users with different abilities
5. Validate HRI system performance under various environmental conditions