---
title: Chapter 2 - Digital Twins & HRI in Unity
sidebar_label: Digital Twins & HRI in Unity
---

# Chapter 2: Digital Twins & HRI in Unity

## Introduction to Digital Twins in Robotics

Digital twins are virtual replicas of physical systems that enable real-time monitoring, simulation, and analysis. In robotics, digital twins serve as powerful tools for:

- Pre-deployment testing and validation
- Human-robot interaction (HRI) prototyping
- System optimization and debugging
- Training and education

Unity provides an ideal platform for creating high-fidelity digital twins with realistic physics, lighting, and interaction capabilities.

## Setting Up Unity for Robotics

### Installation and Requirements

To get started with Unity for robotics applications:

1. **Download Unity Hub**: The recommended way to manage Unity installations
2. **Install Unity Editor**: Use LTS (Long Term Support) versions for stability
3. **Required packages**:
   - Unity Physics
   - Input System
   - Universal Render Pipeline (URP) or High Definition Render Pipeline (HDRP)

### Unity Robotics Hub

Unity provides specialized tools for robotics development:

- **Unity Robotics Hub**: Centralized package management for robotics tools
- **ROS# (ROS Bridge)**: Connect Unity to ROS/ROS2 systems
- **Unity Perception**: Generate synthetic training data for AI models
- **ML-Agents**: Train intelligent agents using reinforcement learning

## Creating Your First Digital Twin

### Basic Scene Setup

Start by creating a new Unity scene for your robot digital twin:

```csharp
// RobotController.cs - Basic robot controller script
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public float movementSpeed = 5.0f;
    public float rotationSpeed = 100.0f;

    [Header("Components")]
    public Transform baseLink;
    public Transform[] joints;

    void Start()
    {
        // Initialize robot components
        InitializeRobot();
    }

    void Update()
    {
        // Handle robot movement and control
        HandleRobotControl();
    }

    void InitializeRobot()
    {
        // Set initial robot state
        if (baseLink == null)
            baseLink = transform;
    }

    void HandleRobotControl()
    {
        // Example: Basic movement controls
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        Vector3 movement = new Vector3(horizontal, 0, vertical) * movementSpeed * Time.deltaTime;
        baseLink.Translate(movement);

        float rotation = Input.GetAxis("Rotate") * rotationSpeed * Time.deltaTime;
        baseLink.Rotate(Vector3.up, rotation);
    }
}
```

### Importing Robot Models

Unity supports various robot model formats:

- **URDF Importer**: Direct import of ROS URDF files
- **FBX/OBJ**: Standard 3D model formats
- **URDF-Unity Bridge**: Convert ROS robot descriptions to Unity

## Human-Robot Interaction (HRI) in Unity

### Interaction Systems

Unity provides multiple approaches for HRI:

1. **VR/AR Integration**: Use XR systems for immersive interaction
2. **Touch/Gesture Controls**: For mobile/touchscreen interfaces
3. **Voice Commands**: Integration with speech recognition systems
4. **Gesture Recognition**: Using camera-based tracking

### UI Systems for HRI

Create intuitive interfaces for human-robot interaction:

```csharp
// HRIInterface.cs - Human-Robot Interaction interface
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class HRIInterface : MonoBehaviour
{
    [Header("UI Elements")]
    public Button[] robotControlButtons;
    public Slider speedSlider;
    public TMP_Text statusText;
    public GameObject interactionPanel;

    [Header("Robot Communication")]
    public string robotIP = "127.0.0.1";
    public int robotPort = 11311;

    void Start()
    {
        SetupUI();
        ConnectToRobot();
    }

    void SetupUI()
    {
        // Configure control buttons
        foreach (Button button in robotControlButtons)
        {
            button.onClick.AddListener(() => OnControlButtonClicked(button.name));
        }

        // Configure speed slider
        speedSlider.onValueChanged.AddListener(OnSpeedChanged);
    }

    void ConnectToRobot()
    {
        // Establish connection to physical robot
        // This could be through ROS bridge, TCP/IP, etc.
        statusText.text = "Connecting to robot...";
    }

    void OnControlButtonClicked(string buttonName)
    {
        // Handle different control commands
        switch (buttonName)
        {
            case "MoveForwardButton":
                SendCommand("MOVE_FORWARD");
                break;
            case "MoveBackwardButton":
                SendCommand("MOVE_BACKWARD");
                break;
            case "StopButton":
                SendCommand("STOP");
                break;
        }
    }

    void OnSpeedChanged(float value)
    {
        SendCommand($"SET_SPEED:{value}");
    }

    void SendCommand(string command)
    {
        // Send command to robot (implementation depends on communication protocol)
        Debug.Log($"Sending command: {command}");
        statusText.text = $"Command sent: {command}";
    }
}
```

## Physics Simulation in Unity

### Unity vs Gazebo Physics

While Gazebo excels in accurate robotics physics, Unity provides:

- **Visual fidelity**: High-quality rendering and lighting
- **Real-time interaction**: Immediate response to user input
- **Cross-platform deployment**: Deploy to multiple platforms
- **Game-like interaction**: Intuitive user interfaces

### Physics Configuration

Configure Unity's physics system for robotics simulation:

```csharp
// PhysicsSettings.cs - Configure physics for robotics
using UnityEngine;

[CreateAssetMenu(fileName = "PhysicsSettings", menuName = "Robotics/Physics Settings")]
public class PhysicsSettings : ScriptableObject
{
    [Header("Rigidbody Settings")]
    public float defaultMass = 1.0f;
    public float drag = 0.1f;
    public float angularDrag = 0.05f;

    [Header("Collision Settings")]
    public PhysicMaterial defaultMaterial;
    public float bounceThreshold = 2.0f;
    public float sleepThreshold = 0.005f;

    [Header("Joint Settings")]
    public float jointSpring = 10000f;
    public float jointDamper = 1000f;
    public float jointForceLimit = 1000f;

    [Header("Simulation Settings")]
    public float fixedTimestep = 0.02f; // 50 Hz
    public int solverIterations = 6;
    public int solverVelocityIterations = 1;
}
```

## Sensor Simulation

### Camera Systems

Unity provides realistic camera systems for sensor simulation:

- **RGB Cameras**: Standard color cameras
- **Depth Cameras**: Generate depth maps
- **Stereo Cameras**: Simulate stereo vision
- **Thermal Cameras**: Simulate thermal imaging

### LiDAR Simulation

Implement LiDAR sensors using raycasting:

```csharp
// LidarSensor.cs - LiDAR sensor simulation
using UnityEngine;
using System.Collections.Generic;

public class LidarSensor : MonoBehaviour
{
    [Header("Lidar Configuration")]
    public int horizontalResolution = 360;
    public int verticalResolution = 1;
    public float minDistance = 0.1f;
    public float maxDistance = 10.0f;
    public float fieldOfView = 360f;

    [Header("Raycast Settings")]
    public LayerMask detectionMask = -1;
    public bool visualizeRays = true;

    private List<float> scanData;
    private LineRenderer[] rayVisualizers;

    void Start()
    {
        InitializeLidar();
    }

    void Update()
    {
        PerformScan();
    }

    void InitializeLidar()
    {
        scanData = new List<float>(horizontalResolution);
        if (visualizeRays)
        {
            rayVisualizers = new LineRenderer[horizontalResolution];
            for (int i = 0; i < horizontalResolution; i++)
            {
                GameObject rayGO = new GameObject($"LidarRay_{i}");
                rayGO.transform.SetParent(transform);
                rayGO.transform.localPosition = Vector3.zero;
                rayVisualizers[i] = rayGO.AddComponent<LineRenderer>();
                rayVisualizers[i].material = new Material(Shader.Find("Sprites/Default"));
                rayVisualizers[i].startWidth = 0.01f;
                rayVisualizers[i].endWidth = 0.01f;
            }
        }
    }

    void PerformScan()
    {
        scanData.Clear();

        for (int i = 0; i < horizontalResolution; i++)
        {
            float angle = (i * fieldOfView / horizontalResolution) * Mathf.Deg2Rad;
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxDistance, detectionMask))
            {
                scanData.Add(hit.distance);

                if (visualizeRays && rayVisualizers[i] != null)
                {
                    rayVisualizers[i].SetPositions(new Vector3[] { transform.position, hit.point });
                    rayVisualizers[i].startColor = Color.red;
                    rayVisualizers[i].endColor = Color.red;
                }
            }
            else
            {
                scanData.Add(maxDistance);

                if (visualizeRays && rayVisualizers[i] != null)
                {
                    Vector3 endPos = transform.position + direction * maxDistance;
                    rayVisualizers[i].SetPositions(new Vector3[] { transform.position, endPos });
                    rayVisualizers[i].startColor = Color.green;
                    rayVisualizers[i].endColor = Color.green;
                }
            }
        }
    }

    public float[] GetScanData()
    {
        return scanData.ToArray();
    }
}
```

## Integration with Real Robots

### ROS/ROS2 Bridge

Connect Unity digital twins to real robots using ROS bridges:

1. **Unity ROS TCP Connector**: Basic TCP/IP communication
2. **Unity Robotics Package**: Official ROS integration
3. **Custom protocols**: Implement specific communication needs

### Real-time Synchronization

Keep digital twins synchronized with physical robots:

- **State publishing**: Send robot states from Unity
- **Command receiving**: Receive commands from real systems
- **Sensor data**: Mirror real sensor readings

## Best Practices for Digital Twins

### Performance Optimization

1. **LOD Systems**: Use Level of Detail for complex models
2. **Occlusion Culling**: Hide non-visible objects
3. **Baking Lighting**: Pre-calculate static lighting
4. **Object Pooling**: Reuse frequently instantiated objects

### Fidelity vs Performance

Balance visual fidelity with real-time performance:

- **Realistic but efficient**: Use optimized meshes and textures
- **Selective detail**: Focus detail on important areas
- **Adaptive quality**: Adjust based on hardware capabilities

## Troubleshooting Common Issues

### Physics Instability

- **Timestep issues**: Use consistent fixed timesteps
- **Mass ratios**: Ensure realistic mass relationships
- **Joint limits**: Properly constrain joint movement

### Performance Problems

- **Draw calls**: Batch similar objects together
- **Shadows**: Use appropriate shadow settings
- **Post-processing**: Limit expensive effects

## Next Steps

In the next chapter, we'll explore sensor simulation and validation techniques to ensure your digital twins accurately represent real-world sensor data.