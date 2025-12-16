---
sidebar_position: 3
---

# Lesson 3: Unity Integration and High-Fidelity Rendering

## Introduction to Unity for Robotics

Unity is a powerful game engine that has been increasingly adopted in robotics for high-fidelity simulation, visualization, and human-robot interaction prototyping. Unlike Gazebo which is primarily focused on physics simulation, Unity excels at creating photorealistic environments and user interfaces.

## Unity ROS Integration

Unity Robotics provides packages that enable communication with ROS 2, allowing you to use Unity as a visualization layer for your robotic systems or as a high-fidelity simulation environment.

### Setting up Unity with ROS 2

To connect Unity to ROS 2, you can use the Unity Robotics Hub which includes:

- ROS TCP Connector: For communication between Unity and ROS 2
- Unity Robotics Packages: For robotics-specific functionality
- Sample projects and tutorials

### Basic ROS Connection in Unity

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UnityRosConnection : MonoBehaviour
{
    ROSConnection ros;
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Initialize(rosIPAddress, rosPort);
    }

    void Update()
    {
        // Send data to ROS
        StringMsg message = new StringMsg("Hello from Unity!");
        ros.Send("unity_topic", message);
    }

    void OnMessageReceived(StringMsg msg)
    {
        Debug.Log("Received from ROS: " + msg.data);
    }
}
```

## High-Fidelity Rendering in Unity

Unity's rendering capabilities enable the creation of photorealistic environments that can be used for synthetic data generation and testing perception algorithms.

### Physically Based Rendering (PBR)

Unity's PBR system accurately simulates how light interacts with surfaces:

```csharp
// Adjust material properties for realistic rendering
void SetRealisticMaterial(GameObject obj, float metalness, float smoothness)
{
    Material material = obj.GetComponent<Renderer>().material;
    material.SetFloat("_Metallic", metalness);
    material.SetFloat("_Smoothness", smoothness);
}
```

### Lighting Setup for Photorealism

```csharp
public class LightingSetup : MonoBehaviour
{
    public Light sunLight;
    public ReflectionProbe reflectionProbe;

    void Start()
    {
        // Configure directional light to simulate sun
        sunLight.type = LightType.Directional;
        sunLight.intensity = 1.0f;
        sunLight.color = Color.white;
        sunLight.transform.rotation = Quaternion.Euler(50, -30, 0);

        // Update reflection probe for realistic reflections
        reflectionProbe.RenderProbe();
    }
}
```

## Human-Robot Interaction (HRI) in Unity

Unity is ideal for prototyping HRI interfaces, allowing you to create interactive environments where humans can interact with virtual robots.

### Creating Interactive Elements

```csharp
using UnityEngine;
using UnityEngine.UI;

public class HriInterface : MonoBehaviour
{
    public Button moveForwardButton;
    public Button turnLeftButton;
    public Button turnRightButton;

    void Start()
    {
        moveForwardButton.onClick.AddListener(() => SendCommand("move_forward"));
        turnLeftButton.onClick.AddListener(() => SendCommand("turn_left"));
        turnRightButton.onClick.AddListener(() => SendCommand("turn_right"));
    }

    void SendCommand(string command)
    {
        // Send command to ROS via TCP connector
        ROSConnection ros = ROSConnection.instance;
        ros.Send("robot_command", new StringMsg(command));
    }
}
```

### Voice Command Integration

```csharp
using UnityEngine;
using System.Collections;

public class VoiceCommand : MonoBehaviour
{
    // This would integrate with a speech recognition system
    // or connect to a ROS voice recognition node

    void ProcessVoiceCommand(string command)
    {
        switch (command.ToLower())
        {
            case "move forward":
                SendCommandToRobot("move_forward");
                break;
            case "turn left":
                SendCommandToRobot("turn_left");
                break;
            case "stop":
                SendCommandToRobot("stop");
                break;
        }
    }

    void SendCommandToRobot(string command)
    {
        ROSConnection ros = ROSConnection.instance;
        ros.Send("voice_command", new StringMsg(command));
    }
}
```

## Unity vs Gazebo: When to Use Each

### Use Gazebo when:
- Accurate physics simulation is critical
- Testing control algorithms
- Working with standard robot models
- Performance is a concern for large-scale simulations

### Use Unity when:
- High-quality visual rendering is needed
- Creating user interfaces and HRI prototypes
- Generating synthetic training data for machine learning
- Developing augmented reality interfaces

## Unity Gazebo Bridge

For the best of both worlds, you can connect Unity and Gazebo:

- Use Gazebo for physics simulation
- Use Unity for visualization
- Synchronize state between both environments

```csharp
// Example of synchronizing robot state between Gazebo and Unity
public class RobotStateSynchronizer : MonoBehaviour
{
    public GameObject robotModel;
    ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<OdometryMsg>("/robot/odom", UpdateRobotPosition);
    }

    void UpdateRobotPosition(OdometryMsg odom)
    {
        // Update Unity robot position based on Gazebo simulation
        Vector3 position = new Vector3(
            (float)odom.pose.pose.position.x,
            (float)odom.pose.pose.position.y,
            (float)odom.pose.pose.position.z
        );
        
        Quaternion rotation = new Quaternion(
            (float)odom.pose.pose.orientation.x,
            (float)odom.pose.pose.orientation.y,
            (float)odom.pose.pose.orientation.z,
            (float)odom.pose.pose.orientation.w
        );
        
        robotModel.transform.position = position;
        robotModel.transform.rotation = rotation;
    }
}
```

## Creating Synthetic Training Data

Unity's rendering capabilities make it ideal for generating synthetic data for training machine learning models:

```csharp
using UnityEngine;

public class SyntheticDataGenerator : MonoBehaviour
{
    public Camera sensorCamera;
    public int datasetSize = 1000;
    private int currentSample = 0;

    void Start()
    {
        StartCoroutine(GenerateDataset());
    }

    IEnumerator GenerateDataset()
    {
        while (currentSample < datasetSize)
        {
            // Move objects to random positions
            MoveSceneObjectsRandomly();
            
            // Capture image from sensor camera
            yield return new WaitForEndOfFrame();
            CaptureImageAndDepth(currentSample);
            
            currentSample++;
            yield return new WaitForSeconds(0.5f); // Wait between captures
        }
    }

    void CaptureImageAndDepth(int index)
    {
        // Capture RGB image
        Texture2D rgbImage = CaptureCameraImage(sensorCamera);
        SaveImage(rgbImage, $"rgb_{index}.png");

        // Capture depth image using depth shader
        Texture2D depthImage = CaptureDepthImage(sensorCamera);
        SaveImage(depthImage, $"depth_{index}.png");
    }

    Texture2D CaptureCameraImage(Camera cam)
    {
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;

        cam.Render();
        Texture2D image = new Texture2D(cam.targetTexture.width, cam.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);

        RenderTexture.active = currentRT;
        return image;
    }
}
```

In the next module, we'll explore the AI-Robot Brain using NVIDIA Isaac, which provides advanced perception and training capabilities that can be enhanced with the synthetic data generation techniques we've discussed.