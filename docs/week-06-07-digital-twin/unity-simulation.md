---
sidebar_position: 2
title: Unity for Robotics
description: Using Unity for high-fidelity robot simulation and synthetic data generation
keywords: [unity, simulation, ros2, robotics, synthetic data, rendering]
---

# Unity for Robotics

<div className="chapter-meta">
  <div className="chapter-meta-item">
    <strong>Week:</strong> 6-7
  </div>
  <div className="chapter-meta-item">
    <strong>Duration:</strong> ~3 hours
  </div>
  <div className="chapter-meta-item">
    <strong>Level:</strong> Intermediate
  </div>
</div>

## Learning Objectives

<div className="learning-objectives">

By the end of this chapter, you will be able to:

- Understand when to use Unity vs. Gazebo for robotics
- Set up Unity with the Robotics packages
- Import robot models and create simulation scenes
- Generate photorealistic synthetic data for ML training
- Connect Unity simulations to ROS 2

</div>

## Unity vs. Gazebo

| Feature | Unity | Gazebo |
|---------|-------|--------|
| **Rendering** | High-fidelity, GPU-accelerated | Basic to moderate |
| **Physics** | PhysX, configurable | ODE, Bullet, DART |
| **ROS Integration** | Via packages | Native |
| **Learning Curve** | Steeper | Moderate |
| **Best For** | Visual AI, synthetic data | Control, navigation |

## Unity Robotics Hub

Unity provides official robotics packages:

- **ROS TCP Connector**: Communication with ROS/ROS 2
- **URDF Importer**: Import robot models
- **Perception**: Synthetic data generation
- **Navigation**: Pathfinding and mapping

### Installation

1. Install Unity Hub and Unity 2021.3+ LTS
2. Create a new 3D project
3. Add packages via Package Manager:
   - ROS TCP Connector
   - URDF Importer
   - Perception

## ROS 2 Integration

### Setting Up Communication

```csharp
// Unity C# - ROS Connection
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/cmd_vel";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void PublishVelocity(float linear, float angular)
    {
        var msg = new TwistMsg
        {
            linear = new Vector3Msg { x = linear },
            angular = new Vector3Msg { z = angular }
        };
        ros.Publish(topicName, msg);
    }
}
```

### Subscribing to Topics

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraSubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance()
            .Subscribe<CompressedImageMsg>("/camera/image", OnImageReceived);
    }

    void OnImageReceived(CompressedImageMsg msg)
    {
        // Process image data
        byte[] imageData = msg.data;
        // Convert to Unity texture...
    }
}
```

## Synthetic Data Generation

Unity Perception package enables:

- **Domain Randomization**: Vary lighting, textures, positions
- **Labeling**: Automatic bounding boxes, segmentation masks
- **Export**: COCO, KITTI, and custom formats

### Example: Object Detection Dataset

```csharp
using UnityEngine.Perception.Randomization.Scenarios;
using UnityEngine.Perception.Randomization.Randomizers;

public class ObjectDetectionScenario : FixedLengthScenario
{
    protected override void OnIterationStart()
    {
        // Randomize scene for each iteration
        RandomizeLighting();
        RandomizeObjectPositions();
        RandomizeTextures();
    }
}
```

## Summary

Unity excels at:

- Photorealistic rendering for visual AI
- Large-scale synthetic data generation
- Complex visual environments
- GPU-accelerated simulation

Use Unity when visual fidelity matters; use Gazebo for control-focused development.

## Exercises

1. **Setup**: Install Unity and create a basic robotics project with ROS TCP Connector.

2. **Import**: Import a URDF robot model into Unity.

3. **Synthetic Data**: Generate 1000 images of a robot in randomized environments.

4. **Integration**: Control a Unity-simulated robot from a ROS 2 node.
