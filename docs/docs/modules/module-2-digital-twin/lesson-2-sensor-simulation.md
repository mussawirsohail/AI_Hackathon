---
sidebar_position: 2
---

# Lesson 2: Simulating Sensors in Gazebo and Unity

## Sensor Simulation Overview

Accurate sensor simulation is crucial for developing and testing robotic systems. In digital twin environments like Gazebo and Unity, we can simulate various sensors with realistic noise models and environmental effects.

## LiDAR Simulation in Gazebo

LiDAR (Light Detection and Ranging) sensors are essential for navigation and mapping. Simulating LiDAR in Gazebo involves configuring ray sensors that mimic real LiDAR behavior.

### LiDAR Configuration in SDF

```xml
<sensor name="lidar_sensor" type="ray">
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_frame</frame_name>
  </plugin>
</sensor>
```

### Accessing LiDAR Data in ROS 2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

    def lidar_callback(self, msg):
        # Process LiDAR data
        ranges = msg.ranges
        min_distance = min(ranges)
        self.get_logger().info(f'Min distance: {min_distance}')
```

## Depth Camera Simulation

Depth cameras provide both color and depth information, which is valuable for 3D environment mapping and object recognition.

### Depth Camera Configuration in SDF

```xml
<sensor name="depth_camera" type="depth">
  <pose>0.1 0 0.1 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>30</update_rate>
  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <baseline>0.2</baseline>
    <always_on>true</always_on>
    <update_rate>30.0</update_rate>
    <camera_name>camera</camera_name>
    <image_topic_name>rgb/image_raw</image_topic_name>
    <depth_image_topic_name>depth/image_raw</depth_image_topic_name>
    <point_cloud_topic_name>depth/points</point_cloud_topic_name>
    <camera_info_topic_name>rgb/camera_info</camera_info_topic_name>
    <depth_image_camera_info_topic_name>depth/camera_info</depth_image_camera_info_topic_name>
    <frame_name>camera_depth_optical_frame</frame_name>
    <point_cloud_cutoff>0.5</point_cloud_cutoff>
    <point_cloud_cutoff_max>3.0</point_cloud_cutoff_max>
    <Cx>0</Cx>
    <Cy>0</Cy>
    <hack_baseline>0</hack_baseline>
  </plugin>
</sensor>
```

### Processing Depth Camera Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthCameraSubscriber(Node):
    def __init__(self):
        super().__init__('depth_camera_subscriber')
        self.bridge = CvBridge()
        
        # Subscribe to depth image
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10
        )

    def depth_image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Process depth information
        distance_at_center = cv_image[int(cv_image.shape[0]/2), int(cv_image.shape[1]/2)]
        self.get_logger().info(f'Distance at center pixel: {distance_at_center} meters')
```

## IMU Simulation

Inertial Measurement Units (IMUs) provide information about acceleration, angular velocity, and orientation.

### IMU Configuration in SDF

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <visualize>false</visualize>
  <topic>__default_topic__</topic>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>/imu</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <sensor>imu_sensor</sensor>
    <gaussian_noise>0.00057</gaussian_noise>
    <frame_name>imu_link</frame_name>
  </plugin>
</sensor>
```

### IMU Data Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # Extract orientation quaternion
        orientation_q = msg.orientation
        # Convert to Euler angles
        (roll, pitch, yaw) = self.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        
        self.get_logger().info(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')

    def euler_from_quaternion(self, quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
```

## Unity Sensor Simulation

Unity provides high-fidelity rendering and can simulate sensors through custom scripts and plugins. Unity Robotics provides packages for sensor simulation and integration with ROS.

### Unity LiDAR Simulation

```csharp
using UnityEngine;
using System.Collections.Generic;

public class UnityLidar : MonoBehaviour
{
    public int rays = 360;
    public float range = 10f;
    public float angle = 360f;
    public LayerMask layerMask;

    void Update()
    {
        var hits = new List<float>();
        float angleStep = angle / rays;

        for (int i = 0; i < rays; i++)
        {
            float currentAngle = transform.eulerAngles.y + i * angleStep;
            Vector3 direction = Quaternion.Euler(0, currentAngle, 0) * transform.forward;

            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, range, layerMask))
            {
                hits.Add(hit.distance);
            }
            else
            {
                hits.Add(range);
            }
        }

        // Process LiDAR data (send via ROS or use internally)
        ProcessLidarData(hits);
    }

    void ProcessLidarData(List<float> distances)
    {
        // Send distances to ROS bridge or use in Unity
    }
}
```

## Sensor Fusion in Simulation

Combining data from multiple sensors improves perception accuracy:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import Pose
from tf2_ros import TransformBroadcaster
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Initialize sensor data storage
        self.lidar_data = None
        self.imu_data = None
        self.camera_data = None
        
        # Create subscribers for each sensor
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10)

    def lidar_callback(self, msg):
        self.lidar_data = msg.ranges

    def imu_callback(self, msg):
        self.imu_data = msg.orientation

    def camera_callback(self, msg):
        # Process camera data and combine with other sensors
        pass

    def sensor_fusion_algorithm(self):
        # Algorithm to combine sensor data for state estimation
        # This is a simplified example
        if self.lidar_data and self.imu_data:
            # Example: combine LiDAR obstacle detection with IMU orientation
            # to improve navigation decisions
            pass
```

Continue to [Lesson 3: Unity Integration and High-Fidelity Rendering](./lesson-3-unity-integration.md) to explore advanced rendering and human-robot interaction in Unity.