---
sidebar_position: 3
---

# Lesson 3: Isaac ROS and Nav2: Path Planning for Bipedal Humanoid Movement

## Introduction to Navigation in Humanoid Robotics

Navigation for bipedal humanoid robots presents unique challenges compared to wheeled robots. Humanoids must consider dynamic stability, balance, footstep planning, and complex kinematics. This lesson covers how Isaac ROS and Nav2 can be adapted for humanoid navigation.

## Overview of Nav2 Architecture

Nav2 (Navigation 2) is the navigation stack for ROS 2, providing path planning, obstacle avoidance, and motion control. For humanoid robots, Nav2 requires special considerations:

- Footstep planning instead of simple pose control
- Dynamic balance constraints
- Complex kinematic chains
- Multi-contact planning

### Standard Nav2 Components

```yaml
# Example Nav2 configuration for humanoid robot
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
```

## Isaac ROS Navigation Acceleration

Isaac ROS enhances Nav2 with hardware acceleration for perception and planning tasks:

### Isaac ROS Perception for Navigation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import numpy as np

class IsaacHumanoidNavigator(Node):
    def __init__(self):
        super().__init__('isaac_humanoid_navigator')
        
        # Isaac-accelerated perception
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_rect_color',
            self.rgb_callback,
            10
        )
        
        # Navigation publishers
        self.path_pub = self.create_publisher(Path, '/humanoid_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF listener for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize Isaac perception backend
        self.initialize_isaac_perception()
        
        # State variables
        self.current_goal = None
        self.current_path = None
        
    def initialize_isaac_perception(self):
        self.get_logger().info('Initializing Isaac-accelerated perception')
        # Initialize Isaac perception pipelines here
        
    def depth_callback(self, msg):
        # Process depth data using Isaac-accelerated pipelines
        # This would use Isaac's CUDA-accelerated depth processing
        pass
        
    def rgb_callback(self, msg):
        # Process RGB data for obstacle detection using Isaac
        # This would use Isaac's accelerated computer vision
        pass
        
    def plan_humanoid_path(self, start_pose, goal_pose):
        # Custom path planning for humanoid considering:
        # 1. Reachable space for bipedal movement
        # 2. Balance constraints
        # 3. Footstep planning
        path = self.compute_footstep_path(start_pose, goal_pose)
        return path
        
    def compute_footstep_path(self, start_pose, goal_pose):
        # Simplified footstep planner (in practice, this would be more complex)
        # This would integrate with actual humanoid kinematics
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Calculate intermediate waypoints
        steps = 10  # Number of footsteps
        step_size = 0.3  # Distance between footsteps in meters
        
        start_pos = np.array([start_pose.pose.position.x, start_pose.pose.position.y])
        goal_pos = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y])
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)
        unit_direction = direction / distance
        
        # Generate footstep positions
        for i in range(steps):
            ratio = i / (steps - 1) if steps > 1 else 0
            pos = start_pos + ratio * (goal_pos - start_pos)
            
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = float(pos[0])
            pose_stamped.pose.position.y = float(pos[1])
            pose_stamped.pose.position.z = 0.0  # Ground level
            
            # Set orientation
            pose_stamped.pose.orientation = start_pose.pose.orientation
            
            path.poses.append(pose_stamped)
            
        return path
```

## Footstep Planning for Bipedal Robots

Humanoid robots require specialized planning that considers foot placement and balance:

### Footstep Planner Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
import numpy as np

class FootstepPlanner(Node):
    def __init__(self):
        super().__init__('footstep_planner')
        
        self.footstep_pub = self.create_publisher(Marker, '/footsteps', 10)
        self.goal_sub = self.create_subscription(
            Pose,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Humanoid-specific parameters
        self.step_length = 0.3  # meters
        self.step_width = 0.2   # meters (distance between feet)
        self.max_step_turn = 0.5  # radians
        
    def goal_callback(self, goal_pose):
        # Plan footsteps from current position to goal_pose
        footsteps = self.plan_footsteps(goal_pose)
        self.visualize_footsteps(footsteps)
        
    def plan_footsteps(self, goal_pose):
        # Calculate footstep sequence to reach goal
        # This is a simplified example
        footsteps = []
        
        # For demonstration, assume current position is at origin
        current_pos = Point()
        current_pos.x = 0.0
        current_pos.y = 0.0
        current_pos.z = 0.0
        
        # Calculate distance and direction to goal
        dx = goal_pose.position.x - current_pos.x
        dy = goal_pose.position.y - current_pos.y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Calculate number of steps needed
        num_steps = int(distance / self.step_length) + 1
        
        for i in range(num_steps):
            ratio = i / num_steps if num_steps > 0 else 0
            
            # Calculate step position (with zigzag pattern for bipedal)
            step_x = current_pos.x + ratio * dx
            step_y = current_pos.y + ratio * dy
            
            # Add slight zigzag to simulate bipedal movement
            if i % 2 == 0:
                step_y += self.step_width / 2  # Right foot
            else:
                step_y -= self.step_width / 2  # Left foot
                
            foot_pose = Pose()
            foot_pose.position.x = step_x
            foot_pose.position.y = step_y
            foot_pose.position.z = 0.0  # Ground level
            footsteps.append(foot_pose)
            
        return footsteps
        
    def visualize_footsteps(self, footsteps):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "footsteps"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set the frame ID and timestamp
        marker.pose.orientation.w = 1.0
        
        # Set the scale of the marker
        marker.scale.x = 0.02  # Line width
        
        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Add points for each footstep
        for foot_pose in footsteps:
            marker.points.append(foot_pose.position)
            
        self.footstep_pub.publish(marker)
```

## Integration with Isaac Sim for Humanoid Navigation

Simulating humanoid navigation requires detailed physics and collision models:

### Isaac Sim Humanoid Navigation Setup

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

class IsaacHumanoidSimulation:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.humanoid_robot = None
        self.setup_environment()
        
    def setup_environment(self):
        # Get Isaac assets
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print("Could not find Isaac Sim assets")
            return
            
        # Add a humanoid robot (using a generic robot for now)
        # In practice, this would be a detailed humanoid model
        add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Robots/Carter/carter_nucleus.usd",
            prim_path="/World/Robot"
        )
        
        # Add obstacles
        obstacle1 = DynamicCuboid(
            prim_path="/World/obstacle1",
            name="obstacle1",
            position=np.array([1.0, 1.0, 0.25]),
            size=0.5,
            color=np.array([0.8, 0.2, 0.1])
        )
        
        obstacle2 = DynamicCuboid(
            prim_path="/World/obstacle2",
            name="obstacle2", 
            position=np.array([2.0, -1.0, 0.25]),
            size=0.5,
            color=np.array([0.2, 0.8, 0.1])
        )
        
        # Add ground plane
        self.world.scene.add_ground_plane("ground", 
                                          static_friction=0.7,
                                          dynamic_friction=0.5,
                                          restitution=0.8)
        
        # Initialize the world
        self.world.reset()
        
    def run_navigation_simulation(self):
        # This would run a complete navigation task
        # The humanoid would plan a path and execute it
        for i in range(1000):  # Run for 1000 simulation steps
            self.world.step(render=True)
            
            # At some interval, check for navigation commands
            if i % 100 == 0:
                self.execute_navigation_step()
                
    def execute_navigation_step(self):
        # This would interface with the ROS navigation stack
        # that's running in the simulation
        pass
```

## Nav2 Configuration for Humanoid Robots

Nav2 needs specific configuration for humanoid movement patterns:

```yaml
# Example Nav2 configuration for a humanoid robot
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3  # Radius of the humanoid robot
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.5  # Inflate humanoids need more space
        cost_scaling_factor: 3.0
        
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.5
        cost_scaling_factor: 3.0

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"] 
    controller_plugins: ["FollowPath"]
    
    # Humanoid-specific controllers would go here
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      no_samples: 100
      motion_model: "DiffDrive"
      x_std: 0.1
      y_std: 0.1
      theta_std: 0.2
      control_duration: 0.1
      cmd_vel_limits: ["-0.5", "0.5", "-0.2", "0.2"]
      transform_tolerance: 0.3
      xy_goal_tolerance: 0.25  # More tolerance for humanoid
      yaw_goal_tolerance: 0.25
      state_bounds_warning: ["-1", "1", "-1", "1", "-1.57", "1.57", "-1.57", "1.57", "-1.57", "1.57", "-3", "3", "-3", "3", "-3", "3"]
```

## Isaac ROS Hardware Acceleration

Isaac ROS packages are optimized for GPU acceleration:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from geometry_msgs.msg import Point
import numpy as np

class IsaacAcceleratedPerception(Node):
    def __init__(self):
        super().__init__('isaac_accelerated_perception')
        
        # Subscribe to camera feed
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_rect_color',
            self.image_callback,
            10
        )
        
        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/isaac_detections',
            10
        )
        
        # Initialize Isaac accelerated perception models
        self.initialize_detection_model()
        
    def initialize_detection_model(self):
        # This would initialize Isaac's hardware-accelerated detection models
        # using NVIDIA TensorRT or similar
        self.get_logger().info("Isaac Accelerated Perception initialized")
        
    def image_callback(self, msg):
        # Process image using Isaac's accelerated pipelines
        # This would leverage NVIDIA GPUs for acceleration
        detections = self.detect_objects(msg)
        
        # Publish results
        detection_msg = self.create_detection_message(detections)
        self.detection_pub.publish(detection_msg)
        
    def detect_objects(self, image_msg):
        # This would use Isaac's optimized detection algorithms
        # For demonstration, returning mock detections
        detections = [
            {'class': 'obstacle', 'confidence': 0.95, 'bbox': [100, 100, 50, 50]},
            {'class': 'path', 'confidence': 0.89, 'bbox': [200, 150, 80, 80]}
        ]
        return detections
        
    def create_detection_message(self, detections):
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "camera_rgb_optical_frame"
        
        for detection in detections:
            d = Detection2D()
            d.bbox.size_x = detection['bbox'][2]
            d.bbox.size_y = detection['bbox'][3]
            d.bbox.center.x = detection['bbox'][0] + detection['bbox'][2]/2
            d.bbox.center.y = detection['bbox'][1] + detection['bbox'][3]/2
            d.results = [{'score': detection['confidence'], 'class': detection['class']}]
            detection_array.detections.append(d)
            
        return detection_array
```

In the next module, we'll explore Vision-Language-Action (VLA) systems that combine perception, natural language processing, and action execution to create truly intelligent robotic systems.