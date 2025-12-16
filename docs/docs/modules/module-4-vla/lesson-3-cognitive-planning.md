---
sidebar_position: 3
---

# Lesson 3: Cognitive Planning with LLMs and Capstone Project: The Autonomous Humanoid

## Cognitive Planning Overview

Cognitive planning in robotics involves translating high-level natural language commands into executable robotic actions. This requires reasoning about the environment, understanding object affordances, and decomposing complex tasks into primitive actions.

## LLM-Based Task Planning

Large Language Models (LLMs) excel at understanding natural language and can be used to decompose high-level commands into sequences of robotic actions.

### Basic Task Planning with LLMs

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItCommand
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json

class CognitivePlanner(Node):
    def __init__(self):
        super().__init__('cognitive_planner')
        
        # Initialize components
        self.bridge = CvBridge()
        
        # Subscriptions
        self.command_sub = self.create_subscription(
            String,
            '/natural_language_command',
            self.command_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(String, '/robot_action', 10)
        self.navigation_pub = self.create_publisher(PoseStamped, '/navigation_goal', 10)
        
        # State
        self.current_image = None
        self.llm_client = self.initialize_llm()  # Initialize your preferred LLM
        
    def initialize_llm(self):
        # Initialize your LLM client (OpenAI, Anthropic, local model, etc.)
        # For this example, we'll create a mock implementation
        class MockLLM:
            def generate_response(self, prompt):
                # This would be replaced with actual LLM call
                return self.mock_plan_response(prompt)
                
            def mock_plan_response(self, prompt):
                # Mock response for demonstration
                return json.dumps({
                    "task_decomposition": [
                        {"action": "scan_environment", "description": "Scan the room to identify objects"},
                        {"action": "identify_target", "description": "Locate the specific object to interact with"},
                        {"action": "plan_path", "description": "Calculate path to the object"},
                        {"action": "navigate", "description": "Move to the object location"},
                        {"action": "manipulate", "description": "Perform the required manipulation action"},
                        {"action": "return", "description": "Return to default position"}
                    ],
                    "object_location": [1.5, 2.0, 0.0],
                    "action_sequence": ["scan", "identify", "navigate", "manipulate"]
                })
        
        return MockLLM()
        
    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        
        if self.current_image is not None:
            self.plan_and_execute(command, self.current_image)
        else:
            self.get_logger().warn("No image available, waiting...")
            
    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def plan_and_execute(self, command, image):
        """Plan and execute the command using LLM reasoning"""
        # Create a prompt for the LLM
        prompt = self.create_planning_prompt(command, image)
        
        try:
            # Get plan from LLM
            plan_json = self.llm_client.generate_response(prompt)
            plan = json.loads(plan_json)
            
            # Execute the plan
            self.execute_plan(plan, command)
            
        except Exception as e:
            self.get_logger().error(f"Error in planning or execution: {e}")
            
    def create_planning_prompt(self, command, image):
        """Create a prompt for the LLM with context"""
        prompt = f"""
        You are a cognitive planning system for a humanoid robot. Your task is to decompose a 
        natural language command into a sequence of executable robotic actions.
        
        Command: "{command}"
        
        Current environment: [The robot has captured an image of its environment]
        
        Please provide:
        1. Task decomposition: Break down the command into logical steps
        2. Object identification: Identify the relevant objects in the environment
        3. Action sequence: The sequence of actions to execute
        4. Spatial information: Locations where actions should occur
        
        Respond in JSON format with the following structure:
        {{
            "task_decomposition": [
                {{"action": "action_type", "description": "what to do"}},
                ...
            ],
            "object_location": [x, y, z],
            "action_sequence": ["action1", "action2", ...]
        }}
        
        The robot has the following capabilities:
        - Navigation: Move to specific coordinates
        - Manipulation: Grasp and move objects
        - Perception: Detect and recognize objects
        - Interaction: Communicate with humans
        """
        
        return prompt
        
    def execute_plan(self, plan, original_command):
        """Execute the plan provided by the LLM"""
        self.get_logger().info(f"Executing plan for command: {original_command}")
        
        for step in plan["task_decomposition"]:
            action_type = step["action"]
            description = step["description"]
            
            self.get_logger().info(f"Executing: {action_type} - {description}")
            
            if action_type == "scan_environment":
                self.execute_scan()
            elif action_type == "identify_target":
                self.execute_identify_target()
            elif action_type == "plan_path":
                self.execute_plan_path()
            elif action_type == "navigate":
                self.execute_navigation(plan.get("object_location", [0, 0, 0]))
            elif action_type == "manipulate":
                self.execute_manipulation()
            elif action_type == "return":
                self.execute_return_home()
            else:
                self.get_logger().warn(f"Unknown action type: {action_type}")
    
    def execute_scan(self):
        """Execute environment scanning"""
        # Publish a command to perception system to scan environment
        cmd_msg = String()
        cmd_msg.data = "scan_environment"
        self.action_pub.publish(cmd_msg)
        
    def execute_identify_target(self):
        """Execute target object identification"""
        cmd_msg = String()
        cmd_msg.data = "identify_target_object"
        self.action_pub.publish(cmd_msg)
        
    def execute_plan_path(self):
        """Execute path planning"""
        cmd_msg = String()
        cmd_msg.data = "plan_path_to_target"
        self.action_pub.publish(cmd_msg)
        
    def execute_navigation(self, target_location):
        """Execute navigation to target location"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = target_location[0]
        goal.pose.position.y = target_location[1]
        goal.pose.position.z = target_location[2] if len(target_location) > 2 else 0.0
        goal.pose.orientation.w = 1.0
        
        self.navigation_pub.publish(goal)
        
    def execute_manipulation(self):
        """Execute manipulation action"""
        cmd_msg = String()
        cmd_msg.data = "perform_manipulation"
        self.action_pub.publish(cmd_msg)
        
    def execute_return_home(self):
        """Return to home position"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.navigation_pub.publish(goal)
```

## Capstone Project: The Autonomous Humanoid

For the capstone project, we'll integrate all the concepts learned in the previous modules to create an autonomous humanoid robot that can receive voice commands, understand them, navigate to locations, manipulate objects, and respond to its environment.

### System Architecture

```
Voice Command ("Clean the room")
    ↓
[OpenAI Whisper] → Natural Language
    ↓
[LLM Cognitive Planner] → Action Sequence
    ↓
[ROS 2 Action Execution]
    ├── [Isaac Sim] → Physics Simulation
    ├── [Navigation] → Path Planning
    ├── [Manipulation] → Object Handling
    └── [Perception] → Environment Awareness
    ↓
Physical Humanoid Robot
```

### Complete Autonomous Humanoid Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import threading
import time
import openai
import whisper
import json

class AutonomousHumanoid(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid')
        
        # Initialize interfaces
        self.bridge = CvBridge()
        
        # Voice command interface
        self.voice_command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10
        )
        
        # Perception interfaces
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Action interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.navigation_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.speech_pub = self.create_publisher(String, '/speech_output', 10)
        
        # State variables
        self.current_image = None
        self.current_scan = None
        self.current_odom = None
        self.is_executing = False
        
        # Initialize AI components
        self.whisper_model = whisper.load_model("base")
        self.llm_client = self.initialize_llm_client()
        
        # Create action servers
        self.action_lock = threading.Lock()
        
        self.get_logger().info("Autonomous Humanoid system initialized")
    
    def initialize_llm_client(self):
        """Initialize LLM client for cognitive planning"""
        # In a real implementation, this would connect to your preferred LLM
        # For this example, we'll use a mock implementation
        class MockLLMClient:
            def plan_action_sequence(self, command, image_description):
                # This would call the actual LLM
                if "clean" in command.lower():
                    return {
                        "actions": [
                            {"type": "scan", "description": "Scan the environment for objects to clean"},
                            {"type": "navigate", "params": {"x": 1.0, "y": 0.5}, "description": "Move to first object location"},
                            {"type": "manipulate", "description": "Pick up object"},
                            {"type": "navigate", "params": {"x": 0.0, "y": 0.0}, "description": "Return to start position"},
                            {"type": "manipulate", "description": "Place object in designated area"}
                        ]
                    }
                elif "bring" in command.lower() or "get" in command.lower():
                    return {
                        "actions": [
                            {"type": "identify", "description": "Identify the requested object"},
                            {"type": "navigate", "params": {"x": 2.0, "y": 1.0}, "description": "Move to object location"},
                            {"type": "manipulate", "description": "Grasp the object"},
                            {"type": "navigate", "params": {"x": 0.0, "y": 0.0}, "description": "Return to user"},
                            {"type": "manipulate", "description": "Release the object to user"}
                        ]
                    }
                else:
                    return {
                        "actions": [
                            {"type": "respond", "params": {"text": "I'm not sure how to perform that task."}}
                        ]
                    }
        
        return MockLLMClient()
    
    def voice_command_callback(self, msg):
        """Process voice command"""
        command = msg.data
        self.get_logger().info(f"Received voice command: {command}")
        
        # Use a lock to ensure only one command is processed at a time
        if self.action_lock.acquire(blocking=False):
            try:
                self.process_command(command)
            finally:
                self.action_lock.release()
        else:
            self.get_logger().warn("Command received while executing previous command, ignoring.")
    
    def image_callback(self, msg):
        """Process image data"""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.current_scan = msg
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_odom = msg
    
    def process_command(self, command):
        """Process a command with cognitive planning and execution"""
        if self.is_executing:
            self.get_logger().warn("Currently executing, cannot process new command")
            return
            
        self.is_executing = True
        self.get_logger().info(f"Processing command: {command}")
        
        try:
            # If we have an image, use it for visual context
            image_context = "Visual information is available" if self.current_image is not None else "No visual information"
            
            # Plan actions using LLM
            action_plan = self.llm_client.plan_action_sequence(command, image_context)
            
            self.get_logger().info(f"Action plan: {action_plan}")
            
            # Execute the action sequence
            for action in action_plan["actions"]:
                self.execute_action(action)
                
                # Small delay between actions
                time.sleep(0.5)
                
            # Report completion
            completion_msg = String()
            completion_msg.data = f"Completed command: {command}"
            self.speech_pub.publish(completion_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            error_msg = String()
            error_msg.data = f"Error executing command: {e}"
            self.speech_pub.publish(error_msg)
        finally:
            self.is_executing = False
    
    def execute_action(self, action):
        """Execute a single action from the planned sequence"""
        action_type = action["type"]
        description = action.get("description", "No description")
        
        self.get_logger().info(f"Executing action: {action_type} - {description}")
        
        if action_type == "navigate":
            params = action.get("params", {})
            x = params.get("x", 0.0)
            y = params.get("y", 0.0)
            self.execute_navigation(x, y)
            
        elif action_type == "manipulate":
            self.execute_manipulation()
            
        elif action_type == "scan":
            self.execute_scan()
            
        elif action_type == "identify":
            self.execute_identify()
            
        elif action_type == "respond":
            params = action.get("params", {})
            response_text = params.get("text", "Operation complete")
            response_msg = String()
            response_msg.data = response_text
            self.speech_pub.publish(response_msg)
    
    def execute_navigation(self, x, y):
        """Execute navigation to specified coordinates"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.navigation_pub.publish(goal)
        
        # Wait for navigation to complete (simplified)
        time.sleep(3)  # In a real system, wait for navigation feedback
    
    def execute_manipulation(self):
        """Execute manipulation action"""
        # In a real system, this would publish to manipulation controller
        # For now, just log the action
        self.get_logger().info("Executing manipulation action")
        
        # In a real humanoid, this might involve:
        # - Planning arm trajectory
        # - Controlling joint positions
        # - Managing grasping forces
        # For now, just simulate the action
        time.sleep(2)
    
    def execute_scan(self):
        """Execute environment scanning"""
        # Publish command to perception system to scan environment
        scan_cmd = String()
        scan_cmd.data = "scan_environment"
        self.speech_pub.publish(scan_cmd)
        
        # Simulate scan time
        time.sleep(1)
    
    def execute_identify(self):
        """Execute object identification"""
        # In a real system, this would use perception algorithms
        # to identify objects in the environment
        self.get_logger().info("Executing object identification")
        time.sleep(1)
    
    def get_robot_position(self):
        """Get current robot position from odometry"""
        if self.current_odom:
            return (
                self.current_odom.pose.pose.position.x,
                self.current_odom.pose.pose.position.y
            )
        return (0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    humanoid_node = AutonomousHumanoid()
    
    try:
        rclpy.spin(humanoid_node)
    except KeyboardInterrupt:
        humanoid_node.get_logger().info("Shutting down Autonomous Humanoid...")
    finally:
        humanoid_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## System Integration and Testing

To test the complete system, we need a launch file that brings up all components:

```xml
<!-- launch/autonomous_humanoid.launch.py -->
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Voice recognition node (using our Whisper implementation)
        Node(
            package='speech_recognition',
            executable='whisper_node',
            name='whisper_voice_node',
            output='screen'
        ),
        
        # Navigation stack
        Node(
            package='nav2_bringup',
            executable='nav2_launch.py',
            name='navigation2',
            output='screen'
        ),
        
        # Perception stack
        Node(
            package='object_detection',
            executable='object_detector',
            name='object_detector',
            output='screen'
        ),
        
        # Main cognitive planner
        Node(
            package='humanoid_control',
            executable='autonomous_humanoid',
            name='autonomous_humanoid',
            output='screen'
        ),
        
        # Isaac Sim bridge (if running simulation)
        Node(
            package='isaac_ros_bridges',
            executable='ros_bridge',
            name='isaac_ros_bridge',
            output='screen'
        )
    ])
```

## Performance Considerations

When implementing a complete VLA system:

1. **Latency**: Minimize delays between command input and action execution
2. **Reliability**: Implement fallback mechanisms for when one component fails
3. **Safety**: Include safety checks and emergency stops
4. **Resource Management**: Optimize GPU and CPU usage across all components
5. **Robustness**: Handle ambiguous commands and unexpected situations

## Evaluation Metrics

To evaluate the autonomous humanoid system:

1. **Task Completion Rate**: Percentage of tasks successfully completed
2. **Command Understanding Accuracy**: How often the LLM correctly interprets commands
3. **Navigation Success Rate**: Percentage of successful navigation attempts
4. **Response Time**: Average time from command to first action
5. **Safety Incidents**: Number of collisions or unsafe behaviors

## Conclusion

This capstone project combines all the concepts from the previous modules:

- **Module 1 (ROS 2)**: Provides the communication backbone and node architecture
- **Module 2 (Gazebo/Unity)**: Enables simulation and testing in virtual environments
- **Module 3 (Isaac)**: Provides AI processing and perception capabilities
- **Module 4 (VLA)**: Integrates voice, vision, and action for natural interaction

The autonomous humanoid represents a complete robotic system where natural language commands are understood, planned, and executed in the physical world. This integration demonstrates the power of combining multiple advanced technologies to create capable, intelligent robotic systems.