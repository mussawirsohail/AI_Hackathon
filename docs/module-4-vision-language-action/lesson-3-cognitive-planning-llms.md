---
title: Lesson 3 - Cognitive Planning with LLMs
sidebar_position: 4
description: Using Large Language Models to translate natural language into robotic actions
learning_objectives:
  - Implement LLM-based cognitive planning for robotic tasks
  - Design prompt engineering techniques for robotic applications
  - Integrate LLMs with ROS 2 action planning systems
  - Validate and verify LLM-generated action sequences
duration: 180
---

# Lesson 3 - Cognitive Planning with LLMs

## Learning Objectives

After completing this lesson, you will be able to:
- Implement LLM-based cognitive planning for robotic tasks
- Design prompt engineering techniques for robotic applications
- Integrate LLMs with ROS 2 action planning systems
- Validate and verify LLM-generated action sequences

## Introduction

Large Language Models (LLMs) have emerged as powerful tools for cognitive planning in robotics, enabling robots to interpret natural language commands and decompose them into executable action sequences. For humanoid robots, LLMs serve as the cognitive bridge between high-level human instructions and low-level robotic actions, providing the reasoning capabilities needed to navigate complex real-world tasks. This lesson covers implementing LLM-driven planning systems with a focus on safety, accuracy, and real-world applicability.

## LLM Architecture for Robotics

### Language Models in Robotics

Modern LLMs for robotics typically include:

- **Transformer Architecture**: Self-attention mechanisms for understanding context
- **Multimodal Capabilities**: Integration of vision and language models
- **Instruction Following**: Training to follow complex multi-step instructions
- **Chain-of-Thought Reasoning**: Step-by-step reasoning for complex tasks

### Robotic-Specific Models

Specialized models for robotics include:

- **RT-2 (Robotics Transformer 2)**: Maps language to robot actions
- **Instruct2Act**: Translates natural language to robotic actions
- **SayCan**: Combines language understanding with action selection
- **VoxPoser**: Vision-language reasoning for manipulation

## Prompt Engineering for Robotics

### System Prompt Design

Designing effective system prompts for robotic applications:

```python
SYSTEM_PROMPT = """
You are a robotic planning assistant. You will receive natural language commands
and must generate step-by-step plans for a humanoid robot to execute.

Robot capabilities:
- Navigate to locations in the environment
- Detect and identify objects
- Manipulate objects (pick up, place, hand over)
- Open/close doors and drawers
- Turn devices on/off
- Communicate with humans through speech

Output format:
<thinking>
Break down the command and consider possible approaches.
Consider safety, feasibility, and environmental constraints.
</thinking>

<action_sequence>
1. [Action 1]
2. [Action 2]
3. [Action 3]
...
</action_sequence>

Available actions:
- NAVIGATE_TO(location)
- DETECT_OBJECT(object_type)
- GRASP_OBJECT(object_name)
- PLACE_OBJECT(location)
- OPEN_CONTAINER(container_name)
- CLOSE_CONTAINER(container_name)
- TURN_ON(device_name)
- TURN_OFF(device_name)
- SPEAK(text)
- WAIT_FOR(condition)
"""
```

### Few-Shot Examples

Providing examples to guide LLM behavior:

```python
FEW_SHOT_EXAMPLES = [
    {
        "input": "Go to the kitchen and bring me a cup of coffee.",
        "output": """
<thinking>
The user wants a cup of coffee from the kitchen.
1. I need to navigate to the kitchen
2. Find a cup
3. Find coffee (or make coffee if possible)
4. Pick up the cup with coffee
5. Navigate back to the user
The robot likely cannot make coffee, so I'll assume there's already coffee available.
</thinking>

<action_sequence>
1. NAVIGATE_TO(kitchen)
2. DETECT_OBJECT(cup)
3. DETECT_OBJECT(coffee)
4. GRASP_OBJECT(cup)
5. PLACE_OBJECT(user_location)
</action_sequence>
        """
    },
    {
        "input": "Clean the table.",
        "output": """
<thinking>
The user wants the table cleaned. This likely means removing objects from the table.
1. I need to identify objects on the table
2. Determine where to place these objects
3. Remove objects systematically
4. Check if cleaning is complete
</thinking>

<action_sequence>
1. NAVIGATE_TO(table)
2. DETECT_OBJECTS(table_surface)
3. FOR each object in detected_objects:
   - DETERMINE appropriate_storage_location(object_type)
   - GRASP_OBJECT(object)
   - PLACE_OBJECT(storage_location)
4. SPEAK(text="The table has been cleaned")
</action_sequence>
        """
    }
]
```

### Context-Aware Prompting

Incorporating environmental context into prompts:

- **Current State**: Robot's location, battery level, recent actions
- **World Knowledge**: Object locations, room layout, affordances
- **User Context**: Preferences, previous interactions, current activity
- **Safety Constraints**: Forbidden actions, hazardous areas

## Integration with ROS 2 Planning Systems

### Action Space Mapping

Mapping LLM outputs to ROS 2 action sequences:

```python
ACTION_MAP = {
    "NAVIGATE_TO": {
        "ros_action": "nav2_msgs/action/NavigateToPose",
        "params": ["x", "y", "theta", "frame_id"],
        "description": "Navigate to a specific pose in the environment"
    },
    "GRASP_OBJECT": {
        "ros_action": "manipulation_msgs/action/Grasp",
        "params": ["object_name", "grasp_type"],
        "description": "Grasp an object with specified grasp type"
    },
    "SPEAK": {
        "ros_action": "sound_play_msgs/action/Speak",
        "params": ["text"],
        "description": "Speak text using text-to-speech"
    }
}
```

### Planning Interface

Creating a planning interface for LLM integration:

```python
import rospy
from llm_planner_msgs.srv import PlanFromNaturalLanguage
from llm_planner_msgs.msg import LLMAction
from std_msgs.msg import String

class LLMPlannerNode:
    def __init__(self):
        rospy.init_node('llm_planner_node')
        
        # Service for receiving natural language commands
        self.plan_service = rospy.Service(
            'plan_from_natural_language',
            PlanFromNaturalLanguage,
            self.plan_callback
        )
        
        # Publisher for action sequences
        self.action_pub = rospy.Publisher(
            'planned_actions',
            LLMAction,
            queue_size=10
        )
        
        # Publisher for status updates
        self.status_pub = rospy.Publisher(
            'planner_status',
            String,
            queue_size=10
        )
        
        # Initialize LLM client
        self.llm_client = self.initialize_llm_client()
        
        rospy.loginfo("LLM Planner node initialized")

    def plan_callback(self, req):
        """Process natural language command and generate action plan"""
        try:
            # Format the command for LLM
            formatted_command = self.format_command(
                req.natural_language_command,
                req.context_info
            )
            
            # Generate plan using LLM
            plan = self.generate_plan_with_llm(formatted_command)
            
            # Validate the plan for safety and feasibility
            validated_plan = self.validate_plan(plan)
            
            # Publish the action sequence
            for action in validated_plan:
                self.action_pub.publish(action)
            
            # Return success response
            return PlanFromNaturalLanguageResponse(True, validated_plan)
            
        except Exception as e:
            rospy.logerr(f"Error generating plan: {e}")
            return PlanFromNaturalLanguageResponse(False, [])
    
    def format_command(self, command, context):
        """Format command with context for LLM"""
        return f"""
Current context: {context}
User command: {command}

Generate a step-by-step plan for the robot to execute this command.
        """
    
    def generate_plan_with_llm(self, formatted_command):
        """Generate plan using LLM"""
        response = self.llm_client.generate(formatted_command)
        return self.parse_llm_response(response)
    
    def validate_plan(self, plan):
        """Validate plan for safety and feasibility"""
        validated_plan = []
        for action in plan:
            if self.is_action_safe(action) and self.is_action_feasible(action):
                validated_plan.append(action)
            else:
                rospy.logwarn(f"Invalid action removed: {action}")
        
        return validated_plan
```

## Safety and Validation

### Safety Constraints

Implementing safety checks for LLM-generated actions:

- **Physical Safety**: Avoiding collisions and dangerous movements
- **Social Safety**: Respecting personal space and privacy
- **Operational Safety**: Preventing damage to robot or environment
- **Ethical Safety**: Following ethical guidelines and user preferences

### Validation Techniques

- **Rule-Based Checking**: Validate against predefined safety rules
- **Simulation Verification**: Test action sequences in simulation
- **Human-in-the-Loop**: Require human approval for complex plans
- **Gradual Execution**: Execute plans step-by-step with monitoring

### Example Safety Validation

```python
def validate_action(action, robot_state, world_model):
    """Validate an action for safety and feasibility"""
    
    # Check physical safety
    if action.type == "NAVIGATE_TO":
        path = plan_path(robot_state.location, action.params["target"])
        if path_has_obstacles(path):
            return False, "Path contains obstacles"
    
    # Check operational constraints
    if action.type == "GRASP_OBJECT":
        obj = get_object_info(action.params["object_name"])
        if obj.weight > robot_max_load:
            return False, "Object too heavy to grasp"
    
    # Check social constraints
    if action.type == "NAVIGATE_TO":
        target = action.params["target"]
        if is_in_personal_space(target, humans_in_environment):
            return False, "Target location in personal space"
    
    return True, "Action is safe and feasible"
```

## Cognitive Planning Strategies

### Hierarchical Planning

Breaking down complex tasks into manageable subtasks:

1. **Task Decomposition**: Breaking high-level commands into subtasks
2. **Subtask Orchestration**: Coordinating execution of subtasks
3. **Failure Recovery**: Handling failures in subtasks
4. **Resource Management**: Allocating resources across subtasks

### Contextual Reasoning

Using environmental and situational context:

- **Spatial Reasoning**: Understanding spatial relationships
- **Temporal Reasoning**: Understanding time-based constraints
- **Social Reasoning**: Understanding social conventions
- **Physical Reasoning**: Understanding object properties and physics

### Learning from Interaction

Improving planning through experience:

- **Success/Failure Analysis**: Learning from plan outcomes
- **User Feedback Integration**: Incorporating user corrections
- **Adaptive Planning**: Adjusting planning strategies based on context
- **Knowledge Accumulation**: Building a knowledge base of successful plans

## Integration with Perception Systems

### Real-Time Perception Integration

Combining LLM planning with real-time perception:

- **Object Recognition**: Identifying objects during plan execution
- **Localization**: Updating robot position during navigation
- **Scene Understanding**: Interpreting environment changes
- **Human Detection**: Recognizing human presence and behavior

### Feedback Loops

Creating feedback between LLM and perception:

```python
def execute_plan_with_feedback(plan):
    """Execute plan with perception-based feedback"""
    for step in plan:
        # Execute action
        action_result = execute_action(step)
        
        # Update world model based on perception
        if action_result.perception_data:
            update_world_model(action_result.perception_data)
        
        # Check if plan needs adjustment
        if check_plan_obsolescence(plan, current_world_model):
            # Regenerate plan based on new information
            new_plan = regenerate_plan_with_context(
                original_command,
                current_world_model
            )
            return execute_plan_with_feedback(new_plan)
    
    return "Plan completed successfully"
```

## Performance Optimization

### Caching Strategies

Improving response time through caching:

- **Plan Caching**: Storing common action sequences
- **Context Caching**: Caching relevant world knowledge
- **Response Caching**: Caching LLM responses for similar queries
- **Computational Caching**: Caching intermediate results

### Parallel Processing

Managing computational resources:

- **Background Processing**: Pre-computing plans for likely commands
- **Asynchronous Execution**: Executing independent actions in parallel
- **Load Balancing**: Distributing computation across available resources
- **Priority Management**: Prioritizing safety-critical computations

## Evaluation Metrics

### Plan Quality Metrics

- **Completeness**: Percentage of tasks successfully completed
- **Efficiency**: Time and resources required for task completion
- **Safety**: Number of safety violations during execution
- **Human Satisfaction**: User satisfaction with robot behavior

### LLM Performance Metrics

- **Accuracy**: Correctness of generated action sequences
- **Consistency**: Consistency in handling similar commands
- **Adaptability**: Ability to handle new situations
- **Latency**: Time from command to first action

## Hands-On Exercise

1. **LLM Integration**: Set up an LLM client for robotic planning:
   ```python
   # Configure your LLM (OpenAI API, Hugging Face, or local model)
   # Create a simple prompt for translating commands to actions
   # Test with simple commands like "Go forward" or "Turn left"
   ```

2. **Prompt Engineering**: Design effective prompts for robotic tasks:
   - Create a system prompt that defines robot capabilities
   - Add few-shot examples for common tasks
   - Include safety constraints in the prompt
   - Test the system with various commands

3. **Safety Validation**: Implement safety validation for LLM outputs:
   - Create rules to validate action sequences
   - Check for physical safety
   - Verify operational constraints
   - Test with potentially unsafe commands

4. **ROS Integration**: Connect LLM planner to ROS 2 system:
   - Create a service that accepts natural language commands
   - Publish generated action sequences to appropriate topics
   - Integrate with existing navigation and manipulation systems
   - Test the complete system with simple tasks

5. **Performance Testing**: Evaluate the system performance:
   - Measure response time for different command types
   - Test accuracy with similar but different commands
   - Analyze resource usage during operation
   - Document any safety issues encountered

## Exercises

1. **Prompt Design**: Design a prompt for a specific robotic task (e.g., setting a table). How would you structure it to ensure safe and effective behavior?

2. **Safety System**: How would you design a safety system that allows LLMs flexibility while preventing dangerous actions?

3. **Context Integration**: How would you integrate real-time perception data into the LLM planning process to handle dynamic environments?

4. **Error Recovery**: Design a strategy for LLM-based planning that handles execution failures gracefully.

## Summary

LLM-based cognitive planning enables humanoid robots to interpret and execute natural language commands through sophisticated reasoning capabilities. Proper integration with ROS 2 systems, safety validation, and performance optimization are essential for reliable operation.

## Self-Assessment

1. What are the key components of LLM-based robotic planning?
2. How do you design effective prompts for robotic applications?
3. What safety considerations are important for LLM-based planning?
4. How do you integrate LLM planning with ROS 2 action systems?