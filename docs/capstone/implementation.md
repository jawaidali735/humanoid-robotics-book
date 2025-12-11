---
id: 3
title: "Capstone Implementation: Building the Voice-Controlled System"
sidebar_position: 3
---

# Capstone Implementation: Building the Voice-Controlled System

## Introduction

This guide provides a comprehensive step-by-step implementation of the voice-controlled autonomous humanoid system. Following this guide will result in a fully functional system that integrates all components from the previous modules.

## Prerequisites

Before beginning the implementation, ensure you have:

1. Completed Modules 1 and 2 (ROS 2 fundamentals and simulation)
2. Installed all required dependencies from previous modules
3. Access to a humanoid robot platform or simulation environment
4. Proper safety equipment and testing area

## System Setup

### 1. Environment Configuration

First, create the necessary ROS 2 workspace structure for the capstone project:

```bash
# Create workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws/src

# Create the capstone package
ros2 pkg create --build-type ament_python capstone_humanoid_system --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs action_msgs
```

### 2. Package Structure

Create the following directory structure in your capstone package:

```
capstone_humanoid_system/
├── launch/
├── config/
├── scripts/
├── src/
│   ├── voice_processing/
│   ├── task_planning/
│   ├── navigation/
│   ├── motion_control/
│   └── sensor_fusion/
└── test/
```

### 3. Dependencies Installation

Install the required dependencies for voice processing and AI integration:

```bash
pip install openai-whisper
pip install transformers
pip install torch
pip install sentence-transformers
```

## Voice Command Interface Implementation

### 1. Voice Input Node

Create `src/voice_processing/voice_input_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading
import queue


class VoiceInputNode(Node):
    def __init__(self):
        super().__init__('voice_input_node')

        # Publisher for recognized text
        self.text_publisher = self.create_publisher(String, 'voice_command', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Configuration parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('timeout', 5.0)

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Start voice recognition thread
        self.recognition_thread = threading.Thread(target=self.recognize_speech)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()

        self.get_logger().info('Voice input node initialized')

    def recognize_speech(self):
        while rclpy.ok():
            try:
                with self.microphone as source:
                    # Listen for audio with timeout
                    audio = self.recognizer.listen(
                        source,
                        timeout=self.get_parameter('timeout').value
                    )

                # Recognize speech using Google's service
                text = self.recognizer.recognize_google(audio)

                # Publish recognized text
                msg = String()
                msg.data = text
                self.text_publisher.publish(msg)

                self.get_logger().info(f'Recognized: {text}')

            except sr.WaitTimeoutError:
                # Timeout is expected, continue listening
                pass
            except sr.UnknownValueError:
                self.get_logger().info('Could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'Error with speech recognition service: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error in speech recognition: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceInputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. NLP Processor Node

Create `src/voice_processing/nlp_processor_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from capstone_humanoid_system.msg import TaskSpecification
import json


class NLPProcessorNode(Node):
    def __init__(self):
        super().__init__('nlp_processor_node')

        # Subscriber for voice commands
        self.command_subscriber = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10
        )

        # Publisher for task specifications
        self.task_publisher = self.create_publisher(TaskSpecification, 'task_specification', 10)

        # Define command patterns and their corresponding tasks
        self.command_patterns = {
            'move forward': 'move_forward',
            'move backward': 'move_backward',
            'turn left': 'turn_left',
            'turn right': 'turn_right',
            'stop': 'stop',
            'go to': 'navigate_to',
            'pick up': 'pick_object',
            'put down': 'place_object',
            'wave': 'wave_hand',
            'sit down': 'sit',
            'stand up': 'stand',
        }

        self.get_logger().info('NLP processor node initialized')

    def command_callback(self, msg):
        command_text = msg.data.lower()
        self.get_logger().info(f'Processing command: {command_text}')

        # Parse the command and generate task specification
        task_spec = self.parse_command(command_text)

        if task_spec:
            # Publish the task specification
            self.task_publisher.publish(task_spec)
            self.get_logger().info(f'Task published: {task_spec.task_type}')
        else:
            self.get_logger().warn(f'Could not parse command: {command_text}')

    def parse_command(self, command_text):
        # Simple pattern matching for command parsing
        for pattern, task_type in self.command_patterns.items():
            if pattern in command_text:
                task_spec = TaskSpecification()
                task_spec.task_type = task_type
                task_spec.priority = 1  # Normal priority

                # Extract additional parameters based on command type
                if task_type == 'navigate_to':
                    # Extract destination from command
                    destination = command_text.replace('go to', '').strip()
                    task_spec.parameters = json.dumps({'destination': destination})
                elif task_type in ['pick_object', 'place_object']:
                    # Extract object name from command
                    object_name = command_text.replace('pick up', '').replace('put down', '').strip()
                    task_spec.parameters = json.dumps({'object': object_name})
                else:
                    task_spec.parameters = '{}'

                return task_spec

        # If no pattern matches, return None
        return None


def main(args=None):
    rclpy.init(args=args)
    node = NLPProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Task Specification Message

Create the custom message file `msg/TaskSpecification.msg`:

```
string task_type
int32 priority
string parameters
builtin_interfaces/Time timestamp
```

Update your `setup.py` to include the message:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'capstone_humanoid_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include message files
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Capstone humanoid system package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_input_node = capstone_humanoid_system.voice_input_node:main',
            'nlp_processor_node = capstone_humanoid_system.nlp_processor_node:main',
        ],
    },
)
```

## Task Planning System

### 1. Task Decomposer Node

Create `src/task_planning/task_decomposer_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from capstone_humanoid_system.msg import TaskSpecification
from capstone_humanoid_system.msg import TaskPlan
import json


class TaskDecomposerNode(Node):
    def __init__(self):
        super().__init__('task_decomposer_node')

        # Subscriber for task specifications
        self.task_subscriber = self.create_subscription(
            TaskSpecification,
            'task_specification',
            self.task_callback,
            10
        )

        # Publisher for task plans
        self.plan_publisher = self.create_publisher(TaskPlan, 'task_plan', 10)

        self.get_logger().info('Task decomposer node initialized')

    def task_callback(self, msg):
        self.get_logger().info(f'Decomposing task: {msg.task_type}')

        # Decompose the high-level task into subtasks
        plan = self.decompose_task(msg)

        if plan:
            # Publish the task plan
            self.plan_publisher.publish(plan)
            self.get_logger().info(f'Task plan published with {len(plan.subtasks)} subtasks')

    def decompose_task(self, task_spec):
        plan = TaskPlan()
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.header.frame_id = 'base_link'

        # Decompose based on task type
        if task_spec.task_type == 'navigate_to':
            params = json.loads(task_spec.parameters)
            destination = params.get('destination', '')

            # Create navigation plan
            plan.subtasks = [
                self.create_subtask('check_environment', '{}'),
                self.create_subtask('plan_path', json.dumps({'destination': destination})),
                self.create_subtask('execute_navigation', json.dumps({'destination': destination})),
                self.create_subtask('confirm_arrival', '{}')
            ]

        elif task_spec.task_type == 'pick_object':
            params = json.loads(task_spec.parameters)
            object_name = params.get('object', '')

            # Create pick object plan
            plan.subtasks = [
                self.create_subtask('locate_object', json.dumps({'object': object_name})),
                self.create_subtask('approach_object', json.dumps({'object': object_name})),
                self.create_subtask('grasp_object', json.dumps({'object': object_name})),
                self.create_subtask('verify_grasp', '{}')
            ]

        elif task_spec.task_type == 'move_forward':
            plan.subtasks = [
                self.create_subtask('check_front_clear', '{}'),
                self.create_subtask('move_forward', json.dumps({'distance': 1.0})),
                self.create_subtask('check_balance', '{}')
            ]

        elif task_spec.task_type == 'wave_hand':
            plan.subtasks = [
                self.create_subtask('move_to_waving_position', '{}'),
                self.create_subtask('wave_motion', '{}'),
                self.create_subtask('return_to_default', '{}')
            ]

        else:
            # Default plan for simple tasks
            plan.subtasks = [
                self.create_subtask(task_spec.task_type, task_spec.parameters)
            ]

        return plan

    def create_subtask(self, task_type, parameters):
        from capstone_humanoid_system.msg import SubTask
        subtask = SubTask()
        subtask.task_type = task_type
        subtask.parameters = parameters
        subtask.priority = 1
        return subtask


def main(args=None):
    rclpy.init(args=args)
    node = TaskDecomposerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. SubTask Message

Create `msg/SubTask.msg`:

```
string task_type
string parameters
int32 priority
bool is_completed
string status
```

And `msg/TaskPlan.msg`:

```
std_msgs/Header header
capstone_humanoid_system/SubTask[] subtasks
int32 current_subtask_index
string plan_status
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time
```

## Navigation System

### 1. Navigation Interface Node

Create `src/navigation/navigation_interface_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from capstone_humanoid_system.msg import SubTask
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class NavigationInterfaceNode(Node):
    def __init__(self):
        super().__init__('navigation_interface_node')

        # Subscriber for navigation subtasks
        self.task_subscriber = self.create_subscription(
            SubTask,
            'navigation_task',
            self.navigation_task_callback,
            10
        )

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Navigation interface node initialized')

    def navigation_task_callback(self, msg):
        if msg.task_type == 'navigate_to_pose':
            self.navigate_to_pose(msg)

    def navigate_to_pose(self, task):
        # Wait for the action server to be available
        self.nav_client.wait_for_server()

        # Create goal message
        goal_msg = NavigateToPose.Goal()

        # Parse destination from task parameters
        import json
        params = json.loads(task.parameters)
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        z = params.get('z', 0.0)

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z

        # Simple orientation (facing forward)
        goal_msg.pose.pose.orientation.w = 1.0

        # Send the goal
        self.get_logger().info(f'Navigating to pose: ({x}, {y}, {z})')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Motion Control System

### 1. Motion Control Interface Node

Create `src/motion_control/motion_control_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from capstone_humanoid_system.msg import SubTask
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class MotionControlNode(Node):
    def __init__(self):
        super().__init__('motion_control_node')

        # Subscriber for motion subtasks
        self.task_subscriber = self.create_subscription(
            SubTask,
            'motion_task',
            self.motion_task_callback,
            10
        )

        # Publisher for joint trajectories
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Publisher for joint states (for simulation)
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Get joint names from parameters
        self.declare_parameter('joint_names', [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ])

        self.joint_names = self.get_parameter('joint_names').value
        self.current_joint_positions = [0.0] * len(self.joint_names)

        self.get_logger().info('Motion control node initialized')

    def motion_task_callback(self, msg):
        self.get_logger().info(f'Executing motion task: {msg.task_type}')

        if msg.task_type == 'move_forward':
            self.execute_move_forward()
        elif msg.task_type == 'wave_hand':
            self.execute_wave_hand()
        elif msg.task_type == 'move_to_waving_position':
            self.move_to_waving_position()
        elif msg.task_type == 'wave_motion':
            self.execute_wave_motion()
        elif msg.task_type == 'return_to_default':
            self.return_to_default_position()
        elif msg.task_type == 'move_joint':
            self.move_joint(msg)

    def execute_move_forward(self):
        # Simple forward walking motion
        # This is a simplified example - real implementation would be more complex
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # Create trajectory points for forward movement
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Default position
        point1.time_from_start.sec = 1
        trajectory.points.append(point1)

        # Publish the trajectory
        self.trajectory_publisher.publish(trajectory)

    def execute_wave_hand(self):
        # Wave hand motion
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # Move right arm to waving position
        point1 = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)
        # Set right shoulder and elbow joints for waving
        right_shoulder_idx = self.joint_names.index('right_shoulder_joint')
        right_elbow_idx = self.joint_names.index('right_elbow_joint')
        positions[right_shoulder_idx] = 0.5  # Raise arm
        positions[right_elbow_idx] = 0.5   # Bend elbow
        point1.positions = positions
        point1.time_from_start.sec = 1
        trajectory.points.append(point1)

        # Wave motion
        point2 = JointTrajectoryPoint()
        positions2 = positions.copy()
        positions2[right_shoulder_idx] = 0.7  # Continue raising
        point2.positions = positions2
        point2.time_from_start.sec = 1
        point2.time_from_start.nanosec = 500000000  # 1.5 seconds
        trajectory.points.append(point2)

        # Return to position
        point3 = JointTrajectoryPoint()
        positions3 = positions.copy()
        positions3[right_shoulder_idx] = 0.3  # Lower slightly
        point3.positions = positions3
        point3.time_from_start.sec = 2
        trajectory.points.append(point3)

        self.trajectory_publisher.publish(trajectory)

    def move_to_waving_position(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point1 = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)
        # Set right arm joints for waving position
        right_shoulder_idx = self.joint_names.index('right_shoulder_joint')
        right_elbow_idx = self.joint_names.index('right_elbow_joint')
        positions[right_shoulder_idx] = 0.5
        positions[right_elbow_idx] = 0.3
        point1.positions = positions
        point1.time_from_start.sec = 1
        trajectory.points.append(point1)

        self.trajectory_publisher.publish(trajectory)

    def execute_wave_motion(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # Create waving motion with multiple points
        for i in range(5):  # 5 wave motions
            point = JointTrajectoryPoint()
            positions = [0.0] * len(self.joint_names)
            right_shoulder_idx = self.joint_names.index('right_shoulder_joint')

            # Alternate shoulder position for waving
            wave_pos = 0.5 + 0.2 * math.sin(i * math.pi / 2)
            positions[right_shoulder_idx] = wave_pos

            point.positions = positions
            point.time_from_start.sec = i + 1
            trajectory.points.append(point)

        self.trajectory_publisher.publish(trajectory)

    def return_to_default_position(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * len(self.joint_names)  # Default position
        point1.time_from_start.sec = 1
        trajectory.points.append(point1)

        self.trajectory_publisher.publish(trajectory)

    def move_joint(self, task):
        import json
        params = json.loads(task.parameters)
        joint_name = params.get('joint_name', '')
        target_position = params.get('position', 0.0)
        duration = params.get('duration', 1.0)

        if joint_name in self.joint_names:
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names

            point1 = JointTrajectoryPoint()
            positions = [0.0] * len(self.joint_names)
            joint_idx = self.joint_names.index(joint_name)
            positions[joint_idx] = target_position
            point1.positions = positions
            point1.time_from_start.sec = int(duration)
            point1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
            trajectory.points.append(point1)

            self.trajectory_publisher.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = MotionControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Main Launch File

Create a launch file to start the entire system:

Create `launch/capstone_system.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Voice input node
        Node(
            package='capstone_humanoid_system',
            executable='voice_input_node',
            name='voice_input_node',
            output='screen',
            parameters=[
                {'sample_rate': 16000},
                {'chunk_size': 1024},
                {'timeout': 5.0}
            ]
        ),

        # NLP processor node
        Node(
            package='capstone_humanoid_system',
            executable='nlp_processor_node',
            name='nlp_processor_node',
            output='screen'
        ),

        # Task decomposer node
        Node(
            package='capstone_humanoid_system',
            executable='task_decomposer_node',
            name='task_decomposer_node',
            output='screen'
        ),

        # Navigation interface node
        Node(
            package='capstone_humanoid_system',
            executable='navigation_interface_node',
            name='navigation_interface_node',
            output='screen'
        ),

        # Motion control node
        Node(
            package='capstone_humanoid_system',
            executable='motion_control_node',
            name='motion_control_node',
            output='screen',
            parameters=[
                {
                    'joint_names': [
                        'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
                        'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
                        'left_shoulder_joint', 'left_elbow_joint',
                        'right_shoulder_joint', 'right_elbow_joint'
                    ]
                }
            ]
        )
    ])
```

## System Integration Testing

### 1. Simple Test Script

Create `test/test_capstone_system.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from capstone_humanoid_system.msg import TaskSpecification
import time


class CapstoneTestNode(Node):
    def __init__(self):
        super().__init__('capstone_test_node')

        # Publisher for voice commands
        self.voice_publisher = self.create_publisher(String, 'voice_command', 10)

        # Publisher for task specifications
        self.task_publisher = self.create_publisher(TaskSpecification, 'task_specification', 10)

        self.get_logger().info('Capstone test node initialized')

    def test_voice_command(self, command):
        msg = String()
        msg.data = command
        self.voice_publisher.publish(msg)
        self.get_logger().info(f'Published voice command: {command}')

    def test_task_specification(self, task_type, parameters='{}'):
        task_msg = TaskSpecification()
        task_msg.task_type = task_type
        task_msg.priority = 1
        task_msg.parameters = parameters
        self.task_publisher.publish(task_msg)
        self.get_logger().info(f'Published task specification: {task_type}')


def main(args=None):
    rclpy.init(args=args)
    test_node = CapstoneTestNode()

    # Give nodes time to start up
    time.sleep(2.0)

    # Test various commands
    test_node.test_voice_command('wave')
    time.sleep(3.0)

    test_node.test_voice_command('move forward')
    time.sleep(3.0)

    test_node.test_task_specification('wave_hand')
    time.sleep(3.0)

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the System

### 1. Build the Package

```bash
cd ~/humanoid_ws
colcon build --packages-select capstone_humanoid_system
source install/setup.bash
```

### 2. Launch the System

```bash
ros2 launch capstone_humanoid_system capstone_system.launch.py
```

### 3. Test the System

In another terminal:

```bash
# Test with voice commands (if microphone is available)
# Or use command line to publish messages directly:

# Test wave command
ros2 topic pub /voice_command std_msgs/String "data: 'wave'"

# Test move forward command
ros2 topic pub /voice_command std_msgs/String "data: 'move forward'"
```

## Safety Considerations

### 1. Emergency Stop Implementation

Add an emergency stop node:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from builtin_interfaces.msg import Time


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')

        # Publisher for emergency stop commands
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)

        # Subscriber for emergency stop requests
        self.emergency_stop_subscriber = self.create_subscription(
            Bool,
            'emergency_stop_request',
            self.emergency_stop_callback,
            10
        )

        self.is_active = True
        self.get_logger().info('Emergency stop node initialized')

    def emergency_stop_callback(self, msg):
        if msg.data:
            self.trigger_emergency_stop()
        else:
            self.release_emergency_stop()

    def trigger_emergency_stop(self):
        if self.is_active:
            self.is_active = False
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_publisher.publish(stop_msg)
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')

    def release_emergency_stop(self):
        if not self.is_active:
            self.is_active = True
            stop_msg = Bool()
            stop_msg.data = False
            self.emergency_stop_publisher.publish(stop_msg)
            self.get_logger().info('Emergency stop released')


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Simulation Integration

### 1. Gazebo Integration

To integrate with Gazebo simulation, create a launch file that starts both the capstone system and the simulation:

Create `launch/capstone_with_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to Gazebo launch file (adjust as needed for your robot model)
    gazebo_launch_file = os.path.join(
        get_package_share_directory('your_robot_gazebo'),
        'launch',
        'robot_world.launch.py'
    )

    return LaunchDescription([
        # Launch Gazebo with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        ),

        # Launch capstone system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('capstone_humanoid_system'),
                    'launch',
                    'capstone_system.launch.py'
                )
            )
        )
    ])
```

## Conclusion

This implementation guide provides a comprehensive foundation for the voice-controlled autonomous humanoid system. The system integrates all four modules through a modular architecture that emphasizes safety, modularity, and extensibility.

Key features implemented:

1. Voice command processing with natural language understanding
2. Task decomposition and planning system
3. Navigation and motion control integration
4. Safety systems and emergency stop mechanisms
5. Simulation integration capabilities

The system can be extended with additional features such as computer vision for object recognition, more sophisticated AI planning, or integration with specific humanoid robot platforms.