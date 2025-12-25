---
id: module-1-ros2-implementation
title: "Implementation: Practical ROS 2 Examples"
sidebar_position: 4
---

# Implementation Walkthroughs: ROS 2 Practical Examples

## Introduction

This section provides hands-on implementation examples for ROS 2 concepts. You'll build complete examples that demonstrate practical applications of ROS 2 in humanoid robotics contexts.

## Learning Outcomes

After completing this section, you will be able to:
- Implement complete ROS 2 nodes for humanoid robot control
- Create message types specific to humanoid robotics
- Integrate multiple ROS 2 concepts in a single application
- Debug and test ROS 2 implementations effectively

## Conceptual Foundations

### Humanoid Robot Control Architecture

Humanoid robots require coordination of multiple systems:
- Joint control and actuation
- Sensor data processing
- Motion planning and execution
- Balance and stability control
- Perception and decision making

ROS 2 provides the communication infrastructure to coordinate these systems effectively.

### Common Humanoid Robot Message Types

Humanoid robots often use specialized message types:
- Joint states and commands
- IMU and force/torque sensor data
- Robot state and configuration
- Motion planning requests and results

## Technical Deep Dive

### Creating Custom Message Types

For humanoid robots, you may need custom message types:

1. **Create a msg directory in your package**:
```
my_robot_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    └── HumanoidJointState.msg
```

2. **Define the message** (`msg/HumanoidJointState.msg`):
```
# Joint names for humanoid robot
string[] joint_names

# Joint positions in radians
float64[] positions

# Joint velocities in rad/s
float64[] velocities

# Joint efforts in Nm
float64[] efforts

# Timestamp
builtin_interfaces/Time timestamp

# Robot state flags
bool is_balancing
bool is_moving
bool emergency_stop
```

3. **Update package.xml**:
```xml
<depend>builtin_interfaces</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

4. **Update CMakeLists.txt**:
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HumanoidJointState.msg"
)
```

### Complete Humanoid Robot Node Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool
import numpy as np


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(
            JointState,  # Simplified - would use custom IMU msg in practice
            '/imu/data',
            self.imu_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        # Robot state
        self.joint_positions = np.zeros(28)  # 28 DOF humanoid
        self.balance_state = True
        self.target_velocity = [0.0, 0.0, 0.0]  # x, y, theta

        self.get_logger().info('Humanoid Controller initialized')

    def control_loop(self):
        """Main control loop running at 100Hz"""
        # Update joint positions based on control algorithm
        self.update_joint_positions()

        # Publish joint states
        self.publish_joint_states()

        # Check balance and stability
        self.check_balance()

    def update_joint_positions(self):
        """Update joint positions based on target velocity and balance"""
        # Simplified control algorithm
        dt = 0.01  # 100Hz
        for i in range(len(self.joint_positions)):
            # Apply simple control law
            self.joint_positions[i] += self.target_velocity[0] * dt * 0.1

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.name = self.get_joint_names()
        msg.position = self.joint_positions.tolist()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_pub.publish(msg)

    def get_joint_names(self):
        """Return list of humanoid joint names"""
        return [
            'left_hip_roll', 'left_hip_yaw', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_roll', 'right_hip_yaw', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            # ... add other joints
        ]

    def check_balance(self):
        """Check robot balance and adjust if needed"""
        # Simplified balance check
        if not self.balance_state:
            self.get_logger().warning('Balance compromised! Adjusting...')

    def imu_callback(self, msg):
        """Handle IMU data for balance control"""
        # Process IMU data for balance algorithms
        pass


def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down humanoid controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Working with Robot State Publisher

For humanoid robots, you'll often use the robot_state_publisher to publish TF transforms:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf_transformations as tf_trans
from std_msgs.msg import Header


class HumanoidStatePublisher(Node):
    def __init__(self):
        super().__init__('humanoid_state_publisher')

        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)

        # Robot description (simplified)
        self.links = ['base_link', 'torso', 'head', 'left_hip', 'right_hip',
                     'left_knee', 'right_knee', 'left_ankle', 'right_ankle']

        self.get_logger().info('Humanoid State Publisher initialized')

    def joint_state_callback(self, msg):
        """Process joint states and publish transforms"""
        tf_msg = TFMessage()

        # Create transforms for each joint
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                transform = self.create_transform(joint_name, msg.position[i])
                if transform:
                    tf_msg.transforms.append(transform)

        if tf_msg.transforms:
            self.tf_pub.publish(tf_msg)

    def create_transform(self, joint_name, joint_position):
        """Create transform for a specific joint"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = joint_name

        # Set transform based on joint position
        # This is simplified - real implementation would use URDF
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Convert joint position to rotation (simplified)
        q = tf_trans.quaternion_from_euler(0, 0, joint_position)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        return t


def main(args=None):
    rclpy.init(args=args)
    publisher = HumanoidStatePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Practical Implementation

### Complete Example: Humanoid Walking Controller

Let's implement a complete example that demonstrates multiple ROS 2 concepts working together:

1. **Create a new package**:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_walker
```

2. **Create the walking controller node** (`humanoid_walker/humanoid_walker/walking_controller.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import math


class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.status_pub = self.create_publisher(Float64MultiArray, 'walking_status', 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for walking pattern generation
        self.walk_timer = self.create_timer(0.02, self.walk_callback)  # 50Hz

        # Walking parameters
        self.step_frequency = 1.0  # Hz
        self.step_height = 0.05    # meters
        self.step_length = 0.2     # meters
        self.current_phase = 0.0

        # Robot state
        self.target_velocity = [0.0, 0.0, 0.0]  # x, y, theta
        self.joint_positions = np.zeros(28)  # 28 DOF humanoid

        self.get_logger().info('Walking Controller initialized')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.target_velocity[0] = msg.linear.x
        self.target_velocity[1] = msg.linear.y
        self.target_velocity[2] = msg.angular.z

    def walk_callback(self):
        """Generate walking pattern based on target velocity"""
        # Update walking phase
        dt = 0.02  # 50Hz
        self.current_phase += 2 * math.pi * self.step_frequency * dt

        if self.current_phase > 2 * math.pi:
            self.current_phase = 0.0

        # Generate walking pattern if moving
        if abs(self.target_velocity[0]) > 0.01 or abs(self.target_velocity[1]) > 0.01:
            self.generate_walking_pattern()

        # Publish joint commands
        self.publish_joint_commands()

    def generate_walking_pattern(self):
        """Generate walking pattern for humanoid joints"""
        # Simplified walking pattern generation
        # In a real implementation, this would use inverse kinematics

        # Left leg pattern
        left_foot_x = self.step_length * math.sin(self.current_phase)
        left_foot_z = self.step_height * abs(math.sin(self.current_phase))

        # Right leg pattern
        right_foot_x = self.step_length * math.sin(self.current_phase + math.pi)
        right_foot_z = self.step_height * abs(math.sin(self.current_phase + math.pi))

        # Map to joint angles (simplified)
        self.joint_positions[0] = left_foot_x * 0.1   # left hip
        self.joint_positions[1] = left_foot_z * 0.2   # left knee
        self.joint_positions[2] = right_foot_x * 0.1  # right hip
        self.joint_positions[3] = right_foot_z * 0.2  # right knee

    def publish_joint_commands(self):
        """Publish current joint positions"""
        msg = JointState()
        msg.name = self.get_joint_names()
        msg.position = self.joint_positions.tolist()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_pub.publish(msg)

        # Publish walking status
        status_msg = Float64MultiArray()
        status_msg.data = [self.current_phase, self.target_velocity[0],
                          self.target_velocity[1], self.target_velocity[2]]
        self.status_pub.publish(status_msg)

    def get_joint_names(self):
        """Return list of humanoid joint names"""
        return [
            'left_hip_roll', 'left_hip_yaw', 'left_hip_pitch',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_roll', 'right_hip_yaw', 'right_hip_pitch',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            'torso_yaw', 'torso_pitch', 'torso_roll',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow', 'left_wrist_yaw', 'left_wrist_pitch',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow', 'right_wrist_yaw', 'right_wrist_pitch',
            'head_yaw'
        ]


def main(args=None):
    rclpy.init(args=args)
    controller = WalkingController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down walking controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

3. **Update setup.py** to include the executable:
```python
entry_points={
    'console_scripts': [
        'walking_controller = humanoid_walker.walking_controller:main',
    ],
},
```

4. **Build and run the package**:
```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_walker
source install/setup.bash
ros2 run humanoid_walker walking_controller
```

### Testing the Implementation

1. **Run the controller**:
```bash
ros2 run humanoid_walker walking_controller
```

2. **Send velocity commands**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
```

3. **Monitor the output**:
```bash
ros2 topic echo /joint_commands
```

## Common Pitfalls & Debugging Tips

### Common Issues

1. **Timing Issues**:
   - Symptoms: Unstable walking patterns, missed control cycles
   - Solution: Ensure consistent control loop timing and appropriate update rates

2. **Joint Limit Violations**:
   - Symptoms: Robot joints reaching physical limits
   - Solution: Implement joint limit checking in control algorithms

3. **Balance Problems**:
   - Symptoms: Robot falling over during walking
   - Solution: Implement proper balance control and center of mass management

4. **Communication Delays**:
   - Symptoms: Lag between command and execution
   - Solution: Optimize network QoS settings and check for bottlenecks

### Debugging Strategies

1. **Use visualization tools**:
   - RViz for visualizing robot state and TF transforms
   - PlotJuggler for plotting time-series data
   - rqt_graph for visualizing node connections

2. **Log critical variables**:
   - Joint positions and velocities
   - Balance state and center of mass
   - Control loop timing

3. **Implement safety checks**:
   - Emergency stop functionality
   - Joint limit enforcement
   - Balance stability monitoring

4. **Test incrementally**:
   - Start with single joint control
   - Progress to coordinated movement
   - Add complexity gradually

## Industry Use Cases

### Research Humanoid Robots
ROS 2 is used extensively in humanoid robotics research for implementing walking algorithms, balance control, and human-robot interaction.

### Industrial Applications
Humanoid robots are being developed for various industrial applications including inspection, maintenance, and collaborative tasks.

### Educational Platforms
Many educational humanoid robots use ROS 2 to teach robotics concepts and provide a platform for research.

## Summary / Key Takeaways

- Implementing humanoid robot control requires integration of multiple ROS 2 concepts
- Custom message types are often needed for humanoid-specific data
- Control loops must run at appropriate frequencies for stable operation
- Safety and balance are critical considerations for humanoid robots
- Visualization and debugging tools are essential for development

## Practice Tasks / Mini-Projects

### Exercise 1: Joint State Publisher
Create a node that publishes realistic joint states for a humanoid robot model, including position, velocity, and effort for all joints.

### Exercise 2: Simple Walking Pattern
Implement a basic walking pattern generator that creates coordinated joint movements for forward walking.

### Exercise 3: Balance Controller
Create a simple balance controller that adjusts joint positions to maintain the robot's center of mass within its support polygon.