---
id: module-1-ros2-exercises
title: "Exercises: Hands-on ROS 2 Practice"
sidebar_position: 6
---

# Exercises: Hands-on Practice for ROS 2 Fundamentals

## Introduction

This section provides hands-on exercises to reinforce your understanding of ROS 2 fundamentals in the context of humanoid robotics. Each exercise builds upon the concepts covered in previous sections and provides practical experience with ROS 2 development.

## Learning Outcomes

After completing these exercises, you will be able to:
- Create and run ROS 2 nodes for humanoid robot applications
- Implement communication patterns (topics, services, actions) in Python
- Debug and troubleshoot ROS 2 nodes
- Apply ROS 2 concepts to humanoid-specific scenarios

## Conceptual Foundations

### Exercise Approach

The exercises in this section follow a progressive difficulty model:
- **Beginner**: Focus on understanding basic ROS 2 concepts
- **Intermediate**: Combine multiple concepts and implement more complex behaviors
- **Advanced**: Address real-world challenges in humanoid robotics

### Tools and Environment

Before starting these exercises, ensure you have:
- A working ROS 2 Humble installation
- Python 3.8+ environment
- Basic knowledge of Python programming
- Access to the ROS 2 command line tools

## Technical Deep Dive

### Exercise 1: Basic Publisher-Subscriber Pattern

**Objective**: Create a simple publisher-subscriber pair that simulates sensor data from a humanoid robot.

**Implementation Steps**:
1. Create a new package: `ros2 pkg create --build-type ament_python robot_sensors`
2. Create a publisher node that simulates joint position data
3. Create a subscriber node that logs the received data
4. Test the communication between nodes

**Sample Publisher Code**:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = [
            random.uniform(-1.0, 1.0),
            random.uniform(-0.5, 1.5),
            random.uniform(-0.5, 0.5)
        ]
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

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

**Expected Outcome**: The publisher should send joint position data at 10Hz, and the subscriber should log this data to the console.

### Exercise 2: Service Implementation

**Objective**: Create a service that calculates humanoid robot inverse kinematics.

**Implementation Steps**:
1. Define a custom service message for inverse kinematics
2. Implement a service server that calculates joint angles
3. Create a service client that requests calculations
4. Test the service with various target positions

**Sample Service Definition** (`srv/InverseKinematics.srv`):
```
# Target position in Cartesian space
geometry_msgs/Point target_position
# Target orientation
geometry_msgs/Quaternion target_orientation
---
# Calculated joint angles
float64[] joint_angles
# Success flag
bool success
```

### Exercise 3: Action Server for Long-Running Tasks

**Objective**: Implement an action server that controls a humanoid robot's walking motion.

**Implementation Steps**:
1. Define a custom action message for walking goals
2. Implement an action server that executes walking patterns
3. Create an action client that sends walking commands
4. Test the action with different walking distances

**Sample Action Definition** (`action/Walk.action`):
```
# Walking goal: distance and direction
float64 distance_x
float64 distance_y
float64 rotation
---
# Result: success and actual distance traveled
bool success
float64 actual_distance_x
float64 actual_distance_y
---
# Feedback: current progress
float64 progress_x
float64 progress_y
string status
```

## Practical Implementation

### Exercise 1: Simple Publisher-Subscriber Pair

Let's implement the first exercise step by step:

1. **Create the package**:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_exercises
```

2. **Create the publisher node** (`robot_exercises/robot_exercises/sensor_publisher.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random


class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(JointState, 'robot_sensors', 10)
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = ['left_hip', 'left_knee', 'left_ankle', 'right_hip', 'right_knee', 'right_ankle']

        # Generate realistic joint positions
        positions = []
        for _ in range(len(msg.name)):
            positions.append(random.uniform(-1.5, 1.5))  # Reasonable joint limits

        msg.position = positions
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.publisher.publish(msg)
        self.get_logger().info(f'Published sensor data: {positions}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

3. **Create the subscriber node** (`robot_exercises/robot_exercises/sensor_subscriber.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'robot_sensors',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received sensor data:')
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.get_logger().info(f'  {name}: {msg.position[i]:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor subscriber')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

4. **Update setup.py** to include executables:
```python
from setuptools import setup

package_name = 'robot_exercises'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Exercises for ROS 2 humanoid robotics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = robot_exercises.sensor_publisher:main',
            'sensor_subscriber = robot_exercises.sensor_subscriber:main',
        ],
    },
)
```

5. **Build and run the exercise**:
```bash
cd ~/ros2_ws
colcon build --packages-select robot_exercises
source install/setup.bash

# Terminal 1: Run the publisher
ros2 run robot_exercises sensor_publisher

# Terminal 2: Run the subscriber
ros2 run robot_exercises sensor_subscriber
```

### Exercise 2: Service Implementation

Let's create a service for calculating center of mass:

1. **Create the service definition** (`robot_exercises/robot_exercises/srv/CenterOfMass.srv`):
```
# Joint positions and masses for CoM calculation
string[] joint_names
float64[] joint_positions
float64[] joint_masses
float64[] joint_positions_xyz  # Flattened array: x1,y1,z1,x2,y2,z2...
---
# Center of mass result
float64 center_of_mass_x
float64 center_of_mass_y
float64 center_of_mass_z
bool success
```

2. **Create the service server** (`robot_exercises/robot_exercises/com_server.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_exercises.srv import CenterOfMass  # Update import based on your package structure


class CenterOfMassServer(Node):

    def __init__(self):
        super().__init__('center_of_mass_server')
        self.srv = self.create_service(
            CenterOfMass,
            'calculate_center_of_mass',
            self.calculate_center_of_mass_callback)

    def calculate_center_of_mass_callback(self, request, response):
        try:
            # Validate input
            if (len(request.joint_names) != len(request.joint_positions) or
                len(request.joint_positions) != len(request.joint_masses) or
                len(request.joint_masses) * 3 != len(request.joint_positions_xyz)):
                response.success = False
                return response

            # Calculate center of mass
            total_mass = sum(request.joint_masses)
            if total_mass <= 0:
                response.success = False
                return response

            com_x = 0.0
            com_y = 0.0
            com_z = 0.0

            for i in range(len(request.joint_masses)):
                mass = request.joint_masses[i]
                idx = i * 3
                if idx + 2 < len(request.joint_positions_xyz):
                    com_x += request.joint_positions_xyz[idx] * mass
                    com_y += request.joint_positions_xyz[idx + 1] * mass
                    com_z += request.joint_positions_xyz[idx + 2] * mass

            response.center_of_mass_x = com_x / total_mass
            response.center_of_mass_y = com_y / total_mass
            response.center_of_mass_z = com_z / total_mass
            response.success = True

            self.get_logger().info(
                f'Calculated CoM: ({response.center_of_mass_x:.3f}, '
                f'{response.center_of_mass_y:.3f}, {response.center_of_mass_z:.3f})'
            )

        except Exception as e:
            self.get_logger().error(f'Error calculating center of mass: {e}')
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CenterOfMassServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down center of mass server')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

3. **Create the service client** (`robot_exercises/robot_exercises/com_client.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# Import the service - adjust based on your actual service definition
from robot_exercises.srv import CenterOfMass


class CenterOfMassClient(Node):

    def __init__(self):
        super().__init__('center_of_mass_client')
        self.cli = self.create_client(CenterOfMass, 'calculate_center_of_mass')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, joint_names, joint_positions, joint_masses, joint_positions_xyz):
        request = CenterOfMass.Request()
        request.joint_names = joint_names
        request.joint_positions = joint_positions
        request.joint_masses = joint_masses
        request.joint_positions_xyz = joint_positions_xyz

        self.future = self.cli.call_async(request)
        return self.future


def main(args=None):
    rclpy.init(args=args)
    client = CenterOfMassClient()

    # Example data for a simple 2-joint system
    joint_names = ['hip', 'knee']
    joint_positions = [0.0, 0.0]
    joint_masses = [5.0, 3.0]  # kg
    joint_positions_xyz = [0.0, 0.5, 0.0, 0.0, 0.2, 0.0]  # x, y, z for each joint

    future = client.send_request(joint_names, joint_positions, joint_masses, joint_positions_xyz)

    try:
        rclpy.spin_until_future_complete(client, future)
        response = future.result()

        if response.success:
            client.get_logger().info(
                f'Center of Mass: ({response.center_of_mass_x:.3f}, '
                f'{response.center_of_mass_y:.3f}, {response.center_of_mass_z:.3f})'
            )
        else:
            client.get_logger().error('Failed to calculate center of mass')

    except KeyboardInterrupt:
        client.get_logger().info('Shutting down center of mass client')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Exercise 3: Action Implementation

For the action exercise, let's create a simple action for moving a humanoid arm:

1. **Create the action definition** (`robot_exercises/robot_exercises/action/MoveArm.action`):
```
# Goal: target joint positions
float64[] target_positions
string[] joint_names
---
# Result: success and actual positions
bool success
float64[] actual_positions
string message
---
# Feedback: current progress
float64[] current_positions
float64 progress_percentage
string status
```

## Common Pitfalls & Debugging Tips

### Exercise 1: Publisher-Subscriber Issues

**Common Problems**:
1. **Nodes not communicating**: Check that both nodes are using the same topic name
2. **Timing issues**: Publishers may start before subscribers, causing initial messages to be missed
3. **Message type mismatches**: Ensure both publisher and subscriber use the same message type

**Solutions**:
- Use `ros2 topic list` and `ros2 topic echo` to verify communication
- Implement latching for important configuration messages
- Add message validation in subscribers

### Exercise 2: Service Implementation Issues

**Common Problems**:
1. **Service not found**: Client starts before server
2. **Long response times**: Complex calculations blocking the service thread
3. **Data validation**: Incorrect input data causing crashes

**Solutions**:
- Use `wait_for_service()` before sending requests
- Implement non-blocking service callbacks for long operations
- Add comprehensive input validation

### Exercise 3: Action Implementation Issues

**Common Problems**:
1. **Action client not connecting**: Client starts before server
2. **Feedback not received**: Incorrect action definition or implementation
3. **Goal cancellation**: Not handling cancellation properly

**Solutions**:
- Wait for action server before sending goals
- Implement proper feedback publishing
- Handle goal cancellation gracefully

### Debugging Strategies

1. **Use ROS 2 command line tools**:
   - `ros2 node list` - Check if nodes are running
   - `ros2 topic list` - Check available topics
   - `ros2 service list` - Check available services
   - `ros2 action list` - Check available actions

2. **Add comprehensive logging**:
   - Log important state changes
   - Log message contents at different levels
   - Use different log levels appropriately

3. **Test incrementally**:
   - Start with simple working examples
   - Add complexity gradually
   - Test each component individually

## Industry Use Cases

### Simulation-Based Development
Many humanoid robotics companies use exercises and simulations similar to these to:
- Train developers on ROS 2 concepts
- Test control algorithms in safe environments
- Validate communication patterns before hardware deployment

### Educational Applications
These types of exercises are commonly used in:
- University robotics courses
- Professional development programs
- Certification and assessment

## Summary / Key Takeaways

- Exercises provide hands-on experience with core ROS 2 concepts
- Start with simple examples and gradually increase complexity
- Use ROS 2 tools extensively for debugging and validation
- Test components individually before integration
- Implement proper error handling and logging

## Practice Tasks / Mini-Projects

### Exercise 1: Basic Communication (Beginner)
Create a publisher that sends temperature readings from a humanoid robot's joints and a subscriber that logs these readings with timestamps. Verify that the communication works correctly.

### Exercise 2: Service Implementation (Intermediate)
Implement a service that takes a desired humanoid pose (joint angles) and returns whether that pose is reachable based on joint limits. Include proper validation and error handling.

### Exercise 3: Action Server (Advanced)
Create an action server that moves a humanoid robot's arm through a sequence of waypoints. The action should provide feedback on progress and handle cancellation requests properly.

### Exercise 4: Multi-Node System (Advanced)
Design and implement a system with multiple nodes that work together: a sensor node that publishes joint states, a processing node that calculates center of mass, and a visualization node that logs the results. Ensure proper communication between all nodes.

### Exercise 5: Safety System (Advanced)
Implement a safety monitoring system that subscribes to joint states and publishes emergency stop commands if joint limits are exceeded. Include proper safety protocols and testing procedures.