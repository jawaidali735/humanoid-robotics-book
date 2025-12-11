---
id: module-1-ros2-concepts
title: "Core Concepts: ROS 2 Fundamentals"
sidebar_position: 2
---

# Core Concepts: ROS 2 Fundamentals

## Introduction

This section introduces the fundamental concepts of Robot Operating System 2 (ROS 2), which provides the middleware infrastructure for robotic applications. Understanding these concepts is crucial for building robust and scalable robotic systems.

## Learning Outcomes

After completing this section, you will be able to:
- Explain the core components of ROS 2 architecture
- Understand the role of middleware in robotic systems
- Identify and describe nodes, topics, services, and actions
- Recognize the benefits of distributed computing in robotics

## Conceptual Foundations

### What is ROS 2?

ROS 2 (Robot Operating System 2) is not an actual operating system but rather a collection of software frameworks and tools that help you build robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

### Key Architecture Components

#### Nodes
A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node typically performs a specific task and communicates with other nodes through topics, services, or actions.

#### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are the data packets sent between nodes. The communication is based on a publish-subscribe pattern where publishers send messages to a topic and subscribers receive messages from a topic.

#### Services
Services provide a request-response communication pattern. A client sends a request to a service server, which processes the request and returns a response.

#### Actions
Actions are used for long-running tasks that may take some time to complete. They provide feedback during execution and can be canceled. Actions use a goal-result-feedback pattern.

### Middleware: DDS
ROS 2 uses Data Distribution Service (DDS) as its underlying middleware. DDS provides a standardized interface for real-time, distributed data exchange.

## Technical Deep Dive

### ROS 2 Architecture

ROS 2 follows a distributed architecture where multiple nodes can run on different machines. The architecture includes:

1. **Node**: The basic execution unit
2. **ROS Client Libraries**: rclcpp (C++), rclpy (Python), etc.
3. **DDS Implementation**: The middleware layer (e.g., Fast DDS, Cyclone DDS)
4. **ROS Middleware Interface (RMW)**: Abstraction layer between ROS and DDS

### Communication Patterns

#### Publisher-Subscriber (Topics)
```python
import rclpy
from std_msgs.msg import String

class MinimalPublisher:
    def __init__(self):
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
```

#### Client-Server (Services)
```python
from example_interfaces.srv import AddTwoInts

class MinimalService:
    def __init__(self):
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

#### Action Server-Client
Actions are ideal for long-running tasks that provide feedback during execution.

### Quality of Service (QoS) Settings

QoS settings allow you to configure how messages are delivered in terms of reliability, durability, and liveliness.

## Practical Implementation

### Setting up Your First ROS 2 Workspace

1. Create a workspace directory:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. Source ROS 2:
```bash
source /opt/ros/humble/setup.bash
```

3. Build the workspace:
```bash
colcon build
source install/setup.bash
```

### Creating a Simple Publisher Node

1. Create a new package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_tutorials
```

2. Edit the package's main Python file to create a publisher node

3. Build and run the package

### Creating a Simple Subscriber Node

1. Create another node in the same package that subscribes to the topic

2. Build and run both nodes simultaneously

## Common Pitfalls & Debugging Tips

### Common Issues

1. **Node Communication Issues**:
   - Symptoms: Nodes not communicating with each other
   - Solution: Check if nodes are on the same ROS domain ID and namespace

2. **DDS Discovery Problems**:
   - Symptoms: Nodes on different machines can't see each other
   - Solution: Verify network configuration and firewall settings

3. **Message Type Mismatches**:
   - Symptoms: Nodes crash or don't receive expected data
   - Solution: Verify that publisher and subscriber use the same message type

### Debugging Strategies

1. **Use `ros2 topic` commands**:
   - `ros2 topic list` - List all topics
   - `ros2 topic echo <topic_name>` - View messages on a topic
   - `ros2 topic info <topic_name>` - Get information about a topic

2. **Use `ros2 node` commands**:
   - `ros2 node list` - List all active nodes
   - `ros2 node info <node_name>` - Get information about a node

3. **Use `rqt_graph`**:
   - Visualize the node graph to understand connections

## Industry Use Cases

### Industrial Robotics
ROS 2 is widely used in manufacturing for coordinating multiple robotic arms, managing complex assembly processes, and ensuring safe human-robot collaboration.

### Autonomous Vehicles
ROS 2 provides the communication infrastructure for sensor fusion, path planning, and control systems in autonomous vehicles.

### Service Robotics
ROS 2 enables service robots to navigate environments, interact with humans, and perform complex tasks in dynamic settings.

## Summary / Key Takeaways

- ROS 2 provides a distributed computing framework for robotics applications
- The core communication patterns are topics (pub/sub), services (request/response), and actions (goal/feedback/result)
- Nodes are the fundamental execution units that communicate through these patterns
- DDS serves as the underlying middleware for reliable message passing
- Quality of Service settings allow fine-tuning communication behavior

## Practice Tasks / Mini-Projects

### Exercise 1: Basic Publisher-Subscriber
Create a publisher that sends temperature readings and a subscriber that logs these readings to the console.

### Exercise 2: Service Implementation
Implement a service that takes two numbers and returns their sum, difference, product, and quotient.

### Exercise 3: Simple Robot Control
Create a publisher that sends velocity commands to a robot and a subscriber that monitors the robot's status.