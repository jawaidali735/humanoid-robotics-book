---
id: module-1-ros2-toolchain
title: "Toolchain: rclpy Python Integration"
sidebar_position: 3
---

# Toolchain Overview: rclpy Python Integration

## Introduction

This section covers the Python client library for ROS 2 (rclpy), which allows you to write ROS 2 nodes in Python. rclpy provides the Python API for ROS 2, enabling you to create publishers, subscribers, services, actions, and other ROS 2 entities using Python.

## Learning Outcomes

After completing this section, you will be able to:
- Set up a Python development environment for ROS 2
- Create ROS 2 nodes using rclpy
- Implement publishers and subscribers in Python
- Develop services and actions using Python
- Debug Python-based ROS 2 nodes

## Conceptual Foundations

### What is rclpy?

rclpy is the Python client library for ROS 2. It provides a Python API that allows you to create ROS 2 nodes and interact with the ROS 2 ecosystem. rclpy is built on top of the ROS Client Library (rcl) and provides Python-specific abstractions for ROS 2 concepts.

### Python in Robotics

Python is widely used in robotics for:
- Rapid prototyping and development
- Algorithm development and testing
- Data processing and analysis
- Machine learning integration
- Testing and debugging

## Technical Deep Dive

### rclpy Architecture

The rclpy architecture consists of:
1. **Python API**: High-level Python classes and functions
2. **Python Bindings**: CPython extensions that interface with rcl
3. **ROS Client Library (rcl)**: Language-agnostic ROS 2 client library
4. **ROS Middleware Interface (RMW)**: Abstraction layer for DDS implementations

### Core rclpy Components

#### Node
The base class for creating ROS 2 nodes in Python:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code
```

#### Publishers and Subscribers
```python
from std_msgs.msg import String

# Creating a publisher
publisher = self.create_publisher(String, 'topic_name', 10)

# Creating a subscriber
subscriber = self.create_subscription(
    String,
    'topic_name',
    self.callback_function,
    10
)
```

#### Services and Clients
```python
from example_interfaces.srv import AddTwoInts

# Creating a service
service = self.create_service(AddTwoInts, 'service_name', self.service_callback)

# Creating a client
client = self.create_client(AddTwoInts, 'service_name')
```

### Threading and Execution Models

rclpy supports different execution models:
- **Single-threaded executor**: Processes all callbacks sequentially
- **Multi-threaded executor**: Processes callbacks in parallel using threads
- **Custom executors**: For specialized execution requirements

## Practical Implementation

### Setting up Python Development Environment

1. **Install ROS 2 Python packages**:
```bash
sudo apt update
sudo apt install python3-rosdep2 python3-vcstool
```

2. **Create a virtual environment** (optional but recommended):
```bash
python3 -m venv ros2_env
source ros2_env/bin/activate
```

3. **Source ROS 2**:
```bash
source /opt/ros/humble/setup.bash
```

### Creating Your First Python Node

1. **Create a package**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_py_pkg
```

2. **Create a Python node file** (`my_py_pkg/my_py_pkg/talker.py`):
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    talker = Talker()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

3. **Update setup.py** to include the executable:
```python
entry_points={
    'console_scripts': [
        'talker = my_py_pkg.talker:main',
        'listener = my_py_pkg.listener:main',
    ],
},
```

4. **Build and run**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_py_pkg
source install/setup.bash
ros2 run my_py_pkg talker
```

### Implementing Publishers and Subscribers

1. **Publisher node** (`publisher_member_function.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

2. **Subscriber node** (`subscriber_member_function.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### Working with Services

1. **Service server**:
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

2. **Service client**:
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Common Pitfalls & Debugging Tips

### Common Issues

1. **Import Errors**:
   - Symptoms: "No module named rclpy" or similar
   - Solution: Ensure ROS 2 environment is sourced and Python packages are installed

2. **Node Naming Conflicts**:
   - Symptoms: Nodes with the same name cannot run simultaneously
   - Solution: Use unique node names or namespaces

3. **Callback Threading Issues**:
   - Symptoms: Callbacks not executing as expected
   - Solution: Understand the execution model being used

4. **Message Type Mismatches**:
   - Symptoms: Nodes not communicating due to incompatible message types
   - Solution: Verify message types match between publishers and subscribers

### Debugging Strategies

1. **Use Python logging**:
   - Use `self.get_logger().info()` for debugging messages
   - Set appropriate log levels for different contexts

2. **ROS 2 command line tools**:
   - `ros2 run` to execute Python nodes
   - `ros2 topic echo` to monitor topics
   - `ros2 node info` to inspect node details

3. **Python-specific debugging**:
   - Use `pdb` for interactive debugging
   - Add print statements for simple debugging
   - Use IDE debuggers for more complex debugging

4. **Check ROS environment**:
   - Verify `ROS_DOMAIN_ID` is set correctly
   - Ensure the correct ROS distribution is sourced

## Industry Use Cases

### Research and Development
Python is extensively used in robotics research for rapid prototyping, algorithm development, and experimentation due to its ease of use and rich ecosystem of scientific libraries.

### Educational Robotics
Python's simplicity makes it ideal for teaching robotics concepts, allowing students to focus on robotics principles rather than complex syntax.

### Data Processing and Analysis
Python's strength in data science makes it perfect for processing sensor data, analyzing robot performance, and machine learning integration.

### Testing and Simulation
Python is commonly used for creating test frameworks and simulation environments for robotic systems.

## Summary / Key Takeaways

- rclpy is the Python client library for ROS 2, providing Python APIs for ROS 2 concepts
- Python nodes are created by inheriting from the Node class
- Publishers, subscribers, services, and actions can all be implemented in Python
- Python's ecosystem makes it ideal for rapid prototyping and data processing
- Understanding execution models is important for proper callback handling

## Practice Tasks / Mini-Projects

### Exercise 1: Python Publisher-Subscriber Pair
Create a publisher that sends temperature readings and a subscriber that logs these readings with timestamps.

### Exercise 2: Service Implementation in Python
Implement a service that takes a string command and returns a processed response, such as converting to uppercase.

### Exercise 3: Parameter Server Usage
Create a node that accepts parameters and changes its behavior based on those parameters, demonstrating how to work with ROS 2 parameters in Python.