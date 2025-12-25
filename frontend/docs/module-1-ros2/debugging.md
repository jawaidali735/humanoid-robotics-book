---
id: module-1-ros2-debugging
title: "Debugging: ROS 2 Troubleshooting"
sidebar_position: 7
---

# Debugging Tips: ROS 2 for Humanoid Robotics

## Introduction

Debugging ROS 2 applications in humanoid robotics presents unique challenges due to the complexity of the systems involved. This section provides comprehensive debugging strategies, tools, and techniques specifically tailored for humanoid robot development with ROS 2.

## Learning Outcomes

After completing this section, you will be able to:
- Identify and resolve common ROS 2 issues in humanoid robotics applications
- Use ROS 2 debugging tools effectively
- Implement logging and monitoring strategies for complex systems
- Apply systematic debugging approaches to multi-node systems

## Conceptual Foundations

### Debugging in Complex Robotic Systems

Humanoid robotics systems present several debugging challenges:
- **Multi-Node Architecture**: Issues can span multiple nodes across different processes
- **Real-time Constraints**: Timing-related issues that are difficult to reproduce
- **Safety-Critical Operations**: Debugging must not compromise robot safety
- **Distributed Systems**: Network-related issues in multi-robot or remote scenarios
- **Hardware Dependencies**: Issues that only occur with physical hardware

### Debugging Philosophy

Effective debugging in ROS 2 follows these principles:
- **Systematic approach**: Isolate components and test individually
- **Reproducible conditions**: Create consistent test scenarios
- **Comprehensive logging**: Capture relevant state information
- **Safety first**: Ensure debugging doesn't create unsafe conditions
- **Incremental complexity**: Start simple, add complexity gradually

## Technical Deep Dive

### ROS 2 Debugging Architecture

ROS 2 provides several layers for debugging:

1. **Application Layer**: Node-specific debugging with logging and error handling
2. **Communication Layer**: Topic/service/action monitoring and inspection
3. **Middleware Layer**: DDS-level debugging for network and communication issues
4. **System Layer**: Process and resource monitoring

### Core Debugging Tools

#### Command Line Tools

**ros2 node**
```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info <node_name>

# Lifecycle node management (for lifecycle nodes)
ros2 lifecycle nodes list
```

**ros2 topic**
```bash
# List all topics
ros2 topic list

# Show information about a topic
ros2 topic info <topic_name>

# Echo messages from a topic
ros2 topic echo <topic_name> <msg_type>

# Publish a single message to a topic
ros2 topic pub <topic_name> <msg_type> <msg_data>
```

**ros2 service**
```bash
# List all services
ros2 service list

# Show information about a service
ros2 service info <service_name>

# Call a service
ros2 service call <service_name> <srv_type> <request_data>
```

**ros2 action**
```bash
# List all actions
ros2 action list

# Send a goal to an action server
ros2 action send_goal <action_name> <action_type> <goal_data>
```

### Advanced Debugging Techniques

#### 1. Logging and Diagnostics

```python
#!/usr/bin/env python3
# Example of comprehensive logging in a humanoid controller

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import traceback


class DebuggingHumanoidController(Node):
    def __init__(self):
        super().__init__('debugging_humanoid_controller')

        # Create a QoS profile for debugging (reliable, transient local)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publishers with different QoS for different purposes
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', qos_profile)
        self.debug_pub = self.create_publisher(Float64MultiArray, 'debug_info', 10)

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        # Debugging state
        self.joint_states = None
        self.last_debug_time = self.get_clock().now()
        self.control_cycle_count = 0

        # Set log level
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().info('Debugging Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Handle joint state messages with comprehensive logging"""
        try:
            self.joint_states = msg
            self.control_cycle_count += 1

            # Log important state changes
            if self.control_cycle_count % 100 == 0:  # Every 100 cycles
                self.get_logger().info(
                    f'Received joint state with {len(msg.name)} joints, '
                    f'timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
                )

            # Check for unusual values
            for i, pos in enumerate(msg.position):
                if abs(pos) > 10.0:  # Unusually large joint position
                    self.get_logger().warn(
                        f'Unusual joint position: {msg.name[i]} = {pos}'
                    )

        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def control_loop(self):
        """Main control loop with comprehensive debugging"""
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_debug_time).nanoseconds / 1e9

            # Log timing information
            if dt > 0.02:  # If loop took longer than expected
                self.get_logger().warn(f'Control loop took {dt:.3f}s (expected ~0.01s)')

            # Perform control calculations
            self.perform_control_calculations()

            # Publish debug information periodically
            if self.control_cycle_count % 50 == 0:  # Every 50 cycles
                self.publish_debug_info()

            self.last_debug_time = current_time

        except Exception as e:
            self.get_logger().error(f'Error in control_loop: {e}')
            self.get_logger().error(traceback.format_exc())
            # Don't crash the node, but ensure safe state
            self.ensure_safe_state()

    def perform_control_calculations(self):
        """Perform control calculations with error handling"""
        if self.joint_states is None:
            return

        try:
            # Example control logic
            target_positions = []
            for pos in self.joint_states.position:
                # Simple proportional control
                target_pos = pos * 0.99  # Dampening factor
                target_positions.append(target_pos)

            # Publish commands
            cmd_msg = JointState()
            cmd_msg.name = self.joint_states.name
            cmd_msg.position = target_positions
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'base_link'

            self.joint_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f'Error in control calculations: {e}')
            self.ensure_safe_state()

    def publish_debug_info(self):
        """Publish debug information for external monitoring"""
        try:
            debug_msg = Float64MultiArray()
            debug_msg.data = [
                float(self.control_cycle_count),
                self.get_clock().now().nanoseconds / 1e9,  # Current time in seconds
                0.0,  # Placeholder for additional debug values
            ]
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing debug info: {e}')

    def ensure_safe_state(self):
        """Ensure robot is in a safe state after error"""
        try:
            # Publish zero joint commands to stop movement
            cmd_msg = JointState()
            if self.joint_states:
                cmd_msg.name = self.joint_states.name
                cmd_msg.position = [0.0] * len(self.joint_states.name)
            else:
                cmd_msg.name = ['default_joint']
                cmd_msg.position = [0.0]

            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'base_link'
            self.joint_pub.publish(cmd_msg)

            self.get_logger().warn('Safe state enforced due to error')

        except Exception as e:
            self.get_logger().error(f'Error enforcing safe state: {e}')

    def destroy_node(self):
        """Override destroy to ensure safe shutdown"""
        self.ensure_safe_state()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = DebuggingHumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down debugging controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 2. Performance Monitoring

```python
#!/usr/bin/env python3
# Performance monitoring node

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import psutil
import os


class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        self.perf_pub = self.create_publisher(Float64MultiArray, 'performance_metrics', 10)
        self.perf_timer = self.create_timer(1.0, self.performance_callback)  # 1Hz

        # Process monitoring
        self.process = psutil.Process(os.getpid())

        self.get_logger().info('Performance Monitor initialized')

    def performance_callback(self):
        """Monitor and publish performance metrics"""
        try:
            # CPU usage
            cpu_percent = self.process.cpu_percent()

            # Memory usage
            memory_info = self.process.memory_info()
            memory_rss = memory_info.rss / (1024 * 1024)  # MB

            # System memory
            system_memory = psutil.virtual_memory().percent

            # Create performance message
            perf_msg = Float64MultiArray()
            perf_msg.data = [
                cpu_percent,        # CPU usage (%)
                memory_rss,         # Process memory (MB)
                system_memory,      # System memory (%)
                0.0,                # Placeholder for additional metrics
            ]

            self.perf_pub.publish(perf_msg)

            # Log warnings for performance issues
            if cpu_percent > 80:
                self.get_logger().warn(f'High CPU usage: {cpu_percent:.1f}%')
            if memory_rss > 100:  # More than 100MB
                self.get_logger().warn(f'High memory usage: {memory_rss:.1f}MB')

        except Exception as e:
            self.get_logger().error(f'Error in performance monitoring: {e}')


def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Practical Implementation

### Setting Up a Debugging Environment

1. **Create a debugging configuration file** (`config/debugging.yaml`):

```yaml
debugging_humanoid_controller:
  ros__parameters:
    # Logging configuration
    log_level: "INFO"
    enable_detailed_logging: true

    # Debugging parameters
    enable_performance_monitoring: true
    enable_safety_monitoring: true

    # Timing parameters for debugging
    control_loop_timeout: 0.02  # 20ms timeout
    debug_publish_frequency: 10  # 10Hz for debug topics
```

2. **Launch file for debugging** (`launch/debugging.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_exercises'),
        'config',
        'debugging.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_exercises',
            executable='debugging_humanoid_controller',
            name='debugging_humanoid_controller',
            parameters=[config],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='robot_exercises',
            executable='performance_monitor',
            name='performance_monitor',
            parameters=[config],
            output='screen'
        )
    ])
```

### Common Debugging Workflows

#### Workflow 1: Node Communication Issues

1. **Verify nodes are running**:
```bash
ros2 node list
```

2. **Check topic connections**:
```bash
ros2 topic info /your_topic_name
```

3. **Monitor messages**:
```bash
ros2 topic echo /your_topic_name
```

4. **Check for message drops**:
```bash
ros2 topic hz /your_topic_name
```

#### Workflow 2: Performance Issues

1. **Monitor system resources**:
```bash
htop
# or
ros2 run robot_exercises performance_monitor
```

2. **Check message frequency**:
```bash
ros2 topic hz /high_frequency_topic
```

3. **Analyze node computation time**:
```bash
# Add timing code to your nodes
start_time = time.time()
# ... your code ...
end_time = time.time()
print(f"Function took {end_time - start_time} seconds")
```

#### Workflow 3: Service/Action Issues

1. **Verify service availability**:
```bash
ros2 service list
ros2 service info /your_service_name
```

2. **Test service directly**:
```bash
ros2 service call /your_service_name your_package/srv/ServiceType "{request_field: value}"
```

3. **Check action status**:
```bash
ros2 action list
ros2 action info /your_action_name
```

### Advanced Debugging Techniques

#### 1. Remote Debugging Setup

For humanoid robots that are physically remote:

```python
#!/usr/bin/env python3
# Remote debugging bridge

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import subprocess
import socket


class RemoteDebugBridge(Node):
    def __init__(self):
        super().__init__('remote_debug_bridge')

        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.remote_debug_pub = self.create_publisher(String, 'remote_debug_commands', 10)

        self.diag_timer = self.create_timer(5.0, self.publish_diagnostics)

        self.get_logger().info('Remote Debug Bridge initialized')

    def publish_diagnostics(self):
        """Publish system diagnostics for remote monitoring"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # System status
        status = DiagnosticStatus()
        status.name = f"{socket.gethostname()}_system_status"
        status.level = DiagnosticStatus.OK
        status.message = "System operational"

        # Add key-value pairs for monitoring
        status.values.extend([
            DiagnosticStatus.KeyValue(key="cpu_load", value=str(psutil.cpu_percent())),
            DiagnosticStatus.KeyValue(key="memory_usage", value=str(psutil.virtual_memory().percent)),
            DiagnosticStatus.KeyValue(key="disk_usage", value=str(psutil.disk_usage('/').percent)),
        ])

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    bridge = RemoteDebugBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 2. Replay and Analysis Tools

Use ROS 2 bag files for debugging:

```bash
# Record all topics
ros2 bag record -a -o debug_session

# Record specific topics
ros2 bag record /joint_states /cmd_vel /diagnostics

# Play back recorded data
ros2 bag play debug_session

# Analyze recorded data
ros2 bag info debug_session
```

## Common Pitfalls & Debugging Tips

### Common Issues in Humanoid ROS 2 Systems

1. **Timing and Synchronization Issues**
   - **Symptoms**: Unstable walking, missed control cycles, sensor fusion problems
   - **Diagnosis**: Use `ros2 topic hz` to check message rates, add timing logs
   - **Solution**: Implement proper real-time scheduling, use appropriate QoS settings

2. **Memory Leaks in Long-Running Systems**
   - **Symptoms**: System slowing down over time, eventual crash
   - **Diagnosis**: Monitor with performance tools, use memory profiling
   - **Solution**: Implement proper cleanup, avoid circular references

3. **Network Communication Problems**
   - **Symptoms**: Intermittent message drops, high latency
   - **Diagnosis**: Check network configuration, use QoS tools
   - **Solution**: Optimize QoS settings, implement redundancy

4. **Safety System Interference**
   - **Symptoms**: Unexpected emergency stops, safety system false triggers
   - **Diagnosis**: Log safety system state, analyze trigger conditions
   - **Solution**: Properly tune safety thresholds, implement hysteresis

### Debugging Strategies

1. **Isolation Method**
   - Test individual nodes in isolation
   - Use mock data for dependencies
   - Gradually add components back

2. **Logging Hierarchy**
   - ERROR: Critical issues that stop operation
   - WARN: Issues that may affect performance
   - INFO: Normal operation information
   - DEBUG: Detailed information for debugging

3. **Incremental Complexity**
   - Start with simple test cases
   - Add complexity step by step
   - Verify at each step

4. **Reproducible Testing**
   - Use bag files for consistent test data
   - Implement unit tests for critical functions
   - Create automated test suites

### Debugging Tools and Commands

#### Essential Commands
```bash
# Monitor all ROS 2 communications
ros2 topic list && ros2 service list && ros2 action list

# Check node health
ros2 run lifecycle lifecycle_service_client __node:=test_client

# Visualize the computation graph
rqt_graph

# Monitor topics visually
rqt_plot

# Check system resources
ros2 run top top

# Bag file analysis
ros2 bag info /path/to/bag
```

#### Custom Debugging Scripts
```bash
#!/bin/bash
# debug_humonoid.sh - Comprehensive debugging script

echo "=== ROS 2 Humanoid Robot Debug Report ==="
echo "Timestamp: $(date)"
echo

echo "--- Active Nodes ---"
ros2 node list
echo

echo "--- Topic Status ---"
for topic in $(ros2 topic list); do
    echo "Topic: $topic"
    ros2 topic info $topic
    echo
done

echo "--- Service Status ---"
ros2 service list
echo

echo "--- Action Status ---"
ros2 action list
echo

echo "Report complete."
```

## Industry Use Cases

### Development Phase Debugging
Companies use systematic debugging approaches during development:
- Automated testing suites that run after each build
- Continuous integration with ROS 2-specific tests
- Simulation-based debugging before hardware testing

### Field Debugging
For deployed humanoid robots:
- Remote monitoring and diagnostics
- Secure debugging interfaces
- Automated issue reporting and resolution

### Safety-Critical Debugging
In safety-critical applications:
- Non-intrusive debugging methods
- Comprehensive logging for incident analysis
- Fail-safe debugging procedures

## Summary / Key Takeaways

- Debugging humanoid ROS 2 systems requires specialized tools and techniques
- Systematic approaches are more effective than random troubleshooting
- Logging and monitoring are essential for complex systems
- Safety must be maintained during debugging
- Reproducible test conditions improve debugging efficiency

## Practice Tasks / Mini-Projects

### Exercise 1: Debugging Tools Setup
Create a comprehensive debugging setup for a humanoid robot simulation, including logging, performance monitoring, and diagnostic publishing.

### Exercise 2: Issue Reproduction
Create a ROS 2 node with a subtle bug (like a memory leak or timing issue), then use debugging tools to identify and fix the issue.

### Exercise 3: Remote Debugging Interface
Implement a remote debugging interface that allows external monitoring and control of a humanoid robot system.

### Exercise 4: Automated Testing
Create an automated test suite that validates different aspects of a humanoid robot's ROS 2 system and generates debug reports.