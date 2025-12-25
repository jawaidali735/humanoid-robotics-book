---
id: module-2-simulation-debugging
title: "Debugging: Simulation Troubleshooting"
sidebar_position: 7
---

# Debugging: Simulation-Specific Issues in Humanoid Robotics

## Introduction

Debugging humanoid robot simulations presents unique challenges that differ significantly from debugging real hardware. This section covers specialized debugging techniques for simulation-specific issues, including physics instabilities, sensor simulation problems, and control system integration challenges. We'll explore tools and methodologies specifically designed for identifying and resolving issues in simulation environments, with a focus on humanoid robotics applications.

## Learning Outcomes

After completing this section, you will be able to:
- Identify and diagnose common simulation-specific issues in humanoid robotics
- Use specialized debugging tools for physics, sensor, and control systems
- Apply systematic approaches to resolve simulation instabilities
- Validate simulation accuracy and identify sources of error
- Implement debugging strategies that bridge simulation and reality

## Conceptual Foundations

### Simulation Debugging Challenges

Debugging humanoid robot simulations presents several unique challenges:

1. **Virtual Environment Complexity**: Multiple interconnected systems (physics, sensors, controls) that interact in complex ways
2. **Non-Intuitive Physics**: Simulation physics may behave differently than expected
3. **Timing Dependencies**: Issues that arise from timing mismatches between different systems
4. **Parameter Sensitivity**: Small changes in parameters can cause large behavioral differences
5. **Validation Difficulty**: Hard to determine if behavior is correct or just plausible

### Debugging Methodology for Simulation

Effective simulation debugging follows a systematic approach:

1. **Isolation**: Identify which subsystem (physics, sensors, controls) is causing the issue
2. **Reproduction**: Create minimal test cases that reproduce the issue consistently
3. **Hypothesis Testing**: Formulate and test hypotheses about root causes
4. **Parameter Analysis**: Examine how parameter changes affect behavior
5. **Validation**: Verify fixes against expected physical principles

### Simulation-Specific Debugging Patterns

Common patterns in simulation debugging include:

- **Physics Instability**: Objects behaving unrealistically or simulation becoming unstable
- **Sensor Inaccuracy**: Simulated sensors not matching expected behavior
- **Control System Issues**: Controllers not working as expected in simulation
- **Timing Problems**: Issues related to simulation time vs. real time
- **Model Inaccuracy**: Robot models not behaving as expected

## Technical Deep Dive

### Physics Debugging

#### Identifying Physics Issues

Physics issues in humanoid simulation can manifest in several ways:

1. **Instability**: Robot joints oscillating or exploding
2. **Penetration**: Robot parts passing through each other or environment
3. **Jittering**: Small, rapid movements that shouldn't occur
4. **Unrealistic Motion**: Movements that don't match expected physics

#### Debugging Physics Parameters

```xml
<!-- Example of physics debugging in URDF -->
<link name="debug_link">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.083"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>

<!-- Debug physics parameters -->
<gazebo reference="debug_link">
  <self_collide>false</self_collide>
  <enable_wind>false</enable_wind>
  <kinematic>false</kinematic>
  <gravity>true</gravity>
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <fdir1>0 0 0</fdir1>
  <slip1>0.0</slip1>
  <slip2>0.0</slip2>
  <kp>1000000000000.0</kp>
  <kd>1.0</kd>
  <max_vel>100.0</max_vel>
  <min_depth>0.001</min_depth>
</gazebo>
```

#### Physics Debugging Tools

```python
#!/usr/bin/env python3
# physics_debugger.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt

class PhysicsDebugger(Node):
    def __init__(self):
        super().__init__('physics_debugger')
        
        # Subscribers for physics data
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.contact_sub = self.create_subscription(
            WrenchStamped, '/contact_force', self.contact_callback, 10)
        
        # Publishers for debug information
        self.debug_pub = self.create_publisher(
            Float64MultiArray, '/physics_debug_info', 10)
        
        # Data storage for analysis
        self.joint_history = []
        self.contact_history = []
        
        # Timer for analysis (1Hz)
        self.analysis_timer = self.create_timer(1.0, self.analyze_physics)
        
        self.get_logger().info('Physics Debugger initialized')

    def joint_callback(self, msg):
        """Store joint data for physics analysis"""
        joint_data = {
            'timestamp': msg.header.stamp,
            'positions': np.array(msg.position),
            'velocities': np.array(msg.velocity),
            'efforts': np.array(msg.effort)
        }
        self.joint_history.append(joint_data)
        
        # Keep only recent data to prevent memory issues
        if len(self.joint_history) > 1000:
            self.joint_history = self.joint_history[-1000:]

    def contact_callback(self, msg):
        """Store contact force data"""
        contact_data = {
            'timestamp': msg.header.stamp,
            'force': np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]),
            'torque': np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        }
        self.contact_history.append(contact_data)

    def analyze_physics(self):
        """Analyze physics data for issues"""
        if len(self.joint_history) < 10:
            return

        # Check for joint instabilities
        position_variance = self.calculate_position_variance()
        velocity_spikes = self.detect_velocity_spikes()
        effort_anomalies = self.detect_effort_anomalies()
        
        # Create debug report
        debug_info = Float64MultiArray()
        debug_info.data = [
            position_variance,
            velocity_spikes,
            effort_anomalies,
            len(self.joint_history)
        ]
        
        self.debug_pub.publish(debug_info)
        
        # Log findings
        if position_variance > 1.0:
            self.get_logger().warn(f'High position variance detected: {position_variance}')
        if velocity_spikes > 0:
            self.get_logger().warn(f'Velocity spikes detected: {velocity_spikes}')
        if effort_anomalies > 0:
            self.get_logger().warn(f'Effort anomalies detected: {effort_anomalies}')

    def calculate_position_variance(self):
        """Calculate variance in joint positions over time"""
        if len(self.joint_history) < 2:
            return 0.0
        
        positions = np.array([entry['positions'] for entry in self.joint_history])
        variance = np.var(positions, axis=0)
        return np.mean(variance)

    def detect_velocity_spikes(self):
        """Detect sudden changes in joint velocities"""
        if len(self.joint_history) < 2:
            return 0
        
        velocity_changes = []
        for i in range(1, len(self.joint_history)):
            prev_vel = self.joint_history[i-1]['velocities']
            curr_vel = self.joint_history[i]['velocities']
            change = np.abs(curr_vel - prev_vel)
            velocity_changes.append(change)
        
        if not velocity_changes:
            return 0
        
        velocity_changes = np.array(velocity_changes)
        # Count spikes as changes > 10 rad/s
        spikes = np.sum(velocity_changes > 10.0)
        return spikes

    def detect_effort_anomalies(self):
        """Detect unusual effort values"""
        if len(self.joint_history) < 1:
            return 0
        
        efforts = np.array([entry['efforts'] for entry in self.joint_history])
        # Count efforts > 95th percentile as anomalies
        effort_threshold = np.percentile(np.abs(efforts), 95)
        anomalies = np.sum(np.abs(efforts) > effort_threshold)
        return anomalies

def main(args=None):
    rclpy.init(args=args)
    debugger = PhysicsDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        debugger.get_logger().info('Shutting down physics debugger')
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Sensor Debugging

#### Common Sensor Issues

Simulated sensors can exhibit various problems:

1. **Noise Mismatch**: Simulated noise doesn't match real sensor characteristics
2. **Timing Issues**: Sensor data published at incorrect rates
3. **Calibration Problems**: Sensor parameters don't match expected values
4. **Range Limitations**: Sensors not detecting objects at expected distances
5. **Field of View Issues**: Sensors seeing or missing objects incorrectly

#### Sensor Debugging Tools

```python
#!/usr/bin/env python3
# sensor_debugger.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy import stats
import cv2
from cv_bridge import CvBridge

class SensorDebugger(Node):
    def __init__(self):
        super().__init__('sensor_debugger')
        
        # Subscribers for different sensor types
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publishers for debug analysis
        self.sensor_health_pub = self.create_publisher(Float64MultiArray, '/sensor_health', 10)
        
        # Data storage
        self.imu_data = []
        self.scan_data = []
        self.image_data = []
        
        # Analysis timer
        self.analysis_timer = self.create_timer(2.0, self.analyze_sensors)
        
        # CV Bridge for image processing
        self.cv_bridge = CvBridge()
        
        self.get_logger().info('Sensor Debugger initialized')

    def imu_callback(self, msg):
        """Analyze IMU data for issues"""
        imu_sample = {
            'timestamp': msg.header.stamp,
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }
        self.imu_data.append(imu_sample)
        
        # Keep only recent data
        if len(self.imu_data) > 500:
            self.imu_data = self.imu_data[-500:]

    def scan_callback(self, msg):
        """Analyze LiDAR data for issues"""
        scan_sample = {
            'timestamp': msg.header.stamp,
            'ranges': np.array(msg.ranges),
            'intensities': np.array(msg.intensities) if msg.intensities else None,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }
        self.scan_data.append(scan_sample)

    def image_callback(self, msg):
        """Analyze camera data for issues"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_sample = {
                'timestamp': msg.header.stamp,
                'image': cv_image,
                'height': msg.height,
                'width': msg.width,
                'encoding': msg.encoding
            }
            self.image_data.append(image_sample)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def analyze_sensors(self):
        """Analyze all sensor data for issues"""
        imu_health = self.analyze_imu_health()
        scan_health = self.analyze_scan_health()
        image_health = self.analyze_image_health()
        
        # Publish sensor health report
        health_report = Float64MultiArray()
        health_report.data = [
            imu_health,
            scan_health,
            image_health
        ]
        self.sensor_health_pub.publish(health_report)
        
        # Log issues
        if imu_health < 0.8:
            self.get_logger().warn(f'IMU health is poor: {imu_health:.2f}')
        if scan_health < 0.8:
            self.get_logger().warn(f'LiDAR health is poor: {scan_health:.2f}')
        if image_health < 0.8:
            self.get_logger().warn(f'Camera health is poor: {image_health:.2f}')

    def analyze_imu_health(self):
        """Analyze IMU data health"""
        if len(self.imu_data) < 10:
            return 1.0
        
        # Check for NaN values
        linear_acc_data = np.array([sample['linear_acceleration'] for sample in self.imu_data])
        nan_count = np.sum(np.isnan(linear_acc_data))
        total_values = linear_acc_data.size
        
        # Check for realistic gravity detection
        avg_gravity = np.mean(np.linalg.norm(linear_acc_data, axis=1))
        gravity_health = 1.0 - abs(avg_gravity - 9.81) / 9.81
        
        # Overall health score
        nan_health = 1.0 - (nan_count / total_values) if total_values > 0 else 1.0
        overall_health = 0.6 * gravity_health + 0.4 * nan_health
        
        return max(overall_health, 0.0)

    def analyze_scan_health(self):
        """Analyze LiDAR scan health"""
        if len(self.scan_data) < 1:
            return 1.0
        
        latest_scan = self.scan_data[-1]
        ranges = latest_scan['ranges']
        
        # Check for valid range values
        valid_ranges = np.sum((ranges >= latest_scan['range_min']) & 
                             (ranges <= latest_scan['range_max']) & 
                             (~np.isnan(ranges)) & 
                             (~np.isinf(ranges)))
        total_ranges = len(ranges)
        
        health = valid_ranges / total_ranges if total_ranges > 0 else 1.0
        return health

    def analyze_image_health(self):
        """Analyze camera image health"""
        if len(self.image_data) < 1:
            return 1.0
        
        latest_image = self.image_data[-1]
        img = latest_image['image']
        
        # Check for basic image properties
        if img is None:
            return 0.0
        
        # Check for uniform color (indicating no real data)
        mean_color = np.mean(img, axis=(0,1))
        std_color = np.std(img, axis=(0,1))
        
        # If standard deviation is very low, image might be uniform
        if np.all(std_color < 1.0):
            return 0.3  # Low health for uniform image
        
        # Check for reasonable brightness
        brightness = np.mean(img)
        if brightness < 10 or brightness > 245:
            return 0.7  # Low health for very dark/bright image
        
        return 1.0  # Good health for normal image

def main(args=None):
    rclpy.init(args=args)
    debugger = SensorDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        debugger.get_logger().info('Shutting down sensor debugger')
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Control System Debugging

#### Common Control Issues in Simulation

Control systems in simulation can have unique problems:

1. **Simulation vs. Real Time**: Control loops running at different rates than expected
2. **Model Mismatch**: Control models not matching simulation dynamics
3. **Actuator Limitations**: Not accounting for simulated actuator constraints
4. **Sensor Feedback Issues**: Delayed or inaccurate sensor feedback
5. **Integration Errors**: Numerical integration problems in control algorithms

#### Control Debugging Tools

```python
#!/usr/bin/env python3
# control_debugger.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class ControlDebugger(Node):
    def __init__(self):
        super().__init__('control_debugger')
        
        # Subscribers for control data
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.controller_state_sub = self.create_subscription(
            JointTrajectoryControllerState, '/joint_trajectory_controller/state', 
            self.controller_state_callback, 10)
        self.command_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 
            self.command_callback, 10)
        
        # Publishers for debug analysis
        self.control_analysis_pub = self.create_publisher(
            Float64MultiArray, '/control_analysis', 10)
        
        # Storage for control data
        self.joint_states = []
        self.controller_states = []
        self.commands = []
        
        # Timing analysis
        self.last_command_time = None
        self.command_intervals = []
        
        # Analysis timer
        self.analysis_timer = self.create_timer(1.0, self.analyze_control)
        
        self.get_logger().info('Control Debugger initialized')

    def joint_state_callback(self, msg):
        """Store joint state data"""
        state_data = {
            'timestamp': msg.header.stamp,
            'names': msg.name,
            'positions': np.array(msg.position),
            'velocities': np.array(msg.velocity) if msg.velocity else np.zeros(len(msg.position)),
            'efforts': np.array(msg.effort) if msg.effort else np.zeros(len(msg.position))
        }
        self.joint_states.append(state_data)
        
        if len(self.joint_states) > 1000:
            self.joint_states = self.joint_states[-1000:]

    def controller_state_callback(self, msg):
        """Store controller state data"""
        state_data = {
            'timestamp': msg.header.stamp,
            'joint_names': msg.joint_names,
            'desired_positions': np.array(msg.desired.positions),
            'actual_positions': np.array(msg.actual.positions),
            'error_positions': np.array(msg.error.positions),
            'desired_velocities': np.array(msg.desired.velocities),
            'actual_velocities': np.array(msg.actual.velocities),
            'error_velocities': np.array(msg.error.velocities)
        }
        self.controller_states.append(state_data)

    def command_callback(self, msg):
        """Store command data and analyze timing"""
        command_data = {
            'timestamp': msg.header.stamp,
            'joint_names': msg.joint_names,
            'points': msg.points
        }
        self.commands.append(command_data)
        
        # Analyze command timing
        if self.last_command_time is not None:
            interval = (msg.header.stamp.sec - self.last_command_time.sec) + \
                      (msg.header.stamp.nanosec - self.last_command_time.nanosec) / 1e9
            self.command_intervals.append(interval)
        
        self.last_command_time = msg.header.stamp
        
        if len(self.command_intervals) > 100:
            self.command_intervals = self.command_intervals[-100:]

    def analyze_control(self):
        """Analyze control system performance"""
        position_error = self.calculate_position_error()
        timing_jitter = self.calculate_timing_jitter()
        control_stability = self.calculate_control_stability()
        
        # Publish analysis
        analysis = Float64MultiArray()
        analysis.data = [
            position_error,
            timing_jitter,
            control_stability,
            len(self.joint_states)
        ]
        self.control_analysis_pub.publish(analysis)
        
        # Log issues
        if position_error > 0.1:  # 10cm error threshold
            self.get_logger().warn(f'High position error: {position_error:.3f}')
        if timing_jitter > 0.01:  # 10ms jitter threshold
            self.get_logger().warn(f'High timing jitter: {timing_jitter:.3f}')
        if control_stability < 0.7:  # Stability threshold
            self.get_logger().warn(f'Low control stability: {control_stability:.3f}')

    def calculate_position_error(self):
        """Calculate average position error"""
        if len(self.controller_states) < 1:
            return 0.0
        
        latest_state = self.controller_states[-1]
        error_norms = np.abs(latest_state['error_positions'])
        avg_error = np.mean(error_norms)
        return avg_error

    def calculate_timing_jitter(self):
        """Calculate timing jitter in command intervals"""
        if len(self.command_intervals) < 2:
            return 0.0
        
        intervals = np.array(self.command_intervals)
        std_interval = np.std(intervals)
        return std_interval

    def calculate_control_stability(self):
        """Calculate control stability based on error trends"""
        if len(self.controller_states) < 10:
            return 1.0
        
        # Look at recent error trends
        recent_states = self.controller_states[-10:]
        errors = [np.mean(np.abs(state['error_positions'])) for state in recent_states]
        
        # Calculate trend (stable if errors are not increasing)
        if len(errors) > 1:
            trend = np.polyfit(range(len(errors)), errors, 1)[0]
            # Negative trend is good (errors decreasing)
            stability = 1.0 / (1.0 + abs(trend) * 10) if trend >= 0 else 1.0
        else:
            stability = 1.0
        
        return min(stability, 1.0)

def main(args=None):
    rclpy.init(args=args)
    debugger = ControlDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        debugger.get_logger().info('Shutting down control debugger')
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Gazebo-Specific Debugging

#### Common Gazebo Issues

Gazebo simulation has its own set of debugging challenges:

1. **Performance Issues**: Slow simulation due to complex models or scenes
2. **Plugin Problems**: Gazebo plugins not working as expected
3. **Physics Engine Issues**: ODE, Bullet, or DART-specific problems
4. **Communication Issues**: Problems with ROS-Gazebo bridge
5. **Rendering Problems**: Visualization issues affecting debugging

#### Gazebo Debugging Commands

```bash
# Monitor Gazebo performance
gz stats

# List all Gazebo topics
gz topic -l

# Monitor specific topic
gz topic -e /gazebo/default/robot_name/joint_states

# Monitor world statistics
gz topic -e /gazebo/default/world_stats

# Check Gazebo model information
gz model -m robot_name -i

# List all models in simulation
gz model -l
```

### Simulation Validation and Debugging Framework

```python
#!/usr/bin/env python3
# simulation_debugging_framework.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import json

class SimulationDebuggingFramework(Node):
    def __init__(self):
        super().__init__('simulation_debugging_framework')
        
        # Subscribers for all system data
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.contact_sub = self.create_subscription(WrenchStamped, '/contact_force', self.contact_callback, 10)
        
        # Publishers for debugging output
        self.debug_report_pub = self.create_publisher(String, '/debug_report', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, '/debug_visualization', 10)
        self.metrics_pub = self.create_publisher(Float64MultiArray, '/debug_metrics', 10)
        
        # Data storage
        self.data_history = {
            'joint_states': [],
            'imu_data': [],
            'contact_forces': []
        }
        
        # Debugging parameters
        self.debug_thresholds = {
            'position_variance': 0.5,
            'velocity_spike': 5.0,
            'imu_gravity_error': 0.5,
            'contact_force_spike': 100.0
        }
        
        # Analysis timer
        self.analysis_timer = self.create_timer(0.5, self.perform_debug_analysis)
        
        # Visualization timer
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)
        
        self.get_logger().info('Simulation Debugging Framework initialized')

    def joint_callback(self, msg):
        """Store joint state data"""
        joint_data = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'names': msg.name,
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort)
        }
        self.data_history['joint_states'].append(joint_data)
        
        # Limit history size
        if len(self.data_history['joint_states']) > 2000:
            self.data_history['joint_states'] = self.data_history['joint_states'][-1000:]

    def imu_callback(self, msg):
        """Store IMU data"""
        imu_data = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }
        self.data_history['imu_data'].append(imu_data)
        
        if len(self.data_history['imu_data']) > 1000:
            self.data_history['imu_data'] = self.data_history['imu_data'][-500:]

    def contact_callback(self, msg):
        """Store contact force data"""
        contact_data = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'force': [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z],
            'torque': [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        }
        self.data_history['contact_forces'].append(contact_data)
        
        if len(self.data_history['contact_forces']) > 500:
            self.data_history['contact_forces'] = self.data_history['contact_forces'][-250:]

    def perform_debug_analysis(self):
        """Perform comprehensive debug analysis"""
        results = {}
        
        # Analyze joint states
        results['joint_analysis'] = self.analyze_joint_states()
        
        # Analyze IMU data
        results['imu_analysis'] = self.analyze_imu_data()
        
        # Analyze contact forces
        results['contact_analysis'] = self.analyze_contact_forces()
        
        # Overall system health
        results['system_health'] = self.calculate_system_health(results)
        
        # Generate debug report
        report = self.generate_debug_report(results)
        
        # Publish results
        report_msg = String()
        report_msg.data = json.dumps(results, indent=2)
        self.debug_report_pub.publish(report_msg)
        
        # Publish metrics
        metrics = Float64MultiArray()
        metrics.data = [
            results['system_health'],
            results['joint_analysis']['instability_score'],
            results['imu_analysis']['gravity_accuracy'],
            results['contact_analysis']['max_force']
        ]
        self.metrics_pub.publish(metrics)
        
        # Log significant issues
        if results['system_health'] < 0.7:
            self.get_logger().warn(f'System health is low: {results["system_health"]:.2f}')
            for issue in report['issues']:
                self.get_logger().warn(f'Debug issue: {issue}')

    def analyze_joint_states(self):
        """Analyze joint state data for issues"""
        if len(self.data_history['joint_states']) < 10:
            return {'instability_score': 0.0, 'issues': []}
        
        # Extract position data
        positions = []
        velocities = []
        efforts = []
        
        for state in self.data_history['joint_states']:
            positions.append(state['positions'])
            velocities.append(state['velocities'])
            efforts.append(state['efforts'])
        
        positions = np.array(positions)
        velocities = np.array(velocities)
        efforts = np.array(efforts)
        
        # Calculate statistics
        pos_variance = np.mean(np.var(positions, axis=0))
        vel_spikes = np.sum(np.abs(velocities) > self.debug_thresholds['velocity_spike'])
        effort_peaks = np.sum(np.abs(efforts) > 50.0)  # Assuming 50.0 as effort threshold
        
        # Instability score (lower is better)
        instability_score = min(pos_variance * 2 + vel_spikes * 0.1 + effort_peaks * 0.05, 1.0)
        
        issues = []
        if pos_variance > self.debug_thresholds['position_variance']:
            issues.append(f'High position variance: {pos_variance:.3f}')
        if vel_spikes > 0:
            issues.append(f'Velocity spikes detected: {vel_spikes}')
        if effort_peaks > 0:
            issues.append(f'High effort values detected: {effort_peaks}')
        
        return {
            'instability_score': instability_score,
            'position_variance': pos_variance,
            'velocity_spikes': vel_spikes,
            'effort_peaks': effort_peaks,
            'issues': issues
        }

    def analyze_imu_data(self):
        """Analyze IMU data for issues"""
        if len(self.data_history['imu_data']) < 10:
            return {'gravity_accuracy': 1.0, 'issues': []}
        
        linear_accs = []
        for data in self.data_history['imu_data']:
            linear_accs.append(data['linear_acceleration'])
        
        linear_accs = np.array(linear_accs)
        
        # Calculate average acceleration magnitude (should be ~9.81 for gravity)
        acc_magnitudes = np.linalg.norm(linear_accs, axis=1)
        avg_magnitude = np.mean(acc_magnitudes)
        gravity_error = abs(avg_magnitude - 9.81)
        
        # Gravity accuracy (higher is better)
        gravity_accuracy = 1.0 - min(gravity_error / 9.81, 1.0)
        
        issues = []
        if gravity_error > self.debug_thresholds['imu_gravity_error']:
            issues.append(f'Gravity magnitude error: expected 9.81, got {avg_magnitude:.3f}')
        
        return {
            'gravity_accuracy': gravity_accuracy,
            'avg_magnitude': avg_magnitude,
            'gravity_error': gravity_error,
            'issues': issues
        }

    def analyze_contact_forces(self):
        """Analyze contact force data for issues"""
        if len(self.data_history['contact_forces']) < 2:
            return {'max_force': 0.0, 'issues': []}
        
        forces = []
        for data in self.data_history['contact_forces']:
            forces.append(np.linalg.norm(data['force']))
        
        forces = np.array(forces)
        max_force = np.max(forces) if len(forces) > 0 else 0.0
        avg_force = np.mean(forces) if len(forces) > 0 else 0.0
        
        issues = []
        if max_force > self.debug_thresholds['contact_force_spike']:
            issues.append(f'High contact force detected: {max_force:.3f}')
        
        return {
            'max_force': max_force,
            'avg_force': avg_force,
            'force_spikes': np.sum(forces > 50.0),  # Count forces > 50N as spikes
            'issues': issues
        }

    def calculate_system_health(self, results):
        """Calculate overall system health based on all analyses"""
        joint_health = 1.0 - results['joint_analysis']['instability_score']
        imu_health = results['imu_analysis']['gravity_accuracy']
        contact_health = 1.0 - min(results['contact_analysis']['max_force'] / 100.0, 1.0)  # Normalize
        
        # Weighted average (adjust weights as needed)
        system_health = 0.4 * joint_health + 0.3 * imu_health + 0.3 * contact_health
        return system_health

    def generate_debug_report(self, results):
        """Generate comprehensive debug report"""
        report = {
            'timestamp': datetime.now().isoformat(),
            'system_health': results['system_health'],
            'joint_analysis': results['joint_analysis'],
            'imu_analysis': results['imu_analysis'],
            'contact_analysis': results['contact_analysis'],
            'issues': [],
            'recommendations': []
        }
        
        # Collect all issues
        report['issues'].extend(results['joint_analysis']['issues'])
        report['issues'].extend(results['imu_analysis']['issues'])
        report['issues'].extend(results['contact_analysis']['issues'])
        
        # Generate recommendations based on issues
        if results['joint_analysis']['instability_score'] > 0.5:
            report['recommendations'].append('Check physics parameters and joint limits')
        if results['imu_analysis']['gravity_error'] > 0.5:
            report['recommendations'].append('Verify IMU sensor configuration')
        if results['contact_analysis']['max_force'] > 50.0:
            report['recommendations'].append('Investigate contact physics parameters')
        
        return report

    def publish_visualization(self):
        """Publish visualization markers for debugging"""
        marker_array = MarkerArray()
        
        # Create a marker for system health (text)
        health_marker = Marker()
        health_marker.header.frame_id = "base_link"
        health_marker.header.stamp = self.get_clock().now().to_msg()
        health_marker.ns = "debug"
        health_marker.id = 0
        health_marker.type = Marker.TEXT_VIEW_FACING
        health_marker.action = Marker.ADD
        
        # Get latest system health
        if hasattr(self, '_latest_health'):
            health_text = f"System Health: {self._latest_health:.2f}"
        else:
            health_text = "System Health: N/A"
        
        health_marker.text = health_text
        health_marker.pose.position.x = 0.0
        health_marker.pose.position.y = 0.0
        health_marker.pose.position.z = 1.0
        health_marker.scale.z = 0.1
        health_marker.color.a = 1.0
        health_marker.color.r = 1.0 if self._latest_health < 0.7 else 0.0
        health_marker.color.g = 0.0 if self._latest_health < 0.7 else 1.0
        health_marker.color.b = 0.0
        
        marker_array.markers.append(health_marker)
        self.visualization_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    framework = SimulationDebuggingFramework()
    
    try:
        rclpy.spin(framework)
    except KeyboardInterrupt:
        framework.get_logger().info('Shutting down simulation debugging framework')
    finally:
        framework.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Implementation

### Setting Up Debugging Environments

#### Debug Configuration Files

```yaml
# config/debugging_params.yaml
simulation_debugger:
  ros__parameters:
    # Physics debugging parameters
    physics:
      position_variance_threshold: 0.5
      velocity_spike_threshold: 5.0
      effort_peak_threshold: 50.0
      stability_time_window: 10.0  # seconds
    
    # Sensor debugging parameters
    sensors:
      imu_gravity_tolerance: 0.5
      lidar_range_tolerance: 0.1
      camera_brightness_threshold: 20.0
      sensor_health_check_interval: 2.0  # seconds
    
    # Control debugging parameters
    control:
      position_error_threshold: 0.1
      timing_jitter_threshold: 0.01
      stability_threshold: 0.7
      control_analysis_interval: 1.0  # seconds
    
    # Debug output parameters
    output:
      enable_detailed_logs: true
      log_level: "debug"
      enable_visualization: true
      enable_metrics_publishing: true
```

#### Launch File for Debugging

```python
# launch/simulation_debugging.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Physics debugger node
    physics_debugger = Node(
        package='humanoid_simulation',
        executable='physics_debugger',
        name='physics_debugger',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('humanoid_simulation'),
                'config',
                'debugging_params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Sensor debugger node
    sensor_debugger = Node(
        package='humanoid_simulation',
        executable='sensor_debugger',
        name='sensor_debugger',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('humanoid_simulation'),
                'config',
                'debugging_params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Control debugger node
    control_debugger = Node(
        package='humanoid_simulation',
        executable='control_debugger',
        name='control_debugger',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('humanoid_simulation'),
                'config',
                'debugging_params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Comprehensive debugging framework
    debug_framework = Node(
        package='humanoid_simulation',
        executable='simulation_debugging_framework',
        name='simulation_debugging_framework',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('humanoid_simulation'),
                'config',
                'debugging_params.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        physics_debugger,
        sensor_debugger,
        control_debugger,
        debug_framework,
    ])
```

### Debugging Workflows

#### Systematic Debugging Process

```bash
#!/bin/bash
# debug_simulation.sh - Systematic debugging workflow

echo "Starting simulation debugging workflow..."

# 1. Check basic simulation status
echo "1. Checking simulation status..."
gz stats

# 2. Verify ROS topics
echo "2. Checking ROS topics..."
ros2 topic list | grep -E "(joint|imu|scan|camera|control)"

# 3. Monitor joint states
echo "3. Monitoring joint states for 5 seconds..."
timeout 5 ros2 topic echo /joint_states --field position > /tmp/joint_states.log &

# 4. Check for NaN values in joint positions
echo "4. Checking for NaN values..."
if grep -q "nan\|inf" /tmp/joint_states.log; then
    echo "WARNING: NaN or infinity values detected in joint states!"
else
    echo "OK: No NaN values in joint states"
fi

# 5. Monitor IMU data
echo "5. Monitoring IMU data for 5 seconds..."
timeout 5 ros2 topic echo /imu/data --field linear_acceleration > /tmp/imu_data.log &

# 6. Check gravity detection
echo "6. Checking gravity detection..."
GRAVITY_MAG=$(python3 -c "
import numpy as np
import re
with open('/tmp/imu_data.log', 'r') as f:
    content = f.read()
    # Extract linear acceleration values
    import ast
    lines = content.strip().split('\n')
    accs = []
    for line in lines:
        if 'x:' in line and 'y:' in line and 'z:' in line:
            try:
                # Parse the acceleration vector
                x = float(re.search(r'x:\s*([+-]?\d*\.?\d+)', line).group(1))
                y = float(re.search(r'y:\s*([+-]?\d*\.?\d+)', line).group(1))
                z = float(re.search(r'z:\s*([+-]?\d*\.?\d+)', line).group(1))
                mag = np.sqrt(x**2 + y**2 + z**2)
                accs.append(mag)
            except:
                pass
if accs:
    avg_grav = np.mean(accs)
    print(f'{avg_grav:.3f}')
else:
    print('0.0')
")

echo "Average gravity magnitude: $GRAVITY_MAG"
if (( $(echo "$GRAVITY_MAG < 8.0 || $GRAVITY_MAG > 12.0" | bc -l) )); then
    echo "WARNING: Gravity magnitude seems incorrect!"
else
    echo "OK: Gravity magnitude is reasonable"
fi

# 7. Check control loop timing
echo "7. Checking control loop timing..."
CONTROL_FREQ=$(ros2 topic hz /joint_states 2>/dev/null | head -1 | grep -oE '[0-9.]+')
echo "Control loop frequency: $CONTROL_FREQ Hz"

if (( $(echo "$CONTROL_FREQ < 50.0" | bc -l) )); then
    echo "WARNING: Control frequency is low (< 50Hz)!"
else
    echo "OK: Control frequency is adequate"
fi

echo "Debugging workflow completed."
```

## Common Pitfalls & Debugging Tips

### Physics Simulation Issues

1. **Joint Instability**:
   - **Issue**: Joints oscillating or behaving unrealistically
   - **Debug Command**: `ros2 topic echo /joint_states --field effort`
   - **Solution**: Check joint limits, damping, and physics parameters
   - **Parameter**: Reduce `max_step_size` in physics configuration

2. **Robot Penetration**:
   - **Issue**: Robot parts passing through environment
   - **Debug Command**: `gz topic -e /gazebo/default/contacts`
   - **Solution**: Increase `min_depth` and adjust `kp` parameters
   - **Configuration**: Set `mu=1.0`, `kp=1e9`, `kd=1.0` for contacts

3. **Simulation Slowdown**:
   - **Issue**: Simulation running slower than real-time
   - **Debug Command**: `gz stats`
   - **Solution**: Simplify collision geometry, reduce update rates
   - **Optimization**: Use simpler collision shapes (boxes vs. meshes)

### Sensor Simulation Issues

1. **IMU Gravity Issues**:
   - **Issue**: IMU not detecting proper gravity
   - **Debug Command**: `ros2 topic echo /imu/data --field linear_acceleration`
   - **Expected**: Magnitude should be ~9.81 m/s² when stationary
   - **Check**: Verify IMU orientation and gravity parameter

2. **LiDAR Range Problems**:
   - **Issue**: LiDAR not detecting objects at expected ranges
   - **Debug Command**: `ros2 topic echo /scan --field ranges`
   - **Check**: Verify `range_min`, `range_max`, and `angle_` parameters
   - **Validation**: Test with known distances in simulation

3. **Camera Image Issues**:
   - **Issue**: Camera publishing blank or incorrect images
   - **Debug Command**: `ros2 run image_view image_view`
   - **Check**: Verify camera parameters and lighting in simulation
   - **Validation**: Compare with known good camera configuration

### Control System Issues

1. **Trajectory Following Errors**:
   - **Issue**: Robot not following commanded trajectories
   - **Debug Command**: `ros2 topic echo /joint_trajectory_controller/state`
   - **Analysis**: Compare `desired.positions` vs `actual.positions`
   - **Solution**: Tune controller gains and check joint limits

2. **Control Loop Timing**:
   - **Issue**: Control loop running at inconsistent rates
   - **Debug Command**: `ros2 topic hz /joint_states`
   - **Target**: 100Hz for position control, 1kHz for effort control
   - **Check**: Verify timer configurations in control nodes

3. **Effort Limiting**:
   - **Issue**: Controllers requesting impossible efforts
   - **Debug Command**: `ros2 topic echo /joint_states --field effort`
   - **Check**: Compare with `effort` limits in URDF
   - **Solution**: Implement proper effort limiting in controllers

### Debugging Tools and Commands

```bash
# Comprehensive debugging script
#!/bin/bash

echo "=== Simulation Debugging Check ==="

# Check Gazebo status
echo "Gazebo Status:"
gz stats 2>/dev/null || echo "Gazebo not running"

# Check ROS 2 status
echo -e "\nROS 2 Nodes:"
ros2 node list

echo -e "\nROS 2 Topics:"
ros2 topic list

# Check key topics
echo -e "\nChecking Joint States:"
ros2 topic echo /joint_states --field position 2>/dev/null &
TOPIC_PID=$!
sleep 2
kill $TOPIC_PID 2>/dev/null

echo -e "\nChecking IMU Data:"
ros2 topic echo /imu/data --field linear_acceleration 2>/dev/null &
TOPIC_PID=$!
sleep 2
kill $TOPIC_PID 2>/dev/null

echo -e "\nChecking TF Tree:"
ros2 run tf2_tools view_frames 2>/dev/null || echo "TF tree not available"

echo -e "\nSystem Resources:"
free -h
df -h

echo -e "\n=== Debugging Complete ==="
```

## Industry Use Cases

### Debugging in Production Simulation Systems

Major robotics companies have developed sophisticated debugging approaches:

- **Boston Dynamics**: Uses comprehensive physics validation tools to debug dynamic behaviors before real robot testing
- **Agility Robotics**: Employs real-time simulation monitoring to identify and resolve issues during development
- **NVIDIA**: Implements advanced visualization and logging for debugging Isaac Sim environments
- **Tesla**: Uses automated testing and validation frameworks for Optimus simulation debugging

### Research Applications

Academic institutions use specialized debugging techniques:

- **ETH Zurich**: Employs systematic parameter identification for debugging physics models
- **MIT CSAIL**: Uses comparative analysis between simulation and real robot data for debugging
- **University of Tokyo**: Implements detailed logging and visualization for humanoid debugging

## Summary / Key Takeaways

- Simulation debugging requires specialized tools and techniques different from real hardware debugging
- Physics, sensor, and control systems all have unique debugging challenges in simulation
- Systematic approaches with proper logging and visualization are essential
- Automated debugging frameworks can significantly improve development efficiency
- Validation against physical principles is crucial for identifying simulation issues
- Performance monitoring is important for maintaining real-time simulation

## Practice Tasks / Mini-Projects

1. Implement a custom physics debugger that monitors specific humanoid joints for instability
2. Create a sensor validation tool that compares simulated sensor data to expected values
3. Develop a control performance analyzer that identifies timing issues in control loops
4. Build a comprehensive debugging dashboard that visualizes multiple simulation metrics
5. Create automated tests that validate simulation behavior against physical constraints
6. Implement a logging system that captures simulation state for post-hoc debugging
7. Develop debugging tools specific to humanoid balance and walking behaviors
8. Create a framework for comparing simulation and real robot behavior during debugging