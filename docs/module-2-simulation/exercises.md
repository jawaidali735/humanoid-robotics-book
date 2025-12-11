---
id: module-2-simulation-exercises
title: "Exercises: Hands-on Simulation Practice"
sidebar_position: 6
---

# Exercises: Simulation Development for Humanoid Robotics

## Introduction

This section provides hands-on exercises designed to reinforce your understanding of simulation environments for humanoid robotics. These exercises build upon the concepts, toolchain, implementation examples, and case studies covered in previous sections. Each exercise is designed to provide practical experience with different aspects of humanoid robot simulation, from basic setup to advanced control and validation techniques.

## Learning Outcomes

After completing these exercises, you will be able to:
- Set up and configure simulation environments for humanoid robots
- Implement realistic sensor models and integrate them with ROS 2
- Develop controllers that work in both simulation and real environments
- Validate simulation accuracy against real robot behaviors
- Apply best practices for simulation-to-reality transfer

## Exercise 1: Basic Humanoid Simulation Setup

### Objective
Set up a basic humanoid robot simulation environment using Gazebo and ROS 2.

### Steps
1. Create a new ROS 2 package for your simulation:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python humanoid_basic_sim --dependencies rclpy std_msgs sensor_msgs geometry_msgs
   ```

2. Create a simple humanoid URDF model with at least 6 joints (3 per leg for basic walking capability):
   - Base link (torso)
   - Hip joints (left and right)
   - Knee joints (left and right) 
   - Ankle joints (left and right)

3. Add Gazebo plugins to your URDF:
   - ROS control plugin for joint control
   - IMU sensor plugin
   - Basic visual and collision properties

4. Create a launch file to start Gazebo with your robot model

5. Test that your robot appears in Gazebo and that you can see joint states in RViz

### Expected Outcome
A basic humanoid robot model loaded in Gazebo that publishes joint states and can be visualized in RViz.

### Solution Guide
```xml
<!-- models/simple_humanoid.urdf -->
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Torso Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.15"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="l_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="l_thigh"/>
    <origin xyz="0 0.15 -0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <link name="l_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="l_knee_joint" type="revolute">
    <parent link="l_thigh"/>
    <child link="l_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.1" upper="2.35" effort="100" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <link name="l_shin">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="l_ankle_joint" type="revolute">
    <parent link="l_shin"/>
    <child link="l_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="50" velocity="1.5"/>
    <dynamics damping="0.05" friction="0.01"/>
  </joint>

  <link name="l_foot">
    <visual>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Leg (mirror of left) -->
  <joint name="r_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="r_thigh"/>
    <origin xyz="0 -0.15 -0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <link name="r_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="r_knee_joint" type="revolute">
    <parent link="r_thigh"/>
    <child link="r_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.1" upper="2.35" effort="100" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <link name="r_shin">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="r_ankle_joint" type="revolute">
    <parent link="r_shin"/>
    <child link="r_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="50" velocity="1.5"/>
    <dynamics damping="0.05" friction="0.01"/>
  </joint>

  <link name="r_foot">
    <visual>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_humanoid</robotNamespace>
    </plugin>
  </gazebo>

  <!-- IMU sensor -->
  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>
</robot>
```

## Exercise 2: Sensor Integration and Simulation

### Objective
Implement realistic sensor simulation for your humanoid robot and verify proper data publication.

### Steps
1. Add a camera sensor to your humanoid robot's head in the URDF
2. Add a LiDAR sensor to your robot
3. Create a sensor data publisher node that publishes realistic sensor data
4. Verify that sensor data is being published correctly using ROS 2 tools
5. Visualize sensor data in RViz

### Expected Outcome
Your robot publishes realistic sensor data (camera images, LiDAR scans, IMU readings) that can be visualized and processed by ROS 2 nodes.

### Solution Guide
```xml
<!-- Add to your URDF file -->
<!-- Camera sensor -->
<gazebo reference="base_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>base_link</frame_name>
    </plugin>
  </sensor>
</gazebo>

<!-- LiDAR sensor -->
<gazebo reference="base_link">
  <sensor name="lidar" type="ray">
    <pose>0.15 0 0.1 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <frame_name>base_link</frame_name>
      <topic_name>scan</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

```python
#!/usr/bin/env python3
# sensors_test_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image, LaserScan
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import math

class SensorTestNode(Node):
    def __init__(self):
        super().__init__('sensor_test_node')
        
        # Publishers for sensor data
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Timer for publishing sensor data
        self.timer = self.create_timer(0.01, self.publish_sensor_data)  # 100Hz
        
        # Initialize joint state message
        self.joint_state = JointState()
        self.joint_state.name = ['l_hip_joint', 'l_knee_joint', 'l_ankle_joint', 
                                'r_hip_joint', 'r_knee_joint', 'r_ankle_joint']
        self.joint_state.position = [0.0] * 6
        self.joint_state.velocity = [0.0] * 6
        self.joint_state.effort = [0.0] * 6
        
        # Initialize IMU message
        self.imu_msg = Imu()
        self.imu_msg.orientation.x = 0.0
        self.imu_msg.orientation.y = 0.0
        self.imu_msg.orientation.z = 0.0
        self.imu_msg.orientation.w = 1.0
        
        self.get_logger().info('Sensor Test Node initialized')

    def publish_sensor_data(self):
        """Publish simulated sensor data"""
        # Update joint states with some movement
        current_time = self.get_clock().now()
        self.joint_state.header.stamp = current_time.to_msg()
        
        # Add some oscillating movement to joints
        time_sec = current_time.nanoseconds / 1e9
        for i in range(len(self.joint_state.position)):
            self.joint_state.position[i] = 0.1 * math.sin(time_sec + i * 0.5)
        
        self.joint_pub.publish(self.joint_state)
        
        # Update IMU data
        self.imu_msg.header.stamp = current_time.to_msg()
        self.imu_msg.header.frame_id = 'base_link'
        
        # Add some simulated IMU data with noise
        self.imu_msg.angular_velocity.z = 0.1 * math.sin(time_sec)
        self.imu_msg.linear_acceleration.x = 9.81 * math.sin(time_sec)
        self.imu_msg.linear_acceleration.y = 9.81 * math.cos(time_sec)
        self.imu_msg.linear_acceleration.z = 9.81  # gravity
        
        self.imu_pub.publish(self.imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor test node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 3: Basic Walking Controller

### Objective
Implement a basic walking controller for your simulated humanoid robot.

### Steps
1. Create a ROS 2 controller node that generates walking patterns
2. Implement inverse kinematics for basic leg movement
3. Test the walking controller in simulation
4. Adjust parameters to achieve stable walking

### Expected Outcome
Your humanoid robot can execute basic walking motions in simulation with coordinated leg movements.

### Solution Guide
```python
#!/usr/bin/env python3
# walking_controller.py
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')
        
        # Publisher for joint trajectories
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Timer for walking control (50Hz)
        self.walk_timer = self.create_timer(0.02, self.generate_walking_step)
        
        # Walking parameters
        self.step_phase = 0.0
        self.step_frequency = 0.5  # Hz
        self.step_amplitude = 0.2  # rad
        self.step_offset = 0.1     # rad
        
        # Joint names for our simple humanoid
        self.joint_names = [
            'l_hip_joint', 'l_knee_joint', 'l_ankle_joint',
            'r_hip_joint', 'r_knee_joint', 'r_ankle_joint'
        ]
        
        self.get_logger().info('Walking Controller initialized')

    def generate_walking_step(self):
        """Generate walking pattern based on current phase"""
        self.step_phase += 2 * math.pi * self.step_frequency * 0.02
        
        # Generate walking pattern for left leg
        left_hip = self.step_amplitude * math.sin(self.step_phase)
        left_knee = self.step_amplitude * 0.7 * math.sin(self.step_phase + math.pi/3)
        left_ankle = self.step_offset + self.step_amplitude * 0.3 * math.sin(self.step_phase - math.pi/4)
        
        # Generate walking pattern for right leg (opposite phase)
        right_hip = self.step_amplitude * math.sin(self.step_phase + math.pi)
        right_knee = self.step_amplitude * 0.7 * math.sin(self.step_phase + math.pi/3 + math.pi)
        right_ankle = self.step_offset + self.step_amplitude * 0.3 * math.sin(self.step_phase - math.pi/4 + math.pi)
        
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [
            left_hip, left_knee, left_ankle,
            right_hip, right_knee, right_ankle
        ]
        
        # Zero velocities and accelerations for simplicity
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        
        # Set timing (next 0.02 seconds)
        point.time_from_start = Duration(sec=0, nanosec=20000000)  # 0.02 seconds
        
        trajectory.points = [point]
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Publish trajectory
        self.trajectory_pub.publish(trajectory)

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

## Exercise 4: Physics Parameter Tuning

### Objective
Tune physics parameters to achieve realistic humanoid behavior in simulation.

### Steps
1. Experiment with different physics engine parameters in Gazebo
2. Adjust friction coefficients for feet to prevent slipping
3. Tune damping and spring parameters for joints
4. Validate that the robot behaves realistically

### Expected Outcome
Your simulated humanoid robot exhibits stable, realistic physical behavior with appropriate friction, damping, and mass properties.

### Solution Guide
```xml
<!-- In your world file -->
<physics name="ode" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>

<!-- In your URDF for feet contacts -->
<gazebo reference="l_foot">
  <collision>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</gazebo>

<gazebo reference="r_foot">
  <collision>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</gazebo>
```

## Exercise 5: Simulation Validation

### Objective
Implement a validation framework to compare simulation behavior with expected real-world behavior.

### Steps
1. Create a validation node that compares simulation data to expected values
2. Implement metrics for evaluating simulation accuracy
3. Generate validation reports
4. Identify and correct simulation inaccuracies

### Expected Outcome
A validation framework that can systematically evaluate the accuracy of your humanoid simulation.

### Solution Guide
```python
#!/usr/bin/env python3
# validation_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

class SimulationValidator(Node):
    def __init__(self):
        super().__init__('simulation_validator')
        
        # Subscribers for validation data
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # Publisher for validation metrics
        self.metrics_pub = self.create_publisher(
            Float64MultiArray, '/validation_metrics', 10)
        
        # Storage for validation data
        self.joint_history = []
        self.imu_history = []
        self.validation_results = []
        
        # Timer for validation analysis (1Hz)
        self.validation_timer = self.create_timer(1.0, self.perform_validation)
        
        self.get_logger().info('Simulation Validator initialized')

    def joint_callback(self, msg):
        """Store joint state data for validation"""
        self.joint_history.append({
            'timestamp': msg.header.stamp,
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort)
        })
        
        # Keep only the last 1000 samples to prevent memory issues
        if len(self.joint_history) > 1000:
            self.joint_history = self.joint_history[-1000:]

    def imu_callback(self, msg):
        """Store IMU data for validation"""
        self.imu_history.append({
            'timestamp': msg.header.stamp,
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        })
        
        # Keep only the last 1000 samples
        if len(self.imu_history) > 1000:
            self.imu_history = self.imu_history[-1000:]

    def perform_validation(self):
        """Perform validation analysis"""
        if len(self.joint_history) < 10 or len(self.imu_history) < 10:
            self.get_logger().info('Not enough data for validation')
            return

        # Calculate validation metrics
        joint_stability = self.calculate_joint_stability()
        imu_consistency = self.calculate_imu_consistency()
        
        # Create validation report
        validation_report = {
            'timestamp': datetime.now().isoformat(),
            'joint_stability': joint_stability,
            'imu_consistency': imu_consistency,
            'data_points': len(self.joint_history)
        }
        
        self.validation_results.append(validation_report)
        self.get_logger().info(f'Validation - Joint Stability: {joint_stability:.3f}, IMU Consistency: {imu_consistency:.3f}')

        # Publish metrics
        metrics_msg = Float64MultiArray()
        metrics_msg.data = [joint_stability, imu_consistency, len(self.joint_history)]
        self.metrics_pub.publish(metrics_msg)

    def calculate_joint_stability(self):
        """Calculate joint stability metric based on position variance"""
        if len(self.joint_history) < 10:
            return 1.0  # Default stable if not enough data

        # Calculate variance of joint positions over time
        positions = np.array([entry['positions'] for entry in self.joint_history])
        position_variance = np.var(positions, axis=0)
        
        # Average variance across all joints (lower is more stable)
        avg_variance = np.mean(position_variance)
        
        # Convert to stability score (0-1, higher is more stable)
        stability_score = 1.0 / (1.0 + avg_variance * 100)  # Scale factor to keep in reasonable range
        return min(stability_score, 1.0)  # Cap at 1.0

    def calculate_imu_consistency(self):
        """Calculate IMU consistency by checking if gravity is properly detected"""
        if len(self.imu_history) < 10:
            return 1.0

        # Calculate average linear acceleration
        accelerations = np.array([entry['linear_acceleration'] for entry in self.imu_history])
        avg_acceleration = np.mean(accelerations, axis=0)
        
        # Expected gravity magnitude (9.81 m/s^2)
        expected_gravity = 9.81
        actual_gravity = np.linalg.norm(avg_acceleration)
        
        # Calculate consistency (closer to 1.0 is more consistent)
        consistency = 1.0 - abs(actual_gravity - expected_gravity) / expected_gravity
        return max(consistency, 0.0)  # Clamp to 0-1 range

    def generate_validation_report(self):
        """Generate a comprehensive validation report"""
        if not self.validation_results:
            self.get_logger().warn('No validation results to report')
            return

        # Calculate average metrics
        avg_joint_stability = np.mean([r['joint_stability'] for r in self.validation_results])
        avg_imu_consistency = np.mean([r['imu_consistency'] for r in self.validation_results])
        
        report = f"""
Simulation Validation Report
============================
Generated: {datetime.now().isoformat()}

Metrics:
- Average Joint Stability: {avg_joint_stability:.3f}
- Average IMU Consistency: {avg_imu_consistency:.3f}
- Total Validation Samples: {len(self.validation_results)}

Assessment:
- Joint Stability: {'Good' if avg_joint_stability > 0.8 else 'Fair' if avg_joint_stability > 0.5 else 'Poor'}
- IMU Consistency: {'Good' if avg_imu_consistency > 0.8 else 'Fair' if avg_imu_consistency > 0.5 else 'Poor'}
        """
        
        self.get_logger().info(report)
        
        # Save to file
        filename = f"validation_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        with open(filename, 'w') as f:
            f.write(report)
        
        self.get_logger().info(f'Validation report saved to {filename}')

def main(args=None):
    rclpy.init(args=args)
    validator = SimulationValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Generating final validation report...')
        validator.generate_validation_report()
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 6: Multi-Robot Simulation

### Objective
Set up a simulation environment with multiple humanoid robots interacting.

### Steps
1. Modify your launch file to spawn multiple humanoid robots
2. Implement communication between robots using ROS 2 topics/services
3. Create a simple coordination task (e.g., formation following)
4. Test the multi-robot interaction

### Expected Outcome
Multiple humanoid robots operating simultaneously in the same simulation environment with proper coordination.

### Solution Guide
```python
#!/usr/bin/env python3
# multi_robot_coordinator.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Pose, Twist
import numpy as np

class MultiRobotCoordinator(Node):
    def __init__(self):
        super().__init__('multi_robot_coordinator')
        
        # Robot names
        self.robot_names = ['robot1', 'robot2', 'robot3']
        
        # Publishers for each robot
        self.robot_cmd_pubs = {}
        for robot_name in self.robot_names:
            self.robot_cmd_pubs[robot_name] = self.create_publisher(
                Float64MultiArray, f'/{robot_name}/joint_commands', 10)
        
        # Subscribers for robot status
        self.robot_status_subs = {}
        for robot_name in self.robot_names:
            self.robot_status_subs[robot_name] = self.create_subscription(
                Float64MultiArray, f'/{robot_name}/joint_states', 
                lambda msg, name=robot_name: self.robot_status_callback(msg, name), 10)
        
        # Timer for coordination (10Hz)
        self.coordination_timer = self.create_timer(0.1, self.perform_coordination)
        
        # Robot positions (simplified for this example)
        self.robot_positions = {name: np.array([0.0, 0.0]) for name in self.robot_names}
        
        self.get_logger().info('Multi-Robot Coordinator initialized')

    def robot_status_callback(self, msg, robot_name):
        """Update robot status"""
        # In a real implementation, this would update the robot's position
        # For this exercise, we'll just log the status
        self.get_logger().info(f'{robot_name} status: {len(msg.data)} joints')

    def perform_coordination(self):
        """Perform coordination between robots"""
        # Example: Formation control - robots maintain triangular formation
        center = np.array([0.0, 0.0])  # Center of formation
        radius = 2.0  # Distance from center
        
        for i, robot_name in enumerate(self.robot_names):
            # Calculate desired position in formation
            angle = 2 * np.pi * i / len(self.robot_names)
            desired_pos = center + radius * np.array([np.cos(angle), np.sin(angle)])
            
            # Generate commands to move toward desired position
            commands = Float64MultiArray()
            commands.data = [0.1 * np.sin(self.get_clock().now().nanoseconds / 1e9 + i)] * 6  # Simple oscillating pattern
            
            self.robot_cmd_pubs[robot_name].publish(commands)
            
            self.get_logger().info(f'{robot_name} commanded to move toward {desired_pos}')

def main(args=None):
    rclpy.init(args=args)
    coordinator = MultiRobotCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.get_logger().info('Shutting down multi-robot coordinator')
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 7: Advanced Sensor Simulation

### Objective
Implement advanced sensor simulation with realistic noise models and environmental effects.

### Steps
1. Create custom Gazebo plugins for advanced sensors
2. Implement realistic sensor noise models based on real hardware specifications
3. Add environmental effects (lighting changes, weather, etc.)
4. Validate sensor performance against real-world benchmarks

### Expected Outcome
Advanced sensor simulation with realistic behavior that closely matches real hardware performance.

### Solution Guide
```cpp
// Advanced sensor plugin example
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

namespace gazebo
{
  class AdvancedSensorPlugin : public SensorPlugin
  {
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
    {
      // Store sensor pointer
      this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
      
      if (!this->parentSensor)
      {
        gzerr << "AdvancedSensorPlugin requires a Ray sensor.\n";
        return;
      }
      
      // Initialize ROS if needed
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "advanced_sensor_plugin", ros::init_options::NoSigintHandler);
      }
      
      // Create ROS node handle
      this->rosNode.reset(new ros::NodeHandle("~"));
      
      // Initialize random number generator for noise
      this->generator.seed(ros::Time::now().toNSec());
      this->normal_dist = std::normal_distribution<double>(0.0, 0.01); // Mean 0, std dev 0.01
      
      // Connect to sensor update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          std::bind(&AdvancedSensorPlugin::OnUpdate, this));
      
      // Make sure sensor is active
      this->parentSensor->SetActive(true);
    }
    
    private: void OnUpdate()
    {
      // Get laser scan data
      auto scan = this->parentSensor->LaserShape()->GetLaserScan();
      
      // Add realistic noise to each range reading
      for (int i = 0; i < scan.count; ++i)
      {
        double original_range = scan.ranges[i];
        
        // Add Gaussian noise
        double noise = this->normal_dist(this->generator);
        
        // Apply noise with range-dependent characteristics
        double noise_factor = 1.0 + 0.001 * original_range; // More noise at longer ranges
        scan.ranges[i] = original_range + noise * noise_factor;
        
        // Ensure range is within valid bounds
        if (scan.ranges[i] < this->parentSensor->RangeMin())
          scan.ranges[i] = this->parentSensor->RangeMin();
        else if (scan.ranges[i] > this->parentSensor->RangeMax())
          scan.ranges[i] = this->parentSensor->RangeMax();
      }
    }
    
    private: sensors::RaySensorPtr parentSensor;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: event::ConnectionPtr updateConnection;
    
    // For noise generation
    private: std::default_random_engine generator;
    private: std::normal_distribution<double> normal_dist;
  };
  
  GZ_REGISTER_SENSOR_PLUGIN(AdvancedSensorPlugin)
}
```

## Common Pitfalls & Debugging Tips

### Exercise-Specific Issues

1. **Joint Limit Violations**:
   - **Issue**: Robot joints exceed physical limits during exercises
   - **Solution**: Verify URDF joint limits match your controller outputs
   - **Debug**: Use `ros2 topic echo /joint_states` to monitor joint positions

2. **Physics Instability**:
   - **Issue**: Robot exhibits unstable behavior or falls over
   - **Solution**: Check mass distribution, adjust damping parameters
   - **Check**: Verify center of mass is within support polygon

3. **Sensor Data Issues**:
   - **Issue**: Sensors publish incorrect or no data
   - **Solution**: Verify Gazebo plugin configuration and ROS topic names
   - **Verify**: Use `ros2 topic list` and `ros2 topic echo` to check topics

4. **Controller Timing**:
   - **Issue**: Controllers run at incorrect frequencies
   - **Solution**: Use appropriate timer rates for your control tasks
   - **Guideline**: Joint position control typically needs 100Hz+, high-speed control may need 1kHz+

### Debugging Commands

```bash
# Check all topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Monitor IMU data
ros2 topic echo /imu/data

# Check Gazebo topics
gz topic -l

# Monitor Gazebo world state
gz topic -e /gazebo/default/world_stats

# Visualize in RViz
ros2 run rviz2 rviz2
```

## Industry Use Cases

### Exercise Applications

These exercises mirror real-world challenges faced by humanoid robotics companies:

- **Boston Dynamics**: Uses simulation for testing dynamic behaviors before real robot testing
- **Agility Robotics**: Develops coordination algorithms for multiple robots in simulation
- **Unitree**: Validates sensor systems and control algorithms in simulated environments
- **Tesla**: Employs simulation for training AI systems on humanoid platforms

## Summary / Key Takeaways

- Start with simple simulation setups and gradually add complexity
- Always validate simulation behavior against expected physical principles
- Use appropriate physics parameters for realistic robot behavior
- Implement proper sensor simulation with realistic noise models
- Test multi-robot scenarios for coordination and communication
- Systematically validate simulation accuracy with quantitative metrics

## Practice Tasks / Mini-Projects

1. Extend your basic humanoid model with arm joints and implement reaching motions
2. Create a more complex environment with obstacles and test navigation
3. Implement a balance controller that responds to external disturbances
4. Develop a simulation scenario with multiple humanoid robots collaborating
5. Create a validation framework that compares simulation to real robot data (if available)
6. Implement domain randomization to improve controller robustness
7. Add realistic actuator dynamics to your simulation model
8. Create a scenario where the humanoid robot interacts with objects in the environment