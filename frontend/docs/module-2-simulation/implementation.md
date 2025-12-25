---
id: module-2-simulation-implementation
title: "Implementation: Practical Simulation Examples"
sidebar_position: 4
---

# Implementation: Practical Simulation Examples for Humanoid Robotics

## Introduction

This section provides hands-on implementation examples of simulation environments specifically tailored for humanoid robotics. We'll walk through creating realistic simulation scenarios, implementing sensor systems, and developing control interfaces that bridge the gap between simulation and real-world deployment. The examples build upon the theoretical concepts and toolchain setup covered in previous sections.

## Learning Outcomes

After completing this section, you will be able to:
- Implement complete simulation environments for humanoid robots
- Create realistic sensor models and integrate them with ROS 2
- Develop control interfaces that work seamlessly in simulation and reality
- Implement physics-based humanoid robot models with accurate dynamics
- Create custom simulation scenarios for specific humanoid tasks

## Conceptual Foundations

### Simulation Implementation Patterns

Effective simulation implementation for humanoid robots follows several key patterns:

1. **Model-Based Design**: Create accurate URDF/SDF representations of physical robots
2. **Sensor Integration**: Implement realistic sensor models that match hardware specifications
3. **Control Interface Consistency**: Ensure simulation and real robot use identical interfaces
4. **Physics Calibration**: Tune simulation parameters to match real-world behavior
5. **Validation Frameworks**: Implement methods to verify simulation accuracy

### Simulation Architecture for Humanoid Robots

A well-designed simulation architecture includes:

- **Robot Models**: Accurate URDF descriptions with proper mass, inertia, and joint properties
- **Environment Models**: Realistic world representations with appropriate physics properties
- **Sensor Systems**: Simulated sensors that publish standard ROS message types
- **Control Systems**: Interfaces that match real robot controllers
- **Validation Tools**: Methods to compare simulation and real-world behavior

### Reality Gap Management

Managing the simulation-to-reality transfer requires:
- Accurate modeling of robot dynamics and environmental interactions
- Realistic sensor noise and latency modeling
- Proper friction and contact physics modeling
- Validation against real robot performance

## Technical Deep Dive

### Creating a Complete Humanoid Robot Model

Let's implement a complete humanoid robot model for simulation with proper physical properties:

```xml
<!-- models/advanced_humanoid.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="advanced_humanoid">

  <!-- Include gazebo plugins -->
  <xacro:include filename="$(find advanced_humanoid_description)/urdf/materials.xacro"/>
  <xacro:include filename="$(find advanced_humanoid_description)/urdf/transmission.xacro"/>
  <xacro:include filename="$(find advanced_humanoid_description)/urdf/gazebo.xacro"/>

  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_mass" value="10.0" />
  <xacro:property name="hip_mass" value="2.0" />
  <xacro:property name="thigh_mass" value="1.5" />
  <xacro:property name="shin_mass" value="1.2" />
  <xacro:property name="foot_mass" value="0.8" />

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="${base_mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.15"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.4"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Hip joint and link -->
  <joint name="base_to_hip" type="fixed">
    <parent link="base_link"/>
    <child link="hip_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="hip_link">
    <inertial>
      <mass value="${hip_mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.025"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left leg - hip pitch -->
  <joint name="l_hip_pitch" type="revolute">
    <parent link="hip_link"/>
    <child link="l_thigh"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-45 * M_PI / 180}" upper="${80 * M_PI / 180}" effort="100" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <link name="l_thigh">
    <inertial>
      <mass value="${thigh_mass}"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Left leg - knee -->
  <joint name="l_knee" type="revolute">
    <parent link="l_thigh"/>
    <child link="l_shin"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-10 * M_PI / 180}" upper="${140 * M_PI / 180}" effort="100" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <link name="l_shin">
    <inertial>
      <mass value="${shin_mass}"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.003"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Left leg - ankle -->
  <joint name="l_ankle" type="revolute">
    <parent link="l_shin"/>
    <child link="l_foot"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-30 * M_PI / 180}" upper="${30 * M_PI / 180}" effort="50" velocity="1.5"/>
    <dynamics damping="0.05" friction="0.01"/>
  </joint>

  <link name="l_foot">
    <inertial>
      <mass value="${foot_mass}"/>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.003"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Right leg (mirrored) -->
  <joint name="r_hip_pitch" type="revolute">
    <parent link="hip_link"/>
    <child link="r_thigh"/>
    <origin xyz="0 -0.1 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-45 * M_PI / 180}" upper="${80 * M_PI / 180}" effort="100" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <link name="r_thigh">
    <inertial>
      <mass value="${thigh_mass}"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="r_knee" type="revolute">
    <parent link="r_thigh"/>
    <child link="r_shin"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-10 * M_PI / 180}" upper="${140 * M_PI / 180}" effort="100" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <link name="r_shin">
    <inertial>
      <mass value="${shin_mass}"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.003"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="r_ankle" type="revolute">
    <parent link="r_shin"/>
    <child link="r_foot"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-30 * M_PI / 180}" upper="${30 * M_PI / 180}" effort="50" velocity="1.5"/>
    <dynamics damping="0.05" friction="0.01"/>
  </joint>

  <link name="r_foot">
    <inertial>
      <mass value="${foot_mass}"/>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.003"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/advanced_humanoid</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- IMU sensor -->
  <gazebo reference="hip_link">
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

### Implementing Realistic Sensor Models

Now let's create a sensor configuration that includes realistic noise models for humanoid applications:

```xml
<!-- config/sensors.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Camera sensor -->
  <xacro:macro name="camera_sensor" params="name parent_link *origin">
    <gazebo reference="${parent_link}">
      <sensor name="${name}_camera" type="camera">
        <update_rate>30</update_rate>
        <camera name="head">
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
        <plugin name="${name}_camera_controller" filename="libgazebo_ros_camera.so">
          <frame_name>${parent_link}</frame_name>
          <min_depth>0.1</min_depth>
          <max_depth>100</max_depth>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>

  <!-- LiDAR sensor -->
  <xacro:macro name="lidar_sensor" params="name parent_link *origin">
    <gazebo reference="${parent_link}">
      <sensor name="${name}_lidar" type="ray">
        <pose>0 0 0 0 0 0</pose>
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
        <plugin name="${name}_lidar_controller" filename="libgazebo_ros_laser.so">
          <frame_name>${parent_link}</frame_name>
          <topic_name>${name}/scan</topic_name>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>

  <!-- Force/Torque sensor -->
  <xacro:macro name="ft_sensor" params="name parent_link child_link joint_name">
    <gazebo>
      <plugin name="${name}_ft_sensor" filename="libgazebo_ros_ft_sensor.so">
        <update_rate>100</update_rate>
        <joint_name>${joint_name}</joint_name>
        <frame_name>${child_link}</frame_name>
        <topic_name>${name}/wrench</topic_name>
      </plugin>
    </gazebo>
  </xacro:macro>

</robot>
```

### Creating a Complete Simulation Environment

Let's implement a world file that represents a realistic humanoid testing environment:

```xml
<!-- worlds/humanoid_lab.world -->
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_lab">
    
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Physics configuration -->
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
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

    <!-- Lab floor with appropriate friction -->
    <model name="lab_floor">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="floor_link">
        <collision name="floor_collision">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles for navigation testing -->
    <model name="obstacle_1">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="obstacle_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Ramp for walking tests -->
    <model name="ramp">
      <pose>-3 0 0 0 0.2 0</pose>
      <static>true</static>
      <link name="ramp_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add the humanoid robot -->
    <include>
      <name>advanced_humanoid</name>
      <pose>0 0 1 0 0 0</pose>
      <uri>model://advanced_humanoid</uri>
    </include>

  </world>
</sdf>
```

### Implementing Control Systems for Simulation

Now let's create a controller that works both in simulation and on real hardware:

```python
#!/usr/bin/env python3
# controllers/humanoid_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
import numpy as np
import math

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        
        # Joint names for the humanoid robot
        self.joint_names = [
            'l_hip_pitch', 'l_knee', 'l_ankle',
            'r_hip_pitch', 'r_knee', 'r_ankle'
        ]
        
        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        self.controller_state_sub = self.create_subscription(
            JointTrajectoryControllerState, '/joint_trajectory_controller/state', 
            self.controller_state_callback, 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz
        
        # Internal state
        self.current_joint_states = None
        self.target_positions = {name: 0.0 for name in self.joint_names}
        self.control_mode = 'stand'  # 'stand', 'walk', 'balance'
        
        self.get_logger().info('Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Handle joint state messages from simulation"""
        if self.current_joint_states is None:
            self.current_joint_states = {}
        
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

    def controller_state_callback(self, msg):
        """Handle controller state feedback"""
        # Process controller state for advanced control logic
        pass

    def control_loop(self):
        """Main control loop for the humanoid robot"""
        if self.current_joint_states is None:
            return
        
        if self.control_mode == 'stand':
            self.execute_stand_control()
        elif self.control_mode == 'walk':
            self.execute_walk_control()
        elif self.control_mode == 'balance':
            self.execute_balance_control()

    def execute_stand_control(self):
        """Control the humanoid to maintain a standing position"""
        # Calculate target positions for standing posture
        stand_positions = {
            'l_hip_pitch': 0.0,
            'l_knee': 0.0,
            'l_ankle': 0.0,
            'r_hip_pitch': 0.0,
            'r_knee': 0.0,
            'r_ankle': 0.0
        }
        
        self.publish_trajectory(stand_positions, duration=1.0)

    def execute_walk_control(self):
        """Implement a simple walking pattern"""
        # This is a simplified walking controller
        # In practice, this would use inverse kinematics and gait planning
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Generate walking pattern based on time
        phase = (current_time * 2.0) % (2 * math.pi)  # 2 Hz walking
        
        walk_positions = {
            'l_hip_pitch': 0.2 * math.sin(phase),
            'l_knee': 0.1 * math.sin(phase + math.pi/2),
            'l_ankle': 0.1 * math.sin(phase),
            'r_hip_pitch': 0.2 * math.sin(phase + math.pi),
            'r_knee': 0.1 * math.sin(phase + math.pi/2 + math.pi),
            'r_ankle': 0.1 * math.sin(phase + math.pi)
        }
        
        self.publish_trajectory(walk_positions, duration=0.05)

    def execute_balance_control(self):
        """Implement balance control based on IMU feedback"""
        # This would typically use IMU data for balance control
        # For simulation, we'll implement a simple PD controller
        if 'imu_data' in self.current_joint_states:
            # Use IMU data to adjust joint positions for balance
            pass

    def publish_trajectory(self, positions, duration=0.1):
        """Publish joint trajectory to the controller"""
        trajectory = JointTrajectory()
        trajectory.joint_names = list(positions.keys())
        
        point = JointTrajectoryPoint()
        point.positions = list(positions.values())
        point.velocities = [0.0] * len(positions)  # Zero velocity for simplicity
        point.accelerations = [0.0] * len(positions)  # Zero acceleration for simplicity
        point.time_from_start = Duration(sec=0, nanosec=int(duration * 1e9))
        
        trajectory.points = [point]
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        self.trajectory_pub.publish(trajectory)

    def set_control_mode(self, mode):
        """Set the control mode for the humanoid"""
        if mode in ['stand', 'walk', 'balance']:
            self.control_mode = mode
            self.get_logger().info(f'Switched to {mode} mode')
        else:
            self.get_logger().warn(f'Invalid control mode: {mode}')


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

### Implementing a Gazebo Plugin for Advanced Physics

For more advanced simulation scenarios, we can create a custom Gazebo plugin:

```cpp
// src/advanced_humanoid_gazebo_plugin.cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo
{
  class AdvancedHumanoidPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the model pointer for later usage
      this->model = _parent;
      
      // Get the links for the humanoid
      this->hip_link = this->model->GetLink("hip_link");
      this->l_foot_link = this->model->GetLink("l_foot");
      this->r_foot_link = this->model->GetLink("r_foot");
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&AdvancedHumanoidPlugin::OnUpdate, this));
          
      // Initialize ROS if not already initialized
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "advanced_humanoid_gazebo_plugin",
                 ros::init_options::NoSigintHandler);
      }
      
      // Create ROS node handle
      this->rosNode.reset(new ros::NodeHandle("~"));
      
      // Publisher for IMU data
      this->imu_pub = this->rosNode->advertise<sensor_msgs::Imu>("/humanoid/imu", 10);
      
      // Publisher for foot force data
      this->l_force_pub = this->rosNode->advertise<geometry_msgs::WrenchStamped>("/humanoid/l_foot_force", 10);
      this->r_force_pub = this->rosNode->advertise<geometry_msgs::WrenchStamped>("/humanoid/r_foot_force", 10);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Get current time and model state
      common::Time current_time = this->model->GetWorld()->SimTime();
      
      // Publish IMU data
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp.sec = current_time.sec;
      imu_msg.header.stamp.nanosec = current_time.nsec;
      imu_msg.header.frame_id = "imu_link";
      
      // Get orientation from model
      ignition::math::Pose3d pose = this->hip_link->WorldPose();
      imu_msg.orientation.x = pose.Rot().X();
      imu_msg.orientation.y = pose.Rot().Y();
      imu_msg.orientation.z = pose.Rot().Z();
      imu_msg.orientation.w = pose.Rot().W();
      
      // Get angular velocity
      ignition::math::Vector3d angular_vel = this->hip_link->WorldAngularVel();
      imu_msg.angular_velocity.x = angular_vel.X();
      imu_msg.angular_velocity.y = angular_vel.Y();
      imu_msg.angular_velocity.z = angular_vel.Z();
      
      // Get linear acceleration (with gravity compensation)
      ignition::math::Vector3d linear_acc = this->hip_link->WorldLinearAccel();
      ignition::math::Vector3d gravity(0, 0, -9.81);
      linear_acc = linear_acc - gravity;
      
      imu_msg.linear_acceleration.x = linear_acc.X();
      imu_msg.linear_acceleration.y = linear_acc.Y();
      imu_msg.linear_acceleration.z = linear_acc.Z();
      
      // Publish the IMU message
      this->imu_pub.publish(imu_msg);
      
      // Publish foot force data
      this->publishFootForces();
    }
    
    private: void publishFootForces()
    {
      // Get contact forces for feet
      // This is a simplified approach - in practice, you'd need to process
      // contact information from the physics engine
      
      geometry_msgs::WrenchStamped l_force_msg, r_force_msg;
      l_force_msg.header.stamp = ros::Time::now();
      l_force_msg.header.frame_id = "l_foot";
      
      r_force_msg.header.stamp = ros::Time::now();
      r_force_msg.header.frame_id = "r_foot";
      
      // For now, we'll just publish zero forces
      // In a real implementation, you'd extract contact forces from Gazebo
      l_force_msg.wrench.force.x = 0.0;
      l_force_msg.wrench.force.y = 0.0;
      l_force_msg.wrench.force.z = 0.0;
      l_force_msg.wrench.torque.x = 0.0;
      l_force_msg.wrench.torque.y = 0.0;
      l_force_msg.wrench.torque.z = 0.0;
      
      r_force_msg.wrench.force.x = 0.0;
      r_force_msg.wrench.force.y = 0.0;
      r_force_msg.wrench.force.z = 0.0;
      r_force_msg.wrench.torque.x = 0.0;
      r_force_msg.wrench.torque.y = 0.0;
      r_force_msg.wrench.torque.z = 0.0;
      
      this->l_force_pub.publish(l_force_msg);
      this->r_force_pub.publish(r_force_msg);
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    
    // Pointers to the links
    private: physics::LinkPtr hip_link;
    private: physics::LinkPtr l_foot_link;
    private: physics::LinkPtr r_foot_link;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // ROS node and publishers
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Publisher imu_pub;
    private: ros::Publisher l_force_pub;
    private: ros::Publisher r_force_pub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AdvancedHumanoidPlugin)
}
```

## Practical Implementation

### Setting Up the Complete Simulation System

Now let's create a launch file that brings up the complete simulation system:

```python
# launch/complete_humanoid_simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('humanoid_simulation'),
            'worlds',
            'humanoid_lab.world'
        ]),
        description='SDF world file'
    )
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='advanced_humanoid',
        description='Name of the robot'
    )

    # Start Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
            'verbose': 'false'
        }.items()
    )

    # Start Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(PathJoinSubstitution([
                FindPackageShare('humanoid_simulation'),
                'models',
                'advanced_humanoid.urdf.xacro'
            ]).perform({})).read()
        }]
    )

    # Joint State Publisher (for GUI control)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # Load and start controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('humanoid_simulation'),
                'config',
                'controllers.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Humanoid controller
    humanoid_controller = Node(
        package='humanoid_simulation',
        executable='humanoid_controller',
        name='humanoid_controller',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz for visualization (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('humanoid_simulation'),
            'rviz',
            'humanoid_simulation.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world_cmd,
        declare_robot_name_cmd,
        gzserver,
        gzclient,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        controller_manager,
        humanoid_controller,
        # rviz,  # Uncomment if you want RViz
    ])
```

### Creating Controller Configuration

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
      - l_hip_pitch
      - l_knee
      - l_ankle
      - r_hip_pitch
      - r_knee
      - r_ankle

    interface_name: position
    state_interface_names: ["position", "velocity"]
    command_interface_names: ["position"]
    
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    
    allow_partial_joints_goal: false
    open_loop_control: true
    
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.5
```

### Implementing a Walking Pattern Generator

```python
#!/usr/bin/env python3
# controllers/walking_pattern_generator.py

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math

class WalkingPatternGenerator(Node):
    def __init__(self):
        super().__init__('walking_pattern_generator')
        
        # Publisher for walking trajectories
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Timer for walking pattern generation
        self.walk_timer = self.create_timer(0.1, self.generate_walking_pattern)
        
        # Walking parameters
        self.step_phase = 0.0
        self.step_frequency = 0.5  # Hz
        self.step_amplitude = 0.1  # rad
        self.step_height = 0.05   # m
        
        self.get_logger().info('Walking Pattern Generator initialized')

    def generate_walking_pattern(self):
        """Generate walking pattern based on current phase"""
        self.step_phase += 2 * math.pi * self.step_frequency * 0.1
        
        # Generate walking pattern using simple sinusoidal approximation
        # This is a simplified model - real walking would use inverse kinematics
        left_hip = self.step_amplitude * math.sin(self.step_phase)
        left_knee = self.step_amplitude * 0.5 * math.sin(self.step_phase + math.pi/2)
        left_ankle = self.step_amplitude * 0.3 * math.sin(self.step_phase - math.pi/4)
        
        right_hip = self.step_amplitude * math.sin(self.step_phase + math.pi)
        right_knee = self.step_amplitude * 0.5 * math.sin(self.step_phase + math.pi/2 + math.pi)
        right_ankle = self.step_amplitude * 0.3 * math.sin(self.step_phase - math.pi/4 + math.pi)
        
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'l_hip_pitch', 'l_knee', 'l_ankle',
            'r_hip_pitch', 'r_knee', 'r_ankle'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [
            left_hip, left_knee, left_ankle,
            right_hip, right_knee, right_ankle
        ]
        
        # Zero velocities and accelerations for simplicity
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        
        # Set timing
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 seconds
        
        trajectory.points = [point]
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Publish trajectory
        self.trajectory_pub.publish(trajectory)

    def start_walking(self):
        """Start the walking pattern generation"""
        self.walk_timer.reset()
        self.get_logger().info('Started walking pattern generation')

    def stop_walking(self):
        """Stop the walking pattern generation"""
        self.walk_timer.cancel()
        self.get_logger().info('Stopped walking pattern generation')


def main(args=None):
    rclpy.init(args=args)
    walker = WalkingPatternGenerator()
    
    try:
        # Start walking immediately
        walker.start_walking()
        rclpy.spin(walker)
    except KeyboardInterrupt:
        walker.get_logger().info('Shutting down walking pattern generator')
    finally:
        walker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Common Pitfalls & Debugging Tips

### Physics Simulation Issues

1. **Robot Instability**:
   - **Issue**: Humanoid robot falls over or exhibits unstable behavior
   - **Solution**: Check mass distribution, adjust center of mass, verify joint limits
   - **Debug Command**: `ros2 topic echo /joint_states` to monitor joint positions

2. **Joint Limit Violations**:
   - **Issue**: Joints exceed physical limits during simulation
   - **Solution**: Verify URDF joint limits match real hardware, adjust controller gains
   - **Debug Command**: `gz topic -e /gazebo/default/advanced_humanoid/joint_states`

3. **Contact Physics Problems**:
   - **Issue**: Robot feet slip or don't make proper contact with ground
   - **Solution**: Increase friction coefficients, adjust contact parameters
   - **Configuration**: Set mu=1.0, mu2=1.0 in surface friction models

### Sensor Simulation Issues

1. **IMU Drift**:
   - **Issue**: Simulated IMU shows drift similar to real sensors
   - **Solution**: Implement bias models and calibration procedures
   - **Implementation**: Add bias terms that change over time

2. **Sensor Noise Mismatch**:
   - **Issue**: Simulated sensor noise doesn't match real hardware
   - **Solution**: Calibrate noise parameters based on real sensor data
   - **Process**: Collect real sensor data and match statistical properties

3. **Timing Issues**:
   - **Issue**: Sensor data published at incorrect rates
   - **Solution**: Verify update rates in sensor configuration files
   - **Check**: Ensure Gazebo update rate matches desired sensor rate

### Control System Issues

1. **Control Loop Timing**:
   - **Issue**: Control loop runs at inconsistent rates
   - **Solution**: Use real-time scheduling, monitor loop timing
   - **Monitoring**: Add timing diagnostics to control nodes

2. **Trajectory Following**:
   - **Issue**: Robot doesn't follow commanded trajectories
   - **Solution**: Adjust controller gains, verify joint limits
   - **Debugging**: Compare commanded vs. actual joint positions

3. **Communication Delays**:
   - **Issue**: Delays in ROS communication affect control performance
   - **Solution**: Optimize network configuration, use appropriate QoS settings
   - **Configuration**: Use reliable QoS for critical control messages

## Industry Use Cases

### Research Applications

- **ETH Zurich**: Uses simulation for dynamic walking control research with precise physics modeling
- **MIT CSAIL**: Employs simulation for humanoid manipulation tasks with realistic contact physics
- **Carnegie Mellon University**: Leverages simulation for humanoid learning and adaptation

### Commercial Applications

- **Agility Robotics**: Uses simulation for Digit robot control system development
- **Unitree Robotics**: Employs simulation for G1 and H1 humanoid robot validation
- **Boston Dynamics**: Utilizes custom simulation for algorithm development and testing

## Summary / Key Takeaways

- Complete humanoid simulation requires accurate models with proper mass, inertia, and joint properties
- Realistic sensor simulation with appropriate noise models is crucial for effective training
- Control systems should be designed to work identically in simulation and reality
- Physics parameters must be carefully tuned to match real-world behavior
- Validation against real hardware is essential to ensure simulation accuracy

## Practice Tasks / Mini-Projects

1. Implement a complete humanoid model with realistic dynamics and sensor models
2. Create a walking controller that can maintain balance in simulation
3. Develop a simulation environment with obstacles for navigation testing
4. Implement a sensor fusion system that combines multiple simulated sensors
5. Create a validation framework to compare simulation and real robot behavior