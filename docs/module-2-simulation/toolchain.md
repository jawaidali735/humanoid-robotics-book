---
id: module-2-simulation-toolchain
title: "Toolchain: Gazebo and Unity Setup"
sidebar_position: 3
---

# Toolchain: Gazebo and Unity Setup for Humanoid Robotics

## Introduction

This section provides a comprehensive guide to setting up and configuring simulation environments for humanoid robotics development. We'll cover the installation, configuration, and integration of Gazebo Classic, Gazebo Garden, and Unity with ROS 2. These tools form the core of the simulation toolchain that enables realistic digital twins for humanoid robot development.

## Learning Outcomes

After completing this section, you will be able to:
- Install and configure Gazebo Classic and Gazebo Garden for humanoid robotics
- Set up Unity with ROS# bridge for advanced simulation scenarios
- Integrate simulation environments with ROS 2 for seamless development workflows
- Configure physics parameters and sensor models in simulation
- Create and customize humanoid robot models for simulation

## Conceptual Foundations

### Gazebo Architecture

Gazebo is built on a plugin architecture that allows for extensibility and customization. The core components include:

- **Physics Engine**: Supports ODE, Bullet, and DART for different simulation needs
- **Sensor System**: Realistic simulation of various sensor types
- **Rendering Engine**: Visualization of the simulation environment
- **Plugin System**: Extensible architecture for custom functionality
- **ROS Integration**: Native support for ROS 1 and ROS 2 through gazebo_ros_pkgs

### Unity Robotics Toolchain

Unity provides a different approach to simulation with its powerful graphics engine and physics system:

- **PhysX Engine**: NVIDIA's physics engine for realistic physics simulation
- **ROS# Bridge**: Middleware for ROS communication
- **Visual Fidelity**: High-quality graphics for realistic sensor simulation
- **XR Support**: Virtual and augmented reality capabilities
- **Isaac Integration**: NVIDIA's robotics simulation platform

### Simulation-to-ROS Integration Patterns

The integration between simulation and ROS 2 follows specific patterns:

- **Robot Description**: URDF/SDF models that define robot geometry and properties
- **Sensor Plugins**: ROS interfaces for simulated sensors
- **Controller Plugins**: ROS interfaces for robot control
- **TF Broadcasting**: Coordinate frame management between simulation and ROS
- **Message Publishing**: Sensor data and robot state publishing to ROS topics

## Technical Deep Dive

### Gazebo Classic Installation and Setup

#### System Requirements
- Ubuntu 20.04/22.04 or equivalent Linux distribution
- ROS 2 Humble Hawksbill
- Minimum 8GB RAM, recommended 16GB+ for complex simulations
- GPU with OpenGL 3.3+ support for visualization

#### Installation Steps

```bash
# Update system packages
sudo apt update

# Install Gazebo Classic (Gazebo 11)
sudo apt install gazebo libgazebo11-dev

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev

# Verify installation
gazebo --version
```

#### Configuration Files

Create a basic Gazebo configuration in your ROS 2 workspace:

```xml
<!-- config/gazebo_config.yaml -->
gazebo:
  ros__parameters:
    # Physics parameters
    physics_engine: "ode"  # Options: ode, bullet, dart
    max_step_size: 0.001   # Physics step size (seconds)
    real_time_factor: 1.0  # Real-time update rate
    real_time_update_rate: 1000.0  # Hz
    
    # Sensor parameters
    sensor_update_rate: 100.0  # Hz
    sensor_noise: true
    
    # Rendering parameters
    enable_visualization: true
    visualization_rate: 60.0  # Hz
```

### Gazebo Garden Installation and Setup

Gazebo Garden (formerly Ignition Gazebo) provides a more modern architecture:

```bash
# Install Gazebo Garden
sudo apt install gazebo libgazebo-dev

# Install ROS 2 Gazebo garden packages
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins

# Verify installation
gz --version
```

### Unity with ROS# Setup

#### Prerequisites
- Unity Hub and Unity 2021.3 LTS or newer
- Visual Studio or Rider for scripting
- ROS 2 Humble with RMW implementation

#### Installation Process

1. **Install Unity Hub**:
   - Download from unity.com
   - Install Unity 2021.3 LTS or newer

2. **Install ROS# Package**:
   - Download ROS# from the Unity Asset Store
   - Import into your Unity project

3. **Configure ROS Communication**:
   - Set up ROS master connection
   - Configure topic and service mappings

### ROS 2 Integration Components

#### Gazebo ROS Packages

The core ROS 2 packages for Gazebo integration include:

1. **gazebo_ros_pkgs**: Core ROS-Gazebo bridge
2. **gazebo_plugins**: Pre-built plugins for common sensors and actuators
3. **gazebo_dev**: Development headers and utilities

#### Common Gazebo Plugins for Humanoid Robots

```xml
<!-- Example Gazebo plugin configuration in URDF -->
<gazebo reference="joint_name">
  <provideFeedback>true</provideFeedback>
  <joint>
    <dynamics>
      <friction>0.1</friction>
      <damping>0.01</damping>
    </dynamics>
  </joint>
</gazebo>

<!-- IMU sensor plugin -->
<gazebo reference="imu_link">
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
    </imu>
  </sensor>
</gazebo>
```

#### Sensor Simulation Configuration

For humanoid robots, common sensors include:

1. **Joint Position Sensors**: Monitor joint angles and velocities
2. **IMU Sensors**: Simulate inertial measurement units
3. **Force/Torque Sensors**: Measure contact forces at joints
4. **Camera Sensors**: Visual perception simulation
5. **LiDAR Sensors**: Range sensing for navigation

### Unity ROS# Integration

#### Basic ROS Communication Setup

```csharp
// Example Unity C# script for ROS communication
using ROS;
using UnityEngine;

public class HumanoidController : MonoBehaviour
{
    private RosSocket rosSocket;
    private string rosBridgeUrl = "ws://localhost:9090";

    void Start()
    {
        // Connect to ROS bridge
        rosSocket = new RosSocket(new RosBridgeProtocol.WebSocketNetProtocol(rosBridgeUrl));
        
        // Subscribe to joint commands
        rosSocket.Subscribe<sensor_msgs.JointState>("/joint_commands", OnJointCommand);
        
        // Publish joint states
        InvokeRepeating("PublishJointStates", 0.0f, 0.01f); // 100Hz
    }

    void OnJointCommand(sensor_msgs.JointState jointState)
    {
        // Process joint commands from ROS
        // Update Unity humanoid model accordingly
    }

    void PublishJointStates()
    {
        // Publish current joint states to ROS
        var jointState = new sensor_msgs.JointState();
        jointState.header.stamp = new std_msgs.Header();
        jointState.name = GetJointNames();
        jointState.position = GetJointPositions();
        
        rosSocket.Publish("/joint_states", jointState);
    }
}
```

## Practical Implementation

### Setting Up a Basic Humanoid Simulation

#### 1. Create a ROS 2 Package for Simulation

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python humanoid_simulation
cd humanoid_simulation
mkdir -p config launch models worlds
```

#### 2. Create Robot Description (URDF)

```xml
<!-- models/humanoid.urdf -->
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Hip joint and link -->
  <joint name="hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="hip_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="hip_link">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
    </plugin>
  </gazebo>
</robot>
```

#### 3. Create Gazebo World File

```xml
<!-- worlds/simple_humanoid.world -->
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics parameters -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Your robot model -->
    <include>
      <name>simple_humanoid</name>
      <pose>0 0 0.5 0 0 0</pose>
      <uri>model://simple_humanoid</uri>
    </include>
  </world>
</sdf>
```

#### 4. Create Launch File

```python
# launch/humanoid_simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

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
            'simple_humanoid.world'
        ]),
        description='SDF world file'
    )

    # Start Gazebo server
    gzserver = Node(
        package='gazebo_ros',
        executable='gzserver',
        arguments=[world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Start Gazebo client
    gzclient = Node(
        package='gazebo_ros',
        executable='gzclient',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # Robot state publisher
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
                'humanoid.urdf'
            ]).perform(SubstitutionContext())).read()
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world_cmd,
        gzserver,
        robot_state_publisher,
    ])
```

### Unity Setup for Humanoid Simulation

#### 1. Create Unity Project Structure

```
UnityHumanoidSim/
├── Assets/
│   ├── Scenes/
│   ├── Scripts/
│   ├── Models/
│   ├── Materials/
│   └── Plugins/
└── ProjectSettings/
```

#### 2. Configure ROS Communication

Create a configuration file for ROS connection parameters:

```json
{
  "rosBridgeUrl": "ws://localhost:9090",
  "robotName": "unity_humanoid",
  "sensorTopics": {
    "jointStates": "/unity_joint_states",
    "imu": "/unity_imu",
    "camera": "/unity_camera"
  },
  "controlTopics": {
    "jointCommands": "/unity_joint_commands"
  }
}
```

#### 3. Implement Humanoid Controller

```csharp
// Assets/Scripts/HumanoidController.cs
using ROS;
using UnityEngine;
using System.Collections.Generic;

public class HumanoidController : MonoBehaviour
{
    [Header("ROS Configuration")]
    public string rosBridgeUrl = "ws://localhost:9090";
    public string robotNamespace = "/unity_humanoid";

    [Header("Joint Configuration")]
    public List<Transform> jointTransforms = new List<Transform>();
    public List<string> jointNames = new List<string>();

    private RosSocket rosSocket;
    private bool isConnected = false;

    void Start()
    {
        ConnectToROS();
    }

    void ConnectToROS()
    {
        try
        {
            rosSocket = new RosSocket(new RosBridgeProtocol.WebSocketNetProtocol(rosBridgeUrl));
            rosSocket.OnConnected += OnConnected;
            rosSocket.OnClosed += OnDisconnected;
            
            // Subscribe to joint commands
            rosSocket.Subscribe<sensor_msgs.JointState>(
                robotNamespace + "/joint_commands", 
                OnJointCommandsReceived
            );
            
            isConnected = true;
        }
        catch (System.Exception e)
        {
            Debug.LogError("Failed to connect to ROS: " + e.Message);
        }
    }

    void OnConnected()
    {
        Debug.Log("Connected to ROS bridge");
        isConnected = true;
    }

    void OnDisconnected()
    {
        Debug.Log("Disconnected from ROS bridge");
        isConnected = false;
    }

    void OnJointCommandsReceived(sensor_msgs.JointState jointState)
    {
        if (jointState.position.Length != jointTransforms.Count)
        {
            Debug.LogWarning("Joint count mismatch between ROS and Unity");
            return;
        }

        for (int i = 0; i < jointTransforms.Count && i < jointState.position.Length; i++)
        {
            // Apply joint position to Unity transform
            // This is a simplified example - real implementation would depend on joint type
            jointTransforms[i].localRotation = Quaternion.Euler(0, 0, jointState.position[i] * Mathf.Rad2Deg);
        }
    }

    void Update()
    {
        if (isConnected)
        {
            PublishJointStates();
        }
    }

    void PublishJointStates()
    {
        var jointState = new sensor_msgs.JointState();
        jointState.header = new std_msgs.Header();
        jointState.header.stamp = new builtin_interfaces.Time();
        jointState.name = jointNames.ToArray();
        
        // Get current joint positions from Unity
        jointState.position = new double[jointTransforms.Count];
        for (int i = 0; i < jointTransforms.Count; i++)
        {
            jointState.position[i] = jointTransforms[i].localEulerAngles.z * Mathf.Deg2Rad;
        }

        rosSocket.Publish(robotNamespace + "/joint_states", jointState);
    }

    void OnDestroy()
    {
        rosSocket?.Close();
    }
}
```

## Common Pitfalls & Debugging Tips

### Gazebo-Specific Issues

1. **Performance Problems**:
   - **Issue**: Slow simulation with complex models
   - **Solution**: Reduce visual complexity, optimize collision geometry, adjust physics parameters
   - **Command**: `gz stats` to monitor simulation performance

2. **Physics Instability**:
   - **Issue**: Robot jittering or exploding in simulation
   - **Solution**: Reduce time step, adjust solver parameters, check mass/inertia values
   - **Parameter**: Set `max_step_size` to 0.001 or smaller

3. **ROS Communication Issues**:
   - **Issue**: Nodes not communicating with simulation
   - **Solution**: Check ROS_DOMAIN_ID, verify network configuration
   - **Command**: `ros2 topic list` to verify topic availability

### Unity ROS# Issues

1. **Connection Problems**:
   - **Issue**: Unity cannot connect to ROS bridge
   - **Solution**: Verify ROS bridge is running, check firewall settings
   - **Command**: `ping localhost` and verify port 9090 is accessible

2. **Message Serialization**:
   - **Issue**: Data not properly transmitted between Unity and ROS
   - **Solution**: Verify message types match between Unity and ROS nodes
   - **Debug**: Add logging to verify message content

3. **Synchronization Issues**:
   - **Issue**: Unity simulation running at different rate than ROS
   - **Solution**: Implement proper time synchronization mechanisms
   - **Approach**: Use ROS time stamps for coordination

### Debugging Strategies

1. **Simulation State Monitoring**:
   ```bash
   # Monitor simulation state
   gz topic -e /gazebo/default/world_stats
   ros2 topic echo /joint_states
   ```

2. **Performance Analysis**:
   ```bash
   # Monitor CPU and memory usage
   htop
   # Monitor Gazebo performance
   gz stats
   ```

3. **Communication Verification**:
   ```bash
   # Check ROS topics
   ros2 topic list
   ros2 topic info /joint_states
   # Echo messages to verify content
   ros2 topic echo /joint_states
   ```

## Industry Use Cases

### Research Applications

- **ETH Zurich**: Uses Gazebo for legged robot research with high-fidelity physics simulation
- **MIT CSAIL**: Employs Unity for visual perception training in humanoid robots
- **NVIDIA**: Leverages Isaac Sim for AI training and simulation-to-reality transfer

### Commercial Applications

- **Boston Dynamics**: Uses custom simulation environments for algorithm development
- **Agility Robotics**: Employs digital twins for Digit robot validation and testing
- **Unitree**: Leverages simulation for control algorithm refinement and safety validation

## Summary / Key Takeaways

- Gazebo Classic and Garden provide robust simulation environments with native ROS 2 integration
- Unity offers high-fidelity graphics and physics for advanced simulation scenarios
- Proper configuration of physics parameters is crucial for realistic simulation
- Sensor simulation must include realistic noise and latency models
- The ROS-Gazebo/Unity integration enables seamless development workflows for humanoid robotics

## Practice Tasks / Mini-Projects

1. Install Gazebo Classic and run the basic examples to verify your setup
2. Create a simple humanoid robot model in URDF and load it in Gazebo
3. Set up ROS communication between your robot model and external ROS nodes
4. Configure basic sensors (IMU, joint position) in your simulation
5. Implement a simple controller to move the robot in simulation