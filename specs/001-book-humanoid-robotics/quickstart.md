# Quickstart Guide: Physical AI and Humanoid Robotics Book

## Overview
This guide provides a rapid setup and initial exploration path for the Physical AI and Humanoid Robotics educational content. Follow these steps to get started with the book's content and examples.

## Prerequisites
- Ubuntu 22.04 LTS (or equivalent Linux distribution)
- Python 3.10+ installed
- Git version control system
- Node.js 18+ and npm/yarn package manager
- At least 16GB RAM (32GB recommended)
- Modern CPU with good multi-core performance
- NVIDIA RTX GPU with 8GB+ VRAM (recommended for Isaac Sim)

## Environment Setup

### 1. System Dependencies
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install basic development tools
sudo apt install -y build-essential cmake python3-dev python3-pip git curl wget

# Install ROS 2 Humble prerequisites
sudo apt install -y locales
sudo locale-gen en_US.UTF-8

# Install additional dependencies
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 2. Install ROS 2 Humble Hawksbill
```bash
# Set locale
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

# Add ROS 2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions

# Install ROS 2 development tools
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers
```

### 3. Setup ROS 2 Environment
```bash
# Source ROS 2 in your shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create a workspace for the book examples
mkdir -p ~/book_ws/src
cd ~/book_ws
colcon build --packages-select
source install/setup.bash
```

### 4. Install Docusaurus for Book Content
```bash
# Install Node.js if not already installed
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

# Install project dependencies
cd ~/book_ws
npm init -y
npm install --save-dev @docusaurus/core @docusaurus/preset-classic
```

### 5. Install Simulation Tools
```bash
# Install Gazebo Harmonic
sudo apt install -y gazebo libgazebo-dev

# Install additional simulation dependencies
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
sudo apt install -y ros-humble-joint-state-publisher ros-humble-robot-state-publisher
```

## First Steps with the Book Content

### 1. Clone the Book Repository
```bash
# Navigate to your development directory
cd ~/
git clone https://github.com/your-organization/humanoid-robotics-book.git
cd humanoid-robotics-book

# Install book dependencies
npm install
```

### 2. Start the Docusaurus Development Server
```bash
# Start the development server
npm run start

# The book will be available at http://localhost:3000
```

### 3. Run Your First ROS 2 Example
```bash
# Open a new terminal and source ROS 2
source /opt/ros/humble/setup.bash
source ~/book_ws/install/setup.bash

# Run a simple publisher-subscriber example
ros2 run demo_nodes_cpp talker
```

In another terminal:
```bash
# Source ROS 2 and run the subscriber
source /opt/ros/humble/setup.bash
source ~/book_ws/install/setup.bash

ros2 run demo_nodes_cpp listener
```

## Module 1: ROS 2 Fundamentals
After completing the setup, proceed with Module 1 content:

1. Read the ROS 2 concepts chapter
2. Complete the toolchain overview
3. Follow the implementation walkthrough
4. Try the hands-on exercises
5. Review debugging tips

## Module 2: Simulation Environments
Once comfortable with ROS 2 basics:

1. Set up Gazebo simulation environment
2. Load your first URDF robot model
3. Configure physics and sensors
4. Run basic simulation scenarios

## Module 3: AI Integration
After mastering simulation:

1. Install NVIDIA Isaac packages
2. Set up Isaac Sim (if GPU available)
3. Implement perception algorithms
4. Configure navigation systems

## Module 4: Vision-Language-Action
The capstone module combining all previous knowledge:

1. Integrate voice recognition (OpenAI Whisper)
2. Connect LLM for task planning
3. Execute actions through ROS 2
4. Complete the full integration project

## Troubleshooting Common Issues

### ROS 2 Installation Issues
- Ensure locale is properly set: `export LANG=en_US.UTF-8`
- Verify apt repository setup if packages aren't found
- Check that system clock is synchronized: `sudo ntpdate -s time.nist.gov`

### Simulation Performance
- Ensure GPU drivers are properly installed
- Check that hardware acceleration is enabled
- Consider reducing simulation complexity for lower-end hardware

### Docusaurus Build Issues
- Verify Node.js version is 18+
- Clear npm cache if encountering build errors: `npm cache clean --force`
- Check that all dependencies are properly installed

## Next Steps
1. Complete the introductory chapters
2. Set up your development environment
3. Run the first practical examples
4. Progress through modules sequentially
5. Complete the capstone project integrating all concepts