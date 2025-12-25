---
id: configuration
title: "Environment Configuration"
sidebar_position: 3
---

# Environment Configuration

This guide covers the configuration management for different deployment targets in the Physical AI and Humanoid Robotics book project.

## Development Environment

### Prerequisites
- Ubuntu 22.04 LTS (or equivalent Linux distribution)
- Python 3.10+ installed
- Node.js 18+ and npm/yarn package manager
- Git version control system
- At least 16GB RAM (32GB recommended)
- Modern CPU with good multi-core performance
- NVIDIA RTX GPU with 8GB+ VRAM (recommended for Isaac Sim)

### ROS 2 Setup
```bash
# Install ROS 2 Humble Hawksbill
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions

# Source ROS 2 in your shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Docusaurus Setup
```bash
# Install project dependencies
npm install
```

## Configuration Files

### Environment Variables
Create `.env` files for different environments:

#### Development (.env.development)
```
NODE_ENV=development
DOKU_ENV=development
API_BASE_URL=http://localhost:3000/api
```

#### Production (.env.production)
```
NODE_ENV=production
DOKU_ENV=production
API_BASE_URL=https://api.physical-ai-book.com/v1
```

## Deployment Targets

### Local Development
- Run with `npm run start`
- Auto-reload on file changes
- Includes development tools and debugging features

### GitHub Pages
- Build with `npm run build`
- Serve with `npm run serve`
- Optimized for production use

## Build Configuration

### Docusaurus Configuration
The `docusaurus.config.js` file contains all site configuration including:
- Site metadata (title, tagline, favicon)
- Navigation structure
- Theme configuration
- Prism syntax highlighting settings

### Sidebar Configuration
The `sidebars.js` file defines the documentation structure and navigation hierarchy.

## Testing Configuration

### Local Testing
- Unit tests for code examples
- Integration tests for documentation functionality
- Build verification tests

### CI/CD Configuration
- Automated build and deployment
- Link validation
- Content quality checks