# Far Planner Setup Guide

This guide will help you set up the Far Planner workspace on your system.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended)
- ROS2 Humble Hawksbill
- At least 8GB RAM
- 20GB free disk space

### Dependencies
- Python 3.8+
- CMake 3.16+
- Git
- Colcon build tools

## Installation

### 1. Install ROS2 Humble

Follow the official ROS2 installation guide:
```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Clone the Repository

```bash
cd ~/Documents
git clone <repository-url> Far_planner_test
cd Far_planner_test
```

### 3. Run Setup Script

```bash
./scripts/setup.sh
```

This will:
- Install required system dependencies
- Initialize rosdep
- Install workspace-specific dependencies
- Make scripts executable

### 4. Build the Workspaces

```bash
./scripts/build.sh
```

For a clean build:
```bash
./scripts/build.sh --clean
```

### 5. Launch the System

```bash
./scripts/launch.sh
```

## Troubleshooting

### Common Issues

#### ROS2 not found
```bash
# Make sure ROS2 is properly installed and sourced
source /opt/ros/humble/setup.bash
```

#### Build failures
```bash
# Clean build and try again
./scripts/build.sh --clean
```

#### Permission errors
```bash
# Make sure scripts are executable
chmod +x scripts/*.sh
```

### Getting Help

1. Check the main README.md for general information
2. Review the CHANGELOG.md for recent changes
3. Look at individual workspace README files in `workspaces/*/`
4. Check ROS2 logs for detailed error information

## Development

### Directory Structure
```
workspaces/
├── autonomous_exploration/  # Mapping and exploration
├── far_planner/            # Path planning algorithms
└── pipeline_launcher/      # System coordination
```

### Building Individual Workspaces
```bash
cd workspaces/autonomous_exploration
colcon build --symlink-install
source install/setup.bash
```

### Adding New Packages
1. Navigate to the appropriate workspace
2. Create your package in the `src/` directory
3. Rebuild the workspace
4. Update dependencies as needed
