#!/bin/bash

# Far Planner Workspace Setup Script
# This script sets up the development environment for the Far Planner workspace

set -e  # Exit on any error

echo "=== Far Planner Workspace Setup ==="
echo "Setting up development environment..."

# Get the workspace root directory
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "Workspace root: $WORKSPACE_ROOT"

# Check ROS2 installation
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 not found. Please install ROS2 Humble first."
    echo "Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "✓ ROS2 found"

# Source ROS2 environment
source /opt/ros/humble/setup.bash
echo "✓ ROS2 environment sourced"

# Install dependencies
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git

echo "✓ System dependencies installed"

# Initialize rosdep if needed
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi

rosdep update
echo "✓ rosdep updated"

# Install workspace dependencies
echo "Installing workspace dependencies..."
cd "$WORKSPACE_ROOT"

for workspace in workspaces/*/; do
    if [ -d "$workspace" ]; then
        echo "Installing dependencies for $(basename "$workspace")..."
        cd "$workspace"
        if [ -f "src/package.xml" ] || find src -name "package.xml" | grep -q .; then
            rosdep install --from-paths src --ignore-src -r -y
        fi
        cd "$WORKSPACE_ROOT"
    fi
done

echo "✓ Workspace dependencies installed"

# Make scripts executable
chmod +x "$WORKSPACE_ROOT/scripts/"*.sh
echo "✓ Scripts made executable"

echo ""
echo "=== Setup Complete ==="
echo "To build the workspaces, run: ./scripts/build.sh"
echo "To launch the pipeline, run: ./scripts/launch.sh"
