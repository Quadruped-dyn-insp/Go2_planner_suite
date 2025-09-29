#!/bin/bash

# Far Planner Workspace Build Script
# This script builds all workspaces in the correct order

set -e  # Exit on any error

echo "=== Far Planner Workspace Build ==="
echo "Building all workspaces..."

# Get the workspace root directory
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "Workspace root: $WORKSPACE_ROOT"

# Source ROS2 environment
source /opt/ros/humble/setup.bash
echo "✓ ROS2 environment sourced"

# Build workspaces in dependency order
WORKSPACES=(
    "autonomous_exploration"
    "far_planner"
    "fastlio2"
    "pipeline_launcher"
)

for workspace in "${WORKSPACES[@]}"; do
    workspace_path="$WORKSPACE_ROOT/workspaces/$workspace"
    
    if [ -d "$workspace_path" ]; then
        echo ""
        echo "=== Building $workspace workspace ==="
        cd "$workspace_path"
        
        # Clean build if requested
        if [ "$1" = "--clean" ] || [ "$1" = "-c" ]; then
            echo "Cleaning build artifacts..."
            rm -rf build install log
        fi
        
        # Build with release configuration
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
        
        if [ $? -eq 0 ]; then
            echo "✓ $workspace workspace built successfully"
        else
            echo "✗ Failed to build $workspace workspace"
            exit 1
        fi
        
        # Source the workspace setup
        source install/setup.bash
        echo "✓ $workspace workspace environment sourced"
    else
        echo "⚠ Warning: $workspace workspace not found at $workspace_path"
    fi
done

echo ""
echo "=== Build Complete ==="
echo "All workspaces (autonomous_exploration, far_planner, fastlio2, pipeline_launcher) built successfully!"
echo "To launch the pipeline, run: ./scripts/launch.sh"
