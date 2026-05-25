#!/bin/bash

# Trap SIGINT to kill background processes when the script is terminated
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT


# Determine the root directory of the workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

source "$WORKSPACE_DIR/workspaces/voxel_ws/install/setup.sh"

# 1. Launch Voxel Publisher Node
echo "Starting Voxel Publisher Node..."
ros2 run voxel_grid_filter voxel_node --ros-args -p voxel_size:=0.5 &
VOXEL_PID=$!

# 3. Start ROS-MQTT Bridge
echo "Starting ROS-MQTT Bridge Node..."
cd "$WORKSPACE_DIR/workspaces/mission_planner_ui/backend" || { echo "Backend directory not found"; exit 1; }
source /opt/ros/humble/setup.bash
python3 ros_mqtt_bridge.py &
MQTT_BRIDGE_PID=$!

# Wait for all processes
echo "ROS Side (MQTT Bridge) is running."
wait $VOXEL_PID $MQTT_BRIDGE_PID
