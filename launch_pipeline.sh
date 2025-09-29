#!/bin/bash

# Pipeline Launch Script
# This script sources the required setup files and launches the complete pipeline

echo "=== Pipeline Launch Script ==="
echo "Sourcing required setup files..."

# Source the autonomous exploration development environment
echo "Sourcing autonomous_exploration_development_environment setup..."
source ~/Documents/Far_planner_test/autonomous_exploration_development_environment/install/setup.sh

# Source the far_planner setup
echo "Sourcing far_planner setup..."
source ~/Documents/Far_planner_test/far_planner/install/setup.sh

# Source the pipeline_launch setup
echo "Sourcing pipeline_launch setup..."
source ~/Documents/Far_planner_test/pipeline_launch/install/setup.sh

echo "All setup files sourced successfully!"
echo ""
echo "=== Launching Pipeline ==="
echo "Starting the complete pipeline with the following sequence:"
echo "  T=0s: fast_lio mapping starts"
echo "  T=3s: vehicle_simulator starts"  
echo "  T=6s: far_planner starts"
echo ""

# Launch the pipeline
ros2 launch pipeline_launcher pipeline.launch.py
