# Pipeline Launcher

This ROS2 Humble package launches a complete pipeline with the following sequence:

1. **fast_lio** - `mapping_mid360.launch.py` (starts immediately)
2. **vehicle_simulator** - `system_real_robot.launch` (after 3 seconds)
3. **far_planner** - `far_planner.launch` (after 6 seconds total)

## Build Instructions

From your workspace root (`/home/yasiru/Documents/Far_planner_test/pipeline_launch`):

```bash
# Build the package
colcon build --packages-select pipeline_launcher

# Source the workspace
source install/setup.bash
```

## Usage

Launch the complete pipeline:

```bash
ros2 launch pipeline_launcher pipeline.launch.py
```

Or use the alternative version if you encounter issues with launch file formats:

```bash
ros2 launch pipeline_launcher pipeline_alternative.launch.py
```

## Launch Sequence

- **T=0s**: fast_lio mapping starts
- **T=3s**: vehicle_simulator starts
- **T=6s**: far_planner starts

Each component has a 3-second delay from the previous one to ensure proper initialization.

## Dependencies

Make sure you have the following packages installed in your ROS2 workspace:
- `fast_lio`
- `vehicle_simulator` 
- `far_planner`

## Troubleshooting

If you encounter issues with launch file formats, try using `pipeline_alternative.launch.py` which uses `ExecuteProcess` instead of `IncludeLaunchDescription`.
