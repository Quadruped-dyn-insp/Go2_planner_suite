from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch the complete pipeline with sequential delays:
    1. fast_lio mapping_mid360.launch.py (starts immediately)
    2. vehicle_simulator system_real_robot.launch (after 3 seconds)
    3. far_planner far_planner.launch (after 6 seconds total)
    
    This version handles both .launch.py and .launch files
    """
    
    # Launch fast_lio mapping immediately
    fast_lio_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'fast_lio', 'mapping_mid360.launch.py'],
        output='screen'
    )
    
    # Launch vehicle_simulator after 3 seconds
    vehicle_simulator_launch = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'vehicle_simulator', 'system_real_robot.launch'],
                output='screen'
            )
        ]
    )
    
    # Launch far_planner after 6 seconds total (3 more after vehicle_simulator)
    far_planner_launch = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'far_planner', 'far_planner.launch'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        fast_lio_launch,
        vehicle_simulator_launch,
        far_planner_launch,
    ])
