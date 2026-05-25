from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.8.160',
        description='IP address of the GO2 robot'
    )
    reconnect_delay_arg = DeclareLaunchArgument(
        'reconnect_delay',
        default_value='3.0',
        description='Seconds between reconnection attempts'
    )
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='1.0',
        description='Maximum speed scaling factor (clamps cmd_vel values)'
    )

    bridge_node = Node(
        package='go2_webrtc_bridge',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'reconnect_delay': LaunchConfiguration('reconnect_delay'),
            'max_speed': LaunchConfiguration('max_speed'),
        }],
    )

    return LaunchDescription([
        robot_ip_arg,
        reconnect_delay_arg,
        max_speed_arg,
        bridge_node,
    ])
