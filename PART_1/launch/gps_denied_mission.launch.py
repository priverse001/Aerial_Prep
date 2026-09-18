import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Micro XRCE-DDS Agent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # Vision Bridge & Health Monitor Node
    vision_bridge_node = Node(
        package='gps_denied_navigation',
        executable='vision_bridge_health_node',
        name='vision_bridge_health_node',
        output='screen',
        parameters=[{
            'target_rate_hz': 30.0,
            'min_features_healthy': 25,
            'critical_features_threshold': 10,
            'base_pos_variance': 0.01,
            'base_vel_variance': 0.02,
            'base_att_variance': 0.005,
        }]
    )

    # Autonomous Offboard Position Hold Node
    offboard_hold_node = Node(
        package='gps_denied_navigation',
        executable='offboard_position_hold_node',
        name='offboard_position_hold_node',
        output='screen',
        parameters=[{
            'target_altitude_m': 10.0,
            'hold_duration_sec': 90.0,
            'max_allowed_radius_m': 1.5,
            'auto_arm': True,
            'failsafe_test_enabled': True
        }]
    )

    return LaunchDescription([
        micro_xrce_agent,
        vision_bridge_node,
        offboard_hold_node
    ])
