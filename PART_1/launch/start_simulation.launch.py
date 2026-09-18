import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    # Path to the OpenVINS config file
    openvins_config_file = os.path.join(
        os.path.expanduser('~'), 'openvins_ws', 'px4_sitl_config.yaml'
    )

    # --- 1. Launch Micro XRCE Agent ---
    # This connects ROS 2 to the PX4 flight controller.
    micro_xrce_agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # --- 2. Launch OpenVINS ---
    # This is the node for visual odometry.
    openvins_node = Node(
        package='ov_msckf',
        executable='ov_msckf_node',
        name='ov_msckf_node',
        output='screen',
        parameters=[openvins_config_file],
        remappings=[
            ('/cam0/image_raw', '/camera/image_raw'),
            ('/imu0', '/fmu/out/sensor_combined')
        ]
    )

    return LaunchDescription([
        micro_xrce_agent_process,
        openvins_node
    ])
