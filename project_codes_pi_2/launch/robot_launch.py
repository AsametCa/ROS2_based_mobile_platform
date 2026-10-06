"""
================================================================================
@file robot_launch.py
@brief Hardware bringup launch file for the differential drive mobile platform.

Responsibilities:
- Establishes serial communication with ESP32 microcontroller via micro_ros_agent.
- Spawns odometry_node to calculate dead reckoning from motor pulse feedback.
- Launches RPLiDAR A1 laser scanner driver on /scan.
- Broadcasts static coordinate transform between base_link and laser frames.

Usage:
  ros2 launch project_codes_pi_2 robot_launch.py serial_port:=/dev/ttyACM0
================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --------------------------------------------------------------------------
    # Launch Arguments
    # --------------------------------------------------------------------------
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    baudrate_arg    = DeclareLaunchArgument('baudrate', default_value='115200')

    # --------------------------------------------------------------------------
    # Hardware Layer: micro-ROS Agent (ESP32 Serial Bridge)
    # --------------------------------------------------------------------------
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
             'serial', '--dev', LaunchConfiguration('serial_port'),
             '-b', LaunchConfiguration('baudrate')],
        output='screen', name='micro_ros_agent',
    )

    # --------------------------------------------------------------------------
    # State Estimation: Odometry Node (/left,right_motor_vel -> /odom + TF)
    # --------------------------------------------------------------------------
    odometry_node = Node(
        package='project_codes_pi_2', executable='odometry_node',
        name='odometry_node', output='screen',
        parameters=[{
            'track_width': 0.30, 'wheel_diameter': 0.07,
            'counts_per_rev': 3200.0, 'publish_rate': 50.0,
        }],
    )

    # --------------------------------------------------------------------------
    # Perception: RPLiDAR A1 2D Laser Scanner (/scan)
    # --------------------------------------------------------------------------
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch', 'rplidar.launch.py')),
        launch_arguments={'serial_port': '/dev/rplidar'}.items(),
    )

    # --------------------------------------------------------------------------
    # Coordinate Transforms: Static Transform (base_link -> laser)
    # --------------------------------------------------------------------------
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.10',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    return LaunchDescription([
        serial_port_arg, baudrate_arg,
        micro_ros_agent, odometry_node,
        rplidar_launch, static_tf,
    ])
