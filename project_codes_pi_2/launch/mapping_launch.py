"""
================================================================================
@file mapping_launch.py
@brief Complete onboard 2D SLAM mapping launch description for Raspberry Pi.

Responsibilities:
- Launches hardware layer: micro_ros_agent, odometry_node, rplidar_ros, and static laser TF.
- Manages command multiplexing via twist_mux for manual teleoperation override.
- Executes slam_toolbox in online async mode to generate the occupancy grid map (/map).
- Starts nav2_map_server's map_saver_server to support remote map persistence from the Web UI.
- Launches trajectory_publisher for continuous path tracking.
- Launches rosbridge_server and rosapi for browser-based dashboard connectivity.

Usage:
  ros2 launch project_codes_pi_2 mapping_launch.py serial_port:=/dev/ttyACM0
================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource, AnyLaunchDescriptionSource,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('project_codes_pi_2')

    # --------------------------------------------------------------------------
    # Launch Arguments
    # --------------------------------------------------------------------------
    serial_arg    = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    baud_arg      = DeclareLaunchArgument('baudrate', default_value='115200')
    rosbridge_arg = DeclareLaunchArgument('rosbridge', default_value='true')

    # --------------------------------------------------------------------------
    # Hardware Layer: micro-ROS Bridge, Odometry, LiDAR, Static TF
    # --------------------------------------------------------------------------
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
             'serial', '--dev', LaunchConfiguration('serial_port'),
             '-b', LaunchConfiguration('baudrate')],
        output='screen', name='micro_ros_agent',
    )

    odometry_node = Node(
        package='project_codes_pi_2', executable='odometry_node',
        name='odometry_node', output='screen',
        parameters=[{
            'track_width': 0.30, 'wheel_diameter': 0.07,
            'counts_per_rev': 3200.0, 'publish_rate': 50.0,
        }],
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch', 'rplidar.launch.py')),
        launch_arguments={'serial_port': '/dev/rplidar'}.items(),
    )

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.10',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    # --------------------------------------------------------------------------
    # Command Multiplexing: twist_mux
    # --------------------------------------------------------------------------
    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        name='twist_mux', output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'twist_mux_params.yaml'),
            {'use_stamped': False},
        ],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # --------------------------------------------------------------------------
    # Mapping Layer: SLAM Toolbox (Online Asynchronous Mode)
    # --------------------------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[{
            'odom_frame': 'odom', 'map_frame': 'map', 'base_frame': 'base_link',
            'scan_topic': '/scan', 'mode': 'mapping',
            'resolution': 0.05, 'max_laser_range': 12.0,
            'minimum_travel_distance': 0.15, 'minimum_travel_heading': 0.15,
            'do_loop_closing': True, 'use_sim_time': False,
            'map_update_interval': 1.0,
        }],
    )

    lifecycle_slam = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_slam', output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['slam_toolbox'], 'use_sim_time': False}],
    )

    # --------------------------------------------------------------------------
    # Map Persistence Server: map_saver_server (Called via Web UI Service)
    # --------------------------------------------------------------------------
    map_saver_server = Node(
        package='nav2_map_server', executable='map_saver_server',
        name='map_saver_server', output='screen',
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'use_sim_time': False,
        }],
    )

    lifecycle_map_saver = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map_saver', output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['map_saver_server'], 'use_sim_time': False}],
    )

    # --------------------------------------------------------------------------
    # Trajectory Diagnostics
    # --------------------------------------------------------------------------
    trajectory_pub = Node(
        package='project_codes_pi_2', executable='trajectory_publisher',
        name='trajectory_publisher', output='screen',
        parameters=[{
            'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
            'publish_rate': 10.0, 'min_dist': 0.02,
        }],
    )

    # --------------------------------------------------------------------------
    # Web Bridge: rosbridge_server & rosapi (Port 9090)
    # --------------------------------------------------------------------------
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')),
        condition=IfCondition(LaunchConfiguration('rosbridge')),
        launch_arguments={'port': '9090'}.items(),
    )

    rosapi = Node(
        condition=IfCondition(LaunchConfiguration('rosbridge')),
        package='rosapi', executable='rosapi_node',
        name='rosapi', output='screen',
    )

    return LaunchDescription([
        serial_arg, baud_arg, rosbridge_arg,
        # Hardware Layer
        micro_ros_agent, odometry_node, rplidar_launch, static_tf,
        # Command Multiplexing
        twist_mux,
        # Mapping & Persistence
        slam_node, lifecycle_slam,
        map_saver_server, lifecycle_map_saver,
        trajectory_pub,
        # Web Bridge
        rosbridge, rosapi,
    ])
