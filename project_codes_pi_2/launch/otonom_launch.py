"""
================================================================================
@file otonom_launch.py
@brief Complete autonomous navigation bringup launch file for Raspberry Pi 4.

Responsibilities:
- Initializes hardware bridge (micro_ros_agent), state estimation (odometry_node),
  and 2D sensor stream (rplidar_ros with static TF).
- Prioritizes manual operator override (/cmd_vel_joy) over autonomous commands
  (/cmd_vel_nav) using twist_mux.
- Serves static occupancy grid map (/map) via nav2_map_server.
- Runs AMCL probabilistic localization with resource-optimized particle limits.
- Executes minimal Nav2 navigation stack (controller, planner, recoveries, BT).
- Publishes ground-truth vs dead-reckoning trajectory paths for drift monitoring.
- Provides WebSocket connectivity via rosbridge_server for web-based dashboard.

Usage:
  ros2 launch project_codes_pi_2 otonom_launch.py \\
      map:=/home/asametpi/maps/my_map.yaml serial_port:=/dev/ttyACM0
================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess,
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
    map_arg       = DeclareLaunchArgument('map', default_value='/home/asametpi/haritam.yaml')
    serial_arg    = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    baud_arg      = DeclareLaunchArgument('baudrate', default_value='115200')
    rosbridge_arg = DeclareLaunchArgument('rosbridge', default_value='true')
    autogl_arg    = DeclareLaunchArgument('auto_global_loc', default_value='true')

    # --------------------------------------------------------------------------
    # Hardware Layer: Micro-ROS Bridge, Odometry, LiDAR, Static TF
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
    # High-priority teleoperation (/cmd_vel_joy) overrides Nav2 (/cmd_vel_nav)
    # --------------------------------------------------------------------------
    twist_mux = Node(
        package='twist_mux', executable='twist_mux',
        name='twist_mux', output='screen',
        parameters=[os.path.join(pkg, 'config', 'twist_mux_params.yaml')],
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    # --------------------------------------------------------------------------
    # Autonomous Layer: Map Server & AMCL Localization
    # --------------------------------------------------------------------------
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map'),
                     'use_sim_time': False}],
    )

    amcl = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        parameters=[os.path.join(pkg, 'config', 'nav2_params_pi.yaml')],
    )

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['map_server', 'amcl'],
                     'use_sim_time': False}],
    )

    # Minimal Nav2 Stack (Optimized for Raspberry Pi 4 2GB)
    nav2_minimal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg, 'launch', 'nav2_minimal_pi_launch.py')),
    )

    # --------------------------------------------------------------------------
    # Trajectory Diagnostics & Path Comparison
    # /odom_path : Raw dead reckoning from wheel encoders (subject to wheel slip)
    # /slam_path : Corrected trajectory in map frame via AMCL
    # --------------------------------------------------------------------------
    trajectory_pub = Node(
        package='project_codes_pi_2', executable='trajectory_publisher',
        name='trajectory_publisher', output='screen',
        parameters=[{
            'map_frame': 'map', 'odom_frame': 'odom', 'base_frame': 'base_link',
            'publish_rate': 10.0, 'min_dist': 0.02,
        }],
    )

    # Automatic Global Localization: Broadcasts particles uniformly across map
    # 12 seconds after startup to initialize robot localization without manual pose input
    auto_global_loc = TimerAction(
        period=12.0,
        condition=IfCondition(LaunchConfiguration('auto_global_loc')),
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call',
                 '/reinitialize_global_localization',
                 'std_srvs/srv/Empty', '{}'],
            output='screen')],
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
        map_arg, serial_arg, baud_arg, rosbridge_arg, autogl_arg,
        # Hardware Layer
        micro_ros_agent, odometry_node, rplidar_launch, static_tf,
        # Command Multiplexing
        twist_mux,
        # Autonomous Navigation Stack
        map_server, amcl, lifecycle_localization, nav2_minimal,
        auto_global_loc,
        trajectory_pub,
        # Web Bridge
        rosbridge, rosapi,
    ])
