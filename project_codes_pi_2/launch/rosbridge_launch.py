"""
================================================================================
@file rosbridge_launch.py
@brief WebSocket bridge launch file for remote web dashboard connectivity.

Responsibilities:
- Launches rosbridge_websocket_launch on default port 9090.
- Launches rosapi_node for querying ROS 2 services, topics, and parameters.

Usage:
  ros2 launch project_codes_pi_2 rosbridge_launch.py
================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --------------------------------------------------------------------------
    # WebSocket Server Node (rosbridge_server on Port 9090)
    # --------------------------------------------------------------------------
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml'
            )
        ),
        launch_arguments={'port': '9090'}.items(),
    )

    # --------------------------------------------------------------------------
    # ROS API Helper Node (Provides service and topic inspection)
    # --------------------------------------------------------------------------
    rosapi = Node(
        package='rosapi', executable='rosapi_node',
        name='rosapi', output='screen',
    )

    return LaunchDescription([rosbridge, rosapi])
