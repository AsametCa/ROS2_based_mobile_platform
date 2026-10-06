"""
================================================================================
@file nav2_minimal_pi_launch.py
@brief Lightweight Nav2 navigation stack launch file tailored for Raspberry Pi 4 (2 GB RAM).

Responsibilities:
- Starts essential navigation nodes: controller_server, planner_server, behavior_server, bt_navigator.
- Bypasses high-overhead auxiliary nodes (collision monitor, dockers, smoothers).
- Remaps output velocity commands to /cmd_vel_nav for twist_mux arbitration.
- Manages node lifecycles using nav2_lifecycle_manager.

Usage:
  Included internally by otonom_launch.py
================================================================================
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('project_codes_pi_2')
    params = os.path.join(pkg, 'config', 'nav2_params_pi.yaml')

    # --------------------------------------------------------------------------
    # Controller Server (DWB Local Planner -> /cmd_vel_nav)
    # --------------------------------------------------------------------------
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[params],
        remappings=[('/cmd_vel', '/cmd_vel_nav')],
    )

    # --------------------------------------------------------------------------
    # Planner Server (NavFn Global Path Planner)
    # --------------------------------------------------------------------------
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[params],
    )

    # --------------------------------------------------------------------------
    # Behavior Server (Recovery Behaviors: Spin, BackUp, Wait)
    # --------------------------------------------------------------------------
    behaviors = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[params],
    )

    # --------------------------------------------------------------------------
    # Behavior Tree Navigator
    # --------------------------------------------------------------------------
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[params],
    )

    # --------------------------------------------------------------------------
    # Lifecycle Manager for Navigation Nodes
    # --------------------------------------------------------------------------
    lifecycle_nav = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        controller, planner, behaviors, bt_navigator, lifecycle_nav,
    ])
