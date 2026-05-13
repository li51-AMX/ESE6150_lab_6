#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory and load parameters
    package_dir = get_package_share_directory('lab7_pkg')
    params_file = os.path.join(package_dir, 'config', 'rrt_params.yaml')

    # Create the RRT node with parameters
    rrt_node = Node(
        package='lab7_pkg',
        executable='rrt_node',
        name='rrt_node',
        parameters=[params_file],
        output='screen'
    )

    # Create the pure pursuit RRT node
    pure_pursuit_rrt = Node(
        package='lab7_pkg',
        executable='pure_pursuit_rrt.py',
        name='pure_pursuit_rrt',
        output='screen'
    )

    # Create the debug node
    debug_node = Node(
        package='lab7_pkg',
        executable='debug_node.py',
        name='debug_node',
        output='screen'
    )

    return LaunchDescription([
        rrt_node,
        pure_pursuit_rrt,
        debug_node
    ])
