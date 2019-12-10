#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription

# Use this to fix delays on RCLCPP_ log functions:
os.environ['RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED'] = "1"


def generate_launch_description():
    return LaunchDescription([
        Node(
            node_namespace="some_namespace",
            package='ros2-yaml-unnamed-node',
            node_executable='main',
            node_name='node_with_params',
            parameters=[os.path.join(
                get_package_share_directory('ros2-yaml-unnamed-node'),
                'param', 'test.param.yaml')],
            output='screen'),
        Node(
            package='ros2-yaml-unnamed-node',
            node_executable='main',
            node_name='another_node_with_same_params',
            parameters=[os.path.join(
                get_package_share_directory('ros2-yaml-unnamed-node'),
                'param', 'test.param.yaml')],
            output='screen'),
        Node(
            node_namespace="",
            package='ros2-yaml-unnamed-node',
            node_executable='main',
            node_name='node_without_params',
            parameters=[],
            output='screen'),
    ])
