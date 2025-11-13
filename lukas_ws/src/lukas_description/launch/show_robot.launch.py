#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'lukas_description'
    pkg_share = FindPackageShare(pkg_name)

    # Paths
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'lukas.xacro'])
    rviz_file = PathJoinSubstitution([pkg_share, 'rviz', 'model.rviz'])

    # Expand xacro -> URDF string
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        # Publishes TFs and robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # GUI sliders for joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
        )
    ])
