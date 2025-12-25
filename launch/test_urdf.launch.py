#!/usr/bin/env python3
"""
Simple test launch file for JeTank URDF visualization.

Tests URDF loading with robot_state_publisher and visualizes in RViz.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        )
    )

    # Get configurations
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Package paths
    jetank_description_share = FindPackageShare("jetank_description")

    # URDF file path
    urdf_file = PathJoinSubstitution([
        jetank_description_share, "urdf", "jetank_ros2_control.urdf.xacro"
    ])

    # Process URDF with xacro (simulation mode, with ros2_control)
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            urdf_file, " ",
            "use_sim:=true", " ",
            "use_ros2_control:=true"
        ]),
        value_type=str
    )

    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint State Publisher GUI (for manual testing)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("jetank_description"), "rviz", "urdf.rviz"])],
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
