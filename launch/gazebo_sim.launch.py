#!/usr/bin/env python3
"""
Launch file for JeTank Gazebo simulation with ros2_control.

Launches:
1. Gazebo with empty world
2. Robot with ros2_control hardware interface
3. Controller manager with arm and gripper controllers
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo with GUI",
        )
    )

    # Get substitutions
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    # Package paths
    jetank_description_share = FindPackageShare("jetank_description")
    jetank_motor_control_share = FindPackageShare("jetank_motor_control")

    # URDF file path
    urdf_file = PathJoinSubstitution([
        jetank_description_share, "urdf", "jetank_ros2_control.urdf.xacro"
    ])

    # Process URDF with xacro (with simulation flag)
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

    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    # Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(gui),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "jetank",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Arm Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Event handlers for sequential controller loading
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            gazebo_server,
            gazebo_client,
            spawn_entity,
            delay_joint_state_broadcaster,
            delay_arm_controller,
            delay_gripper_controller,
        ]
    )
