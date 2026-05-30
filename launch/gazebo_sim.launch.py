#!/usr/bin/env python3
"""DEPRECATED — Gazebo Classic launch for JeTank.

This workspace standardised on Gazebo Fortress (Ignition) via ros_gz and
ign_ros2_control. The RoboStack pixi environment does not ship Gazebo Classic
(gzserver / gzclient / gazebo_ros), so this launch no longer functions.

Use the Ignition launch instead:
    ros2 launch jetank_simulation gazebo.launch.py           # with GUI
    ros2 launch jetank_simulation gazebo_headless.launch.py  # headless

Kept (rather than deleted) as a sign-post for existing scripts / muscle memory.
"""

from launch import LaunchDescription
from launch.actions import LogInfo


_MSG = (
    "jetank_motor_control/gazebo_sim.launch.py is DEPRECATED (Gazebo Classic). "
    "This workspace uses Gazebo Fortress — run "
    "'ros2 launch jetank_simulation gazebo.launch.py' or "
    "'ros2 launch jetank_simulation gazebo_headless.launch.py' instead."
)


def generate_launch_description():
    # Emit the deprecation notice and launch nothing classic.
    return LaunchDescription([LogInfo(msg=_MSG)])
