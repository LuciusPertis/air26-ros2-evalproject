"""Standalone description viewer.

Brings up robot_state_publisher from the xacro, a joint_state_publisher_gui with
sliders (so you can pose the wheels/arm by hand), and RViz. Handy for eyeballing
a Task 1 change without launching the whole sim.

    ros2 launch air26_description display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("air26_description")
    xacro_path = os.path.join(pkg, "urdf", "air26_robot.urdf.xacro")
    rviz_path = os.path.join(pkg, "rviz", "air26.rviz")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path]), value_type=str),
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_path],
            output="screen",
        ),
    ])
