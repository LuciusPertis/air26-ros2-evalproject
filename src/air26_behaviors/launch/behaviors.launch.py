"""Bring up all four behavior nodes (with sim time)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    common = [{"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="air26_behaviors", executable="backup_node",
             name="backup_node", output="screen", parameters=common),
        Node(package="air26_behaviors", executable="swipe_node",
             name="swipe_node", output="screen", parameters=common),
        Node(package="air26_behaviors", executable="arm_pose_node",
             name="arm_pose_node", output="screen", parameters=common),
        Node(package="air26_behaviors", executable="reactive_node",
             name="reactive_node", output="screen", parameters=common),
    ])
