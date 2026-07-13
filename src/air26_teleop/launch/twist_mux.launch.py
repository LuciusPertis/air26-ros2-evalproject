"""Start twist_mux with the AIR26 arbitration config.

Muxes /teleop/cmd_vel (low pri) and /backup/cmd_vel (high pri) into /cmd_vel,
which the engine driver consumes as its base command.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("air26_teleop"), "config", "twist_mux.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[cfg, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel_out", "/cmd_vel")],
        ),
    ])
