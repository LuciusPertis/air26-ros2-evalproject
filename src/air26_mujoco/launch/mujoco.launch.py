"""mujoco.launch.py — the full AIR26 demo on the MuJoCo engine.

Reuses the shared air26_bringup demo (description, mux, odometry, robot_info,
behaviors, RViz) with the fake driver disabled, and starts the real MuJoCo
driver in its place. The ROS interface is identical to M0.

    ros2 launch air26_mujoco mujoco.launch.py
    ros2 launch air26_mujoco mujoco.launch.py viewer:=true      # MuJoCo GUI
    ros2 launch air26_mujoco mujoco.launch.py rviz:=false       # headless
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory("air26_bringup")
    mujoco_pkg = get_package_share_directory("air26_mujoco")

    model_path = os.path.join(mujoco_pkg, "mjcf", "air26_rover.xml")
    params = os.path.join(bringup_pkg, "config", "params.yaml")

    viewer = LaunchConfiguration("viewer")
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="false",
                              description="Launch the MuJoCo passive viewer (needs a display)."),
        DeclareLaunchArgument("rviz", default_value="true"),

        # shared graph, fake driver OFF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, "launch", "demo.launch.py")),
            launch_arguments={
                "use_fake_driver": "false",
                "rviz": use_rviz,
            }.items(),
        ),

        # the real MuJoCo driver (clock source -> wall time)
        Node(
            package="air26_mujoco",
            executable="mujoco_driver",
            name="mujoco_driver",
            output="screen",
            parameters=[params, {
                "use_sim_time": False,
                "model_path": model_path,
                "viewer": viewer,
            }],
        ),
    ])
