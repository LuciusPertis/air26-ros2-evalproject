"""demo.launch.py — the top-level AIR26 bringup.

Brings up the full graph against the M0 fake driver:
  robot_state_publisher (URDF + TF) | fake_driver (/clock, sensing, joint echo)
  | twist_mux | odometry (T2) | robot_info (T3) | the four behaviors (T4) | RViz.

Everything runs on sim time. On an engine branch, set `use_fake_driver:=false`
and launch that engine's driver instead — every other node is unchanged.

    ros2 launch air26_bringup demo.launch.py
    ros2 launch air26_bringup demo.launch.py rviz:=false        # headless
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory("air26_description")
    teleop_pkg = get_package_share_directory("air26_teleop")
    behaviors_pkg = get_package_share_directory("air26_behaviors")
    bringup_pkg = get_package_share_directory("air26_bringup")

    xacro_path = os.path.join(desc_pkg, "urdf", "air26_robot.urdf.xacro")
    rviz_path = os.path.join(desc_pkg, "rviz", "air26.rviz")
    params = os.path.join(bringup_pkg, "config", "params.yaml")

    use_fake_driver = LaunchConfiguration("use_fake_driver")
    use_rviz = LaunchConfiguration("rviz")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path]), value_type=str),
    }
    sim_time = {"use_sim_time": True}

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_driver", default_value="true",
                              description="Launch the M0 fake driver."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz."),

        # --- description / TF ---
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, sim_time],
        ),

        # --- M0 fake driver (engine branches replace this) ---
        # NOTE: the driver is the CLOCK SOURCE, so it must run on wall time
        # (use_sim_time:=false). If it ran on sim time it would wait on a clock
        # only it can publish -> deadlock. Everything else runs on sim time.
        Node(
            package="air26_fake_driver",
            executable="fake_driver",
            name="fake_driver",
            output="screen",
            parameters=[params, {"use_sim_time": False}],
            condition=IfCondition(use_fake_driver),
        ),

        # --- base command arbitration ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_pkg, "launch", "twist_mux.launch.py")),
        ),

        # --- Task 2: odometry ---
        Node(
            package="air26_odometry",
            executable="odometry_node",
            name="odometry_node",
            output="screen",
            parameters=[params],
        ),

        # --- Task 3: robot info ---
        Node(
            package="air26_robot_info",
            executable="robot_info_node",
            name="robot_info_node",
            output="screen",
            parameters=[sim_time],
        ),

        # --- Task 4: behaviors ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(behaviors_pkg, "launch", "behaviors.launch.py")),
        ),

        # --- RViz ---
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_path],
            output="screen",
            parameters=[sim_time],
            condition=IfCondition(use_rviz),
        ),
    ])
