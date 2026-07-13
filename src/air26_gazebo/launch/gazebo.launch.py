"""gazebo.launch.py — the full AIR26 demo on the Gazebo Classic engine (M2).

Reuses the shared air26_bringup demo (description, mux, odometry, robot_info,
behaviors, RViz) with the fake driver disabled, and brings up Gazebo Classic +
gazebo_ros2_control in its place. The ROS interface is identical to M0/M1 — the
shared robot_state_publisher still publishes the pristine /robot_description.

  ros2 launch air26_gazebo gazebo.launch.py
  ros2 launch air26_gazebo gazebo.launch.py gui:=false        # headless gzserver
  ros2 launch air26_gazebo gazebo.launch.py rviz:=false       # no RViz

Description flow (see description_publisher.py for the why):
  shared RSP        -> /robot_description        (pristine, for TF/RViz/T3)
  gz_description     -> gz_robot_description      (engine URDF, comment-stripped)
                        + param robot_description (read by gazebo_ros2_control)
  spawn_entity      -> loads gz_robot_description into Gazebo

Command routing:
  twist_mux -> /cmd_vel --(relay)--> /ackermann_steering_controller/reference_unstamped
  behaviors -> /arm_controller/joint_trajectory -> joint_trajectory_controller
Gazebo is the /clock source; every ROS node runs on sim time.
"""

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _engine_urdf() -> str:
    """Expand the engine xacro and strip XML comments.

    gazebo_ros2_control forwards this URDF to its controller_manager as an rcl
    parameter override; rcl parses the value as YAML and trips over comment text
    (non-ASCII, backticks, `-->`). Stripping comments keeps it clean.
    """
    gazebo_pkg = get_package_share_directory("air26_gazebo")
    xacro_path = os.path.join(gazebo_pkg, "urdf", "air26_gazebo.urdf.xacro")
    doc = xacro.process_file(xacro_path)
    urdf = doc.toxml()
    urdf = re.sub(r"<!--.*?-->", "", urdf, flags=re.DOTALL)
    return urdf


def generate_launch_description():
    gazebo_pkg = get_package_share_directory("air26_gazebo")
    bringup_pkg = get_package_share_directory("air26_bringup")
    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")

    world_path = os.path.join(gazebo_pkg, "worlds", "air26.world")
    engine_urdf = _engine_urdf()

    use_rviz = LaunchConfiguration("rviz")
    use_gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")

    # --- Gazebo Classic: gzserver (init+factory -> /clock + spawn) and gzclient ---
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world, "verbose": "true"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gzclient.launch.py")),
        condition=IfCondition(use_gui),
    )

    # --- shared graph (fake driver OFF); RSP publishes the PRISTINE description ---
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "demo.launch.py")),
        launch_arguments={
            "use_fake_driver": "false",
            "rviz": use_rviz,
        }.items(),
    )

    # --- engine description holder (feeds gazebo_ros2_control + spawn) ---
    gz_description = Node(
        package="air26_gazebo", executable="description_publisher",
        name="gz_description", output="screen",
        parameters=[{"robot_description": engine_urdf, "use_sim_time": True}],
    )

    # --- spawn the rover from the engine description ---
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", name="spawn_air26",
        arguments=["-topic", "gz_robot_description", "-entity", "air26",
                   "-z", "0.06"],
        output="screen",
    )

    # --- controllers (loaded once gazebo_ros2_control's controller_manager is up,
    #     i.e. after the robot is spawned) ---
    jsb = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    ackermann = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["ackermann_steering_controller", "-c", "/controller_manager"],
    )
    arm = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # --- /cmd_vel (mux out) -> the ackermann controller's Twist reference ---
    cmd_relay = Node(
        package="air26_gazebo", executable="cmd_vel_relay", name="cmd_vel_relay",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Launch the Gazebo client GUI."),
        DeclareLaunchArgument("world", default_value=world_path),

        gzserver,
        gzclient,
        demo,
        gz_description,
        spawn,
        cmd_relay,

        # chain the spawners so controllers load in dependency order
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb, on_exit=[ackermann, arm])),
    ])
