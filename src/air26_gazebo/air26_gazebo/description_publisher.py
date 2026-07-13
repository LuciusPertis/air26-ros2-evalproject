#!/usr/bin/env python3
"""description_publisher — the Gazebo-side robot description holder.

Why this exists (and why the SHARED robot_state_publisher is left untouched):

  * The shared RSP keeps publishing the PRISTINE /robot_description (the shared
    air26_description URDF, byte-identical to the MuJoCo branch) so T3/RViz/TF and
    the frozen interface are unchanged.
  * Gazebo, however, needs the ENGINE description — the same robot PLUS the
    <ros2_control> + <gazebo> plugin/sensor tags. gazebo_ros2_control also passes
    that description to its controller_manager as a CLI parameter override, and
    rcl's YAML parser trips over XML comments (non-ASCII, backticks, `-->`, ...).

So the launch file expands the engine xacro, STRIPS the comments, and hands the
result to this node as the `robot_description` parameter. This node:
  * exposes that param (gazebo_ros2_control reads it via <robot_param_node>), and
  * latches it on `gz_robot_description` so `spawn_entity.py -topic ...` can load
    the model into Gazebo.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class DescriptionPublisher(Node):
    def __init__(self):
        super().__init__("gz_description")
        # the comment-stripped engine URDF, injected by the launch file
        self.declare_parameter("robot_description", "")
        urdf = self.get_parameter("robot_description").value

        if not urdf:
            self.get_logger().error("robot_description param is empty")
        # latched (transient_local) so a late-joining spawn_entity still gets it
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, "gz_robot_description", qos)
        self.pub.publish(String(data=urdf))
        self.get_logger().info(
            f"holding engine description ({len(urdf)} chars) on "
            "gz_robot_description + param robot_description")


def main():
    rclpy.init()
    try:
        rclpy.spin(DescriptionPublisher())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
