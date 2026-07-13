#!/usr/bin/env python3
"""air26_robot_info — a service that reports facts about the robot.  (TASK 3)

WHAT SHIPS (working): a node that correctly receives the LATCHED /robot_description
and answers a DescribeRobot service with a minimal report (it echoes a parameter
and confirms the description arrived). It prints the report to its terminal too.

WHAT'S MISSING (your task): real URDF introspection. describe_robot() has a
# TODO(student): parse the stored URDF and report structural facts — joint/link
counts, a named joint's limits, a named link's mass, etc. Extend both this node
AND, if you need a richer request, the DescribeRobot.srv in air26_interfaces.

THE TRAP (already handled for you): /robot_description is published once, latched,
with TRANSIENT_LOCAL durability. A default (VOLATILE) subscriber connects after
the publisher and silently never gets the message. The subscriber below is set to
TRANSIENT_LOCAL so your callback actually fires — don't lose an hour to this.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String

from air26_interfaces.srv import DescribeRobot


class RobotInfoNode(Node):
    def __init__(self):
        super().__init__("robot_info_node")

        # a hardcoded parameter, used by the minimal working example below
        self.declare_parameter("robot_nickname", "AIR26 Clearing Rover")

        self.urdf_xml = None  # filled in by the latched-topic callback

        # --- the QoS that makes the latched /robot_description arrive ---
        latched_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # <-- the important bit
        )
        self.create_subscription(
            String, "/robot_description", self._on_description, latched_qos)

        self.srv = self.create_service(
            DescribeRobot, "/describe_robot", self._on_describe)

        self.get_logger().info(
            "robot_info_node up — minimal example only; extend describe_robot() for Task 3")

    def _on_description(self, msg: String):
        self.urdf_xml = msg.data
        self.get_logger().info(
            f"received /robot_description ({len(self.urdf_xml)} chars)")

    def _on_describe(self, request, response):
        report = self.describe_robot(request.query)
        response.success = True
        response.report = report
        # the node also prints the report to its own terminal (per the spec)
        self.get_logger().info("describe_robot ->\n" + report)
        return response

    # ==================================================================
    # TODO(student): TASK 3 — turn this into real URDF introspection.
    # ==================================================================
    def describe_robot(self, query: str) -> str:
        """Build the human-readable report for a DescribeRobot request.

        MINIMAL WORKING EXAMPLE (ships): echoes the nickname parameter and says
        whether the URDF has been received. This already returns without crashing.

        YOUR TASK: parse self.urdf_xml and report real structure. Ideas:
          * "" (empty query) -> a summary: number of links, number of joints.
          * a joint name      -> its type and limits (lower/upper/effort/velocity).
          * a link name       -> its mass (and maybe its inertia).
        Parsing options: `xml.etree.ElementTree` (already available), or
        `urdf_parser_py.urdf.URDF.from_xml_string(self.urdf_xml)` if installed.
        Nice touch: make it report on whatever you added in Task 1.
        """
        nickname = self.get_parameter("robot_nickname").value

        if self.urdf_xml is None:
            return (f"[{nickname}] robot_description not received yet "
                    "(is robot_state_publisher running?)")

        # --- minimal example. Replace/extend the block below for Task 3. ---
        lines = [
            f"nickname: {nickname}",
            f"robot_description: received ({len(self.urdf_xml)} chars)",
            f"query: {query!r}",
            "NOTE: full URDF introspection is TODO(student) — see describe_robot().",
        ]
        return "\n".join(lines)


def main():
    try:
        rclpy.init()
        rclpy.spin(RobotInfoNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
