#!/usr/bin/env python3
"""Behavior 2 — Manual swipe (air26_interfaces/SwipeArm service).  [WORKING]

Executes a scripted paddle swipe for the requested plane (ground/canopy) and
direction (left-to-right / right-to-left) by publishing a JointTrajectory.

Task 4 modification point: the swipe GEOMETRY (arc width, height, phase timing)
lives in arm_common.make_swipe_trajectory / the PLANE_* tables. Change the arc,
add a repeat, or add a pre-swipe "ready" step there and here.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory

from air26_interfaces.srv import SwipeArm

from air26_behaviors.arm_common import ARM_TOPIC, make_swipe_trajectory


class SwipeNode(Node):
    def __init__(self):
        super().__init__("swipe_node")
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.srv = self.create_service(SwipeArm, "/swipe_arm", self._on_swipe)
        self.get_logger().info("swipe_node up — /swipe_arm ready")

    def _on_swipe(self, request, response):
        plane = "canopy" if request.plane == SwipeArm.Request.PLANE_CANOPY else "ground"
        left_to_right = (request.direction == SwipeArm.Request.DIR_LEFT_TO_RIGHT)

        # ---- Task 4: tweak the swipe here (or in arm_common) ----
        traj = make_swipe_trajectory(plane, left_to_right)
        self.arm_pub.publish(traj)

        dir_txt = "L->R" if left_to_right else "R->L"
        msg = f"swipe: plane={plane} dir={dir_txt}"
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response


def main():
    try:
        rclpy.init()
        rclpy.spin(SwipeNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
