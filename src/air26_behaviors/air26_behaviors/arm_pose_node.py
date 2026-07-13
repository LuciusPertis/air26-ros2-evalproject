#!/usr/bin/env python3
"""Behavior 3 — Stow / Ready (air26_interfaces/SetArmPose service).  [WORKING]

Moves the arm to one of two named joint configurations by publishing a
single-point JointTrajectory.

Task 4 modification point: the two poses (POSE_STOW, POSE_READY) live in
arm_common — retune them, or add a third named pose (extend SetArmPose.srv in
air26_interfaces to carry the new constant).
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory

from air26_interfaces.srv import SetArmPose

from air26_behaviors.arm_common import (
    ARM_TOPIC, POSE_READY, POSE_STOW, make_pose_trajectory)


class ArmPoseNode(Node):
    def __init__(self):
        super().__init__("arm_pose_node")
        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.srv = self.create_service(SetArmPose, "/set_arm_pose", self._on_set_pose)
        self.get_logger().info("arm_pose_node up — /set_arm_pose ready")

    def _on_set_pose(self, request, response):
        if request.pose == SetArmPose.Request.POSE_STOW:
            name, positions = "stow", POSE_STOW
        elif request.pose == SetArmPose.Request.POSE_READY:
            name, positions = "ready", POSE_READY
        else:
            response.success = False
            response.message = f"unknown pose id {request.pose}"
            self.get_logger().warn(response.message)
            return response

        # ---- Task 4: retune poses here (or in arm_common) ----
        self.arm_pub.publish(make_pose_trajectory(positions))
        response.success = True
        response.message = f"arm -> {name}"
        self.get_logger().info(response.message)
        return response


def main():
    try:
        rclpy.init()
        rclpy.spin(ArmPoseNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
