#!/usr/bin/env python3
"""Behavior 1 — Backup-to-clearing (air26_interfaces/BackupToClearing action). [WORKING]

Reverses the rover in a STRAIGHT line (steering centered — no maneuvering) while
watching the rear range sensor. Stops when the rear min-range exceeds
`clearance_threshold_m` or `timeout_s` trips. Cancelable; streams
current_clearance_m + elapsed_s as feedback.

Command routing: publishes Twist on /backup/cmd_vel, which twist_mux gives HIGH
priority while the action is active, outranking teleop, then releases on stop.

Task 4 modification point: the STOP CONDITION and motion profile in execute (see
the marked block) — e.g. add a short settle pause once clear, change the reverse
speed ramp, or require the clearance to hold for N cycles before declaring success.
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

from air26_interfaces.action import BackupToClearing

REVERSE_SPEED = 0.15  # m/s (magnitude; sign is applied as reverse)


class BackupNode(Node):
    def __init__(self):
        super().__init__("backup_node")
        self.cb_group = ReentrantCallbackGroup()

        self.rear_range = float("inf")  # latest rear clearance (m)

        best_effort = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Range, "/range/rear", self._on_rear, best_effort,
            callback_group=self.cb_group)

        self.cmd_pub = self.create_publisher(Twist, "/backup/cmd_vel", 10)

        self.action_server = ActionServer(
            self, BackupToClearing, "/backup_to_clearing",
            execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cb_group)

        self.get_logger().info("backup_node up — /backup_to_clearing ready")

    def _on_rear(self, msg: Range):
        self.rear_range = msg.range

    def _publish_cmd(self, vx: float):
        t = Twist()
        t.linear.x = vx
        t.angular.z = 0.0  # straight reverse only — steering centered
        self.cmd_pub.publish(t)

    def _execute(self, goal_handle):
        goal = goal_handle.request
        threshold = goal.clearance_threshold_m
        timeout = goal.timeout_s
        self.get_logger().info(
            f"backup: threshold={threshold:.2f}m timeout={timeout:.1f}s")

        start = self.get_clock().now()
        rate = self.create_rate(20.0)  # 20 Hz control loop (respects sim time)
        result = BackupToClearing.Result()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
            clearance = self.rear_range

            # ---- cancel ----
            if goal_handle.is_cancel_requested:
                self._publish_cmd(0.0)
                goal_handle.canceled()
                result.cleared = False
                result.timed_out = False
                result.final_clearance_m = float(clearance)
                result.elapsed_s = float(elapsed)
                self.get_logger().info("backup canceled")
                return result

            # ======== Task 4 modification point: STOP CONDITION ========
            cleared = clearance > threshold
            timed_out = elapsed >= timeout
            if cleared or timed_out:
                self._publish_cmd(0.0)
                goal_handle.succeed()
                result.cleared = bool(cleared)
                result.timed_out = bool(timed_out and not cleared)
                result.final_clearance_m = float(clearance)
                result.elapsed_s = float(elapsed)
                self.get_logger().info(
                    f"backup done: cleared={result.cleared} "
                    f"timed_out={result.timed_out} clearance={clearance:.2f}m")
                return result
            # ===========================================================

            # keep reversing straight
            self._publish_cmd(-REVERSE_SPEED)

            fb = BackupToClearing.Feedback()
            fb.current_clearance_m = float(clearance)
            fb.elapsed_s = float(elapsed)
            goal_handle.publish_feedback(fb)

            rate.sleep()

        # rclpy shutting down
        self._publish_cmd(0.0)
        goal_handle.abort()
        return result


def main():
    try:
        rclpy.init()
        node = BackupNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
