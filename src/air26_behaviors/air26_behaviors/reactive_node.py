#!/usr/bin/env python3
"""Behavior 4 — Reactive swipe.  [WORKING]

Watches the front / canopy range sensors and automatically calls the /swipe_arm
service when a light obstacle appears. Gated by a SetReactiveMode toggle
(DEFAULT OFF) and a per-swipe cooldown so it doesn't chatter while an obstacle
sits in range.

Task 4 modification point (the spec calls this out specifically): the TRIGGER
CONDITION in _evaluate() — which sensors, what threshold, ground vs canopy
routing. Ideas: add a second reactive zone (e.g. the side sensors), require two
sensors to agree, or pick swipe direction from which side is closer.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Range

from air26_interfaces.srv import SetReactiveMode, SwipeArm

FRONT_SENSORS = ["front_center", "front_left", "front_right"]
CANOPY_SENSORS = ["canopy_left", "canopy_right"]


class ReactiveNode(Node):
    def __init__(self):
        super().__init__("reactive_node")

        self.declare_parameter("trigger_distance_m", 0.45)  # fire when closer than this
        self.declare_parameter("cooldown_s", 6.0)           # min gap between swipes
        self.trigger_distance = self.get_parameter("trigger_distance_m").value
        self.cooldown = self.get_parameter("cooldown_s").value

        self.enabled = False              # DEFAULT OFF (per spec)
        self.ranges = {}                  # sensor name -> latest range (m)
        self.last_swipe_time = None       # rclpy.time.Time of last triggered swipe

        best_effort = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        for name in FRONT_SENSORS + CANOPY_SENSORS:
            self.create_subscription(
                Range, f"/range/{name}",
                lambda msg, n=name: self.ranges.__setitem__(n, msg.range),
                best_effort)

        self.mode_srv = self.create_service(
            SetReactiveMode, "/set_reactive_mode", self._on_set_mode)
        self.swipe_client = self.create_client(SwipeArm, "/swipe_arm")

        # evaluate the trigger at a steady rate
        self.timer = self.create_timer(0.2, self._evaluate)
        self.get_logger().info(
            "reactive_node up — /set_reactive_mode ready (reactive is OFF by default)")

    def _on_set_mode(self, request, response):
        self.enabled = bool(request.enable)
        response.success = True
        response.message = f"reactive mode {'ENABLED' if self.enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def _in_cooldown(self) -> bool:
        if self.last_swipe_time is None:
            return False
        dt = (self.get_clock().now() - self.last_swipe_time).nanoseconds * 1e-9
        return dt < self.cooldown

    # ================= Task 4 modification point: TRIGGER =================
    def _evaluate(self):
        if not self.enabled or self._in_cooldown():
            return

        front_min = min((self.ranges.get(s, float("inf")) for s in FRONT_SENSORS),
                        default=float("inf"))
        canopy_min = min((self.ranges.get(s, float("inf")) for s in CANOPY_SENSORS),
                         default=float("inf"))

        if front_min < self.trigger_distance:
            self._fire(SwipeArm.Request.PLANE_GROUND, f"front obstacle @ {front_min:.2f}m")
        elif canopy_min < self.trigger_distance:
            self._fire(SwipeArm.Request.PLANE_CANOPY, f"canopy obstacle @ {canopy_min:.2f}m")
    # =====================================================================

    def _fire(self, plane, reason: str):
        if not self.swipe_client.service_is_ready():
            self.get_logger().warn("swipe service not available yet; skipping trigger")
            return
        self.last_swipe_time = self.get_clock().now()
        req = SwipeArm.Request()
        req.plane = plane
        req.direction = SwipeArm.Request.DIR_LEFT_TO_RIGHT
        self.swipe_client.call_async(req)
        self.get_logger().info(f"reactive swipe triggered: {reason}")


def main():
    try:
        rclpy.init()
        rclpy.spin(ReactiveNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
