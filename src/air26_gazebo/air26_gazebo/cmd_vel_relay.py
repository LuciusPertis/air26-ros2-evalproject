#!/usr/bin/env python3
"""cmd_vel_relay — bridge the shared /cmd_vel to the ackermann controller input.

twist_mux publishes the arbitrated base command as geometry_msgs/Twist on the
frozen `/cmd_vel` topic. ackermann_steering_controller (with use_stamped_vel:=false)
listens for a Twist on its own `~/reference_unstamped` topic. This node just
forwards one to the other, so the shared command-routing graph is untouched and no
external topic_tools dependency is needed.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter(
            "output_topic",
            "/ackermann_steering_controller/reference_unstamped")
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._relay, 10)
        self.get_logger().info(f"relaying {in_topic} -> {out_topic}")

    def _relay(self, msg: Twist):
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(CmdVelRelay())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
