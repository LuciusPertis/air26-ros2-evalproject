#!/usr/bin/env python3
"""air26_teleop — a tiny keyboard driver for the base.

Publishes geometry_msgs/Twist on /teleop/cmd_vel (the LOW-priority twist_mux
input). Run it in its own terminal so it has keyboard focus:

    ros2 run air26_teleop teleop_keyboard

Controls (hold-to-drive, decays to stop):
    w / s   forward / reverse
    a / d   steer left / right
    x       stop
    q       quit

linear.x is a forward speed (m/s); angular.z is a yaw-rate request that the
driver turns into a steering angle via the bicycle model.
"""

import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist

HELP = """\
air26 teleop:  w/s = drive,  a/d = steer,  x = stop,  q = quit
"""

LIN_STEP = 0.05
ANG_STEP = 0.1
LIN_MAX = 0.6
ANG_MAX = 1.2


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.pub = self.create_publisher(Twist, "/teleop/cmd_vel", 10)
        self.lin = 0.0
        self.ang = 0.0

    def publish(self):
        t = Twist()
        t.linear.x = self.lin
        t.angular.z = self.ang
        self.pub.publish(t)


def _get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    rclpy.init()
    node = TeleopKeyboard()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        while rclpy.ok():
            key = _get_key(settings)
            if key == "w":
                node.lin = _clamp(node.lin + LIN_STEP, -LIN_MAX, LIN_MAX)
            elif key == "s":
                node.lin = _clamp(node.lin - LIN_STEP, -LIN_MAX, LIN_MAX)
            elif key == "a":
                node.ang = _clamp(node.ang + ANG_STEP, -ANG_MAX, ANG_MAX)
            elif key == "d":
                node.ang = _clamp(node.ang - ANG_STEP, -ANG_MAX, ANG_MAX)
            elif key == "x":
                node.lin = 0.0
                node.ang = 0.0
            elif key == "q" or key == "\x03":  # q or Ctrl-C
                break
            node.publish()
            print(f"\rlin={node.lin:+.2f} m/s  ang={node.ang:+.2f} rad/s   ", end="")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # stop the robot on exit
        node.lin = 0.0
        node.ang = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
