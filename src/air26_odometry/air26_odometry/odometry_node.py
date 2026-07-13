#!/usr/bin/env python3
"""air26_odometry — wheel odometry for the car-like base.  (TASK 2)

WHAT SHIPS (working): a node that subscribes to /joint_states, reads the base
geometry as parameters, publishes a valid nav_msgs/Odometry on /odom, and
broadcasts the odom -> base_link transform. Downstream (RViz, TF) is happy.

WHAT'S MISSING (your task): compute_odometry() is a STUB. Right now it returns a
frozen identity pose, so the robot never appears to move in the odom frame. Your
job is to turn wheel motion into a pose estimate. See the docstring on
compute_odometry() for the model.

Hard rules (from the spec / student guide):
  * This is a bicycle (Ackermann) model, NOT diff-drive. Do not use diff-drive.
  * Publish odom -> base_link and ONLY that transform. robot_state_publisher
    already owns base_link -> wheels; publishing it twice is the classic bug.
  * You are NOT given ground truth. Do not subscribe to any true-pose topic.
  * Follow REP-103: x forward, y left, z up; yaw CCW positive.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")

        # --- base geometry (keep in sync with the URDF) ---
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheelbase", 0.30)
        self.declare_parameter("track_width", 0.26)
        # --- frames ---
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # --- which joints carry speed / steering ---
        self.declare_parameter("rear_wheel_joints",
                               ["rear_left_wheel_joint", "rear_right_wheel_joint"])
        self.declare_parameter("steer_joints",
                               ["front_left_steer_joint", "front_right_steer_joint"])

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.track_width = self.get_parameter("track_width").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.rear_wheel_joints = list(self.get_parameter("rear_wheel_joints").value)
        self.steer_joints = list(self.get_parameter("steer_joints").value)

        # --- estimated state (SE(2) pose in the odom frame) ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # bookkeeping for finite differences between /joint_states messages
        self.last_stamp = None
        self.last_wheel_pos = None   # dict joint -> position (rad)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self.get_logger().info(
            "odometry_node up — publishing a FROZEN pose until Task 2 is done")

    # ------------------------------------------------------------------
    def _on_joint_states(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        stamp = msg.header.stamp

        # compute dt and per-wheel deltas from the previous message
        if self.last_stamp is not None:
            dt = (stamp.sec - self.last_stamp.sec) + \
                 (stamp.nanosec - self.last_stamp.nanosec) * 1e-9
        else:
            dt = 0.0

        dx, dy, dtheta = self.compute_odometry(pos, vel, dt)
        self.x += dx
        self.y += dy
        self.theta = math.atan2(math.sin(self.theta + dtheta),
                                math.cos(self.theta + dtheta))

        self.last_stamp = stamp
        self.last_wheel_pos = {j: pos.get(j, 0.0) for j in self.rear_wheel_joints}

        self._publish(stamp)

    # ==================================================================
    # TODO(student): TASK 2 — implement the bicycle-model integration.
    # ==================================================================
    def compute_odometry(self, positions, velocities, dt):
        """Return the base-frame pose increment (dx, dy, dtheta) for this step.

        This is a CAR (bicycle / Ackermann) model. Sketch:

          1. Forward speed of the rear axle midpoint:
                 v = wheel_radius * omega_rear
             where omega_rear is the rear wheels' angular rate. You can read it
             from `velocities[<rear wheel>]`, or finite-difference the wheel
             ANGLE across steps: (positions[j] - self.last_wheel_pos[j]) / dt.
             Average the left and right rear wheels.

          2. Steering angle delta: average the two front steer joint angles from
             `positions[<steer joint>]`.

          3. Yaw rate from the bicycle model:
                 theta_dot = v / wheelbase * tan(delta)

          4. Integrate in SE(2) over dt (REP-103, base_link at the rear axle):
                 dtheta = theta_dot * dt
                 dx = v * cos(self.theta) * dt      # world-frame increment
                 dy = v * sin(self.theta) * dt
             (Simple Euler is fine; a midpoint/exact-arc form is a nice touch.)

        Guard against dt == 0 and against a missing self.last_wheel_pos on the
        first message.

        Right now this returns zeros -> the pose stays frozen at the origin.
        Delete the stub and implement the four steps above.
        """
        # --- STUB: no motion. Replace this. ---
        del positions, velocities, dt  # silence "unused" until you use them
        dx = 0.0
        dy = 0.0
        dtheta = 0.0
        return dx, dy, dtheta

    # ------------------------------------------------------------------
    def _publish(self, stamp):
        # odom -> base_link transform (this node owns it; nothing else may)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(tf)

        # matching Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)
        self.odom_pub.publish(odom)


def main():
    try:
        rclpy.init()
        rclpy.spin(OdometryNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
