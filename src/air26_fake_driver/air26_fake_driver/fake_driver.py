#!/usr/bin/env python3
"""air26_fake_driver — the M0 stand-in for a real physics engine.

There is NO physics here. This node exists so the whole ROS graph (teleop, mux,
behaviors, odometry, robot_info, RViz) is runnable and testable before the MuJoCo
/ Gazebo drivers exist. It:

  * owns sim time: steps a clock and publishes /clock,
  * echoes commanded motion into /joint_states (dead-reckoned wheel + arm angles),
  * subscribes the mux base command (/cmd_vel) and the arm trajectory topic,
  * publishes a rear /scan and the ring of /range/<name> sensors from a *scripted*
    virtual world — just enough for the backup + reactive behaviors to do something
    visible. These readings are synthetic and clearly marked; real sensing arrives
    with the engine branches.

Every published topic/frame here is part of the frozen interface (see PROGRESS.md),
so the MuJoCo/Gazebo drivers publish exactly the same names.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, Range
from trajectory_msgs.msg import JointTrajectory

# --- joint names (invariant; see PROGRESS.md) ---
STEER_JOINTS = ["front_left_steer_joint", "front_right_steer_joint"]
WHEEL_JOINTS = [
    "front_left_wheel_joint", "front_right_wheel_joint",
    "rear_left_wheel_joint", "rear_right_wheel_joint",
]
ARM_JOINTS = ["arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint"]

# --- range sensor ring (topic /range/<name>, frame <name>_range_link) ---
RANGE_SENSORS = [
    "front_center", "front_left", "front_right", "left", "right", "rear",
    "canopy_left", "canopy_right",
]
FRONT_SENSORS = ["front_center", "front_left", "front_right"]
CANOPY_SENSORS = ["canopy_left", "canopy_right"]


class FakeDriver(Node):
    def __init__(self):
        super().__init__("fake_driver")

        # geometry (kept in sync with the xacro / odometry params)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheelbase", 0.30)
        self.declare_parameter("track_width", 0.26)
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("range_max", 2.0)
        # synthetic-world knobs (M0 only)
        self.declare_parameter("rear_start_clearance", 0.25)   # initial rear gap (m)
        self.declare_parameter("front_obstacle_range", 0.32)   # obstacle ahead (m)
        self.declare_parameter("swipe_clear_seconds", 4.0)     # obstacle stays gone after a swipe

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.rate = self.get_parameter("update_rate").value
        self.range_max = self.get_parameter("range_max").value
        self.dt = 1.0 / self.rate

        # sim clock
        self.sim_time = 0.0

        # commanded base state
        self.cmd_v = 0.0          # rear-axle forward speed (m/s)
        self.cmd_steer = 0.0      # front steering angle (rad)

        # dead-reckoned internal pose — used ONLY for synthetic sensing, never
        # published as odometry (that is the Task 2 node's job).
        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0
        self.reversed_dist = 0.0  # cumulative straight-reverse distance

        # integrated joint positions
        self.wheel_pos = {j: 0.0 for j in WHEEL_JOINTS}
        self.arm_pos = {j: 0.0 for j in ARM_JOINTS}
        self.arm_target = dict(self.arm_pos)
        self.last_swipe_time = -1e9  # last time an arm trajectory arrived

        # ---- publishers ----
        best_effort = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", best_effort)
        self.range_pubs = {
            name: self.create_publisher(Range, f"/range/{name}", best_effort)
            for name in RANGE_SENSORS
        }

        # ---- subscribers ----
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(
            JointTrajectory, "/arm_controller/joint_trajectory",
            self._on_arm_traj, 10)

        # main loop on wall time (this node is the sim-time *source*)
        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info("fake_driver up — synthetic sensing, no physics (M0)")

    # ------------------------------------------------------------------ cmds
    def _on_cmd_vel(self, msg: Twist):
        self.cmd_v = msg.linear.x
        # bicycle steering: wz = v/L * tan(delta)  ->  delta = atan(L*wz/v)
        if abs(msg.linear.x) > 1e-3:
            self.cmd_steer = math.atan2(self.wheelbase * msg.angular.z, msg.linear.x)
        else:
            # in-place: interpret angular.z directly as a steer command
            self.cmd_steer = max(-0.6, min(0.6, msg.angular.z))

    def _on_arm_traj(self, msg: JointTrajectory):
        if not msg.points:
            return
        target = msg.points[-1].positions
        for name, val in zip(msg.joint_names, target):
            if name in self.arm_target:
                self.arm_target[name] = val
        self.last_swipe_time = self.sim_time
        self.get_logger().info(f"arm trajectory received ({len(msg.points)} pts)")

    # ------------------------------------------------------------------ loop
    def _step(self):
        self.sim_time += self.dt
        now = self._time_msg()

        # advance clock
        self.clock_pub.publish(Clock(clock=now))

        # integrate dead-reckoned pose (bicycle model, internal only)
        v = self.cmd_v
        if abs(v) > 1e-6:
            wz = v / self.wheelbase * math.tan(self.cmd_steer)
        else:
            wz = 0.0
        self.px += v * math.cos(self.pth) * self.dt
        self.py += v * math.sin(self.pth) * self.dt
        self.pth += wz * self.dt
        # rear clearance grows while reversing and re-closes while driving
        # forward, so the backup demo is repeatable.
        self.reversed_dist = max(0.0, self.reversed_dist - v * self.dt)

        # roll wheels: omega = v / r
        wheel_omega = v / self.wheel_radius
        for j in WHEEL_JOINTS:
            self.wheel_pos[j] += wheel_omega * self.dt

        # move arm toward its target at a limited rate
        arm_speed = 1.5  # rad/s
        for j in ARM_JOINTS:
            err = self.arm_target[j] - self.arm_pos[j]
            step = max(-arm_speed * self.dt, min(arm_speed * self.dt, err))
            self.arm_pos[j] += step

        self._publish_joint_states(now)
        self._publish_ranges(now)
        self._publish_scan(now)

    # -------------------------------------------------------------- outputs
    def _publish_joint_states(self, now):
        js = JointState()
        js.header.stamp = now
        # steering angle mirrors the command on both front knuckles
        js.name = list(STEER_JOINTS) + list(WHEEL_JOINTS) + list(ARM_JOINTS)
        js.position = (
            [self.cmd_steer, self.cmd_steer]
            + [self.wheel_pos[j] for j in WHEEL_JOINTS]
            + [self.arm_pos[j] for j in ARM_JOINTS]
        )
        self.joint_pub.publish(js)

    def _front_obstacle_present(self):
        """Scripted front/canopy obstacle: present, but 'knocked away' for a few
        seconds after each swipe so reactive-swipe shows a clear cause/effect."""
        clear_win = self.get_parameter("swipe_clear_seconds").value
        return (self.sim_time - self.last_swipe_time) > clear_win

    def _publish_ranges(self, now):
        rear_start = self.get_parameter("rear_start_clearance").value
        front_obs = self.get_parameter("front_obstacle_range").value

        for name in RANGE_SENSORS:
            r = Range()
            r.header.stamp = now
            r.header.frame_id = f"{name}_range_link"
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = 0.5
            r.min_range = 0.02
            r.max_range = self.range_max

            if name == "rear":
                # synthetic: rear opens up as the rover reverses straight
                val = min(self.range_max, rear_start + self.reversed_dist)
            elif name in FRONT_SENSORS or name in CANOPY_SENSORS:
                # obstacle present -> close reading; recently swiped -> cleared
                val = front_obs if self._front_obstacle_present() else self.range_max
                # only the center + canopy actually face the scripted obstacle
                if name in ("front_left", "front_right"):
                    val = self.range_max
            else:
                val = self.range_max  # sides: clear

            r.range = float(val)
            self.range_pubs[name].publish(r)

    def _publish_scan(self, now):
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "lidar_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        n = 180
        scan.angle_increment = (scan.angle_max - scan.angle_min) / n
        scan.range_min = 0.05
        scan.range_max = self.range_max
        # flat synthetic ring at max range (no obstacles modeled in the lidar plane)
        scan.ranges = [self.range_max] * n
        self.scan_pub.publish(scan)

    # --------------------------------------------------------------- helpers
    def _time_msg(self) -> TimeMsg:
        t = TimeMsg()
        t.sec = int(self.sim_time)
        t.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
        return t


def main():
    try:
        rclpy.init()
        node = FakeDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
