#!/usr/bin/env python3
"""air26_mujoco — the MuJoCo physics driver (M1 reference engine).

Loads the MJCF, steps the sim on a wall-clock timer, and speaks the SAME ROS
interface as the M0 fake driver (see PROGRESS.md) so every shared node — teleop,
twist_mux, odometry (T2), robot_info (T3), behaviors (T4) — runs unchanged:

  publishes : /clock, /joint_states, /scan (rear lidar), /range/<name> (x8)
  subscribes: /cmd_vel (mux base command), /arm_controller/joint_trajectory

Mapping:
  * /cmd_vel.linear.x  -> rear wheel velocity (omega = v / wheel_radius), both
    rear wheels equal (no differential).
  * /cmd_vel.angular.z -> front steering angle via the bicycle model, applied to
    both front steer joints (position servos).
  * arm trajectory     -> the arm position servos track the last point.

Sensing is real: range sensors and the lidar are `mj_ray` casts against the
obstacle group. The driver publishes NO odometry and NO odom->base_link TF — that
is the student's Task 2 node (this driver's own odom would only ever be identity).

The driver is the CLOCK SOURCE, so it runs on wall time (use_sim_time:=false).
"""

import math
import os

import numpy as np

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, Range
from trajectory_msgs.msg import JointTrajectory

import mujoco
try:
    import mujoco.viewer as mujoco_viewer
except Exception:  # headless without GL, etc.
    mujoco_viewer = None

# --- joint names (frozen interface; order matches the URDF / fake driver) ---
STEER_JOINTS = ["front_left_steer_joint", "front_right_steer_joint"]
WHEEL_JOINTS = [
    "front_left_wheel_joint", "front_right_wheel_joint",
    "rear_left_wheel_joint", "rear_right_wheel_joint",
]
ARM_JOINTS = ["arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint"]
JOINT_ORDER = STEER_JOINTS + WHEEL_JOINTS + ARM_JOINTS

RANGE_SENSORS = [
    "front_center", "front_left", "front_right", "left", "right", "rear",
    "canopy_left", "canopy_right",
]

# arm home (stow) — used to initialise the sim so the arm doesn't drop on spawn
ARM_STOW = {"arm_yaw_joint": 0.0, "arm_shoulder_joint": 1.10, "arm_elbow_joint": -1.40}

# obstacle geom group the rays may hit (group 3 in the MJCF)
RAY_GROUP = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)


class MujocoDriver(Node):
    def __init__(self):
        super().__init__("mujoco_driver")

        default_model = os.path.join(
            get_package_share_directory("air26_mujoco"), "mjcf", "air26_rover.xml")
        self.declare_parameter("model_path", "")
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheelbase", 0.30)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("lidar_rays", 180)
        self.declare_parameter("range_max", 2.0)
        self.declare_parameter("cmd_timeout", 0.5)   # stop base if no cmd_vel
        self.declare_parameter("viewer", False)      # passive GUI (needs a display)

        model_path = self.get_parameter("model_path").value or default_model
        model_path = os.path.abspath(model_path)
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.rate = self.get_parameter("publish_rate").value
        self.n_lidar = int(self.get_parameter("lidar_rays").value)
        self.range_max = self.get_parameter("range_max").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value

        self.get_logger().info(f"loading MJCF: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.timestep = self.model.opt.timestep
        self.substeps = max(1, int(round((1.0 / self.rate) / self.timestep)))

        # cache joint address lookups
        self._qadr = {j: self.model.joint(j).qposadr[0] for j in JOINT_ORDER}
        self._vadr = {j: self.model.joint(j).dofadr[0] for j in JOINT_ORDER}
        self._act = {self.model.actuator(i).name: i for i in range(self.model.nu)}

        # commanded base state
        self.cmd_v = 0.0
        self.cmd_steer = 0.0
        self.last_cmd_time = -1e9
        # arm targets (start stowed)
        self.arm_target = dict(ARM_STOW)

        # initialise arm at stow and settle so nothing jumps on spawn
        for j, val in ARM_STOW.items():
            self.data.qpos[self._qadr[j]] = val
        mujoco.mj_forward(self.model, self.data)
        self._apply_ctrl()
        for _ in range(400):
            self._apply_ctrl()
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0.0  # define t=0 at the settled state

        # scratch buffers for mj_ray
        self._gid = np.zeros(1, dtype=np.int32)

        # ---- publishers ----
        best_effort = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
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
            JointTrajectory, "/arm_controller/joint_trajectory", self._on_arm_traj, 10)

        # optional passive viewer
        self.viewer = None
        if self.get_parameter("viewer").value:
            if mujoco_viewer is None:
                self.get_logger().warn("mujoco.viewer unavailable; running headless")
            else:
                try:
                    self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
                    self.get_logger().info("passive viewer launched")
                except Exception as exc:  # no display, etc.
                    self.get_logger().warn(f"viewer unavailable: {exc}")

        self.timer = self.create_timer(1.0 / self.rate, self._step)
        self.get_logger().info(
            f"mujoco_driver up — real physics, {self.substeps} substeps/tick")

    # ------------------------------------------------------------------ cmds
    def _on_cmd_vel(self, msg: Twist):
        self.cmd_v = msg.linear.x
        if abs(msg.linear.x) > 1e-3:
            self.cmd_steer = math.atan2(self.wheelbase * msg.angular.z, msg.linear.x)
        else:
            self.cmd_steer = max(-0.6, min(0.6, msg.angular.z))
        self.last_cmd_time = self.data.time

    def _on_arm_traj(self, msg: JointTrajectory):
        if not msg.points:
            return
        target = msg.points[-1].positions
        for name, val in zip(msg.joint_names, target):
            if name in self.arm_target:
                self.arm_target[name] = float(val)
        self.get_logger().info(f"arm trajectory received ({len(msg.points)} pts)")

    def _apply_ctrl(self):
        # base: drop the command if it has gone stale (safety)
        v, steer = self.cmd_v, self.cmd_steer
        if (self.data.time - self.last_cmd_time) > self.cmd_timeout:
            v, steer = 0.0, self.cmd_steer  # coast to stop, hold steering
        omega = v / self.wheel_radius
        self.data.ctrl[self._act["drive_left"]] = omega
        self.data.ctrl[self._act["drive_right"]] = omega
        self.data.ctrl[self._act["steer_left"]] = steer
        self.data.ctrl[self._act["steer_right"]] = steer
        # arm: position servos track the target
        self.data.ctrl[self._act["arm_yaw"]] = self.arm_target["arm_yaw_joint"]
        self.data.ctrl[self._act["arm_shoulder"]] = self.arm_target["arm_shoulder_joint"]
        self.data.ctrl[self._act["arm_elbow"]] = self.arm_target["arm_elbow_joint"]

    # ------------------------------------------------------------------ loop
    def _step(self):
        for _ in range(self.substeps):
            self._apply_ctrl()
            mujoco.mj_step(self.model, self.data)

        now = self._time_msg(self.data.time)
        self.clock_pub.publish(Clock(clock=now))
        self._publish_joint_states(now)
        self._publish_ranges(now)
        self._publish_scan(now)
        if self.viewer is not None:
            try:
                self.viewer.sync()
            except Exception:
                self.viewer = None

    # -------------------------------------------------------------- outputs
    def _publish_joint_states(self, now):
        js = JointState()
        js.header.stamp = now
        js.name = list(JOINT_ORDER)
        js.position = [float(self.data.qpos[self._qadr[j]]) for j in JOINT_ORDER]
        js.velocity = [float(self.data.qvel[self._vadr[j]]) for j in JOINT_ORDER]
        self.joint_pub.publish(js)

    def _cast(self, site_name):
        """Distance from a sensor site along its local +x, or range_max if clear."""
        site = self.data.site(site_name)
        pnt = np.array(site.xpos, dtype=np.float64)
        vec = np.array(site.xmat, dtype=np.float64).reshape(3, 3)[:, 0].copy()
        self._gid[0] = -1
        dist = mujoco.mj_ray(self.model, self.data, pnt, vec,
                             RAY_GROUP, 1, -1, self._gid)
        if dist < 0.0 or dist > self.range_max:
            return self.range_max
        return dist

    def _publish_ranges(self, now):
        for name in RANGE_SENSORS:
            r = Range()
            r.header.stamp = now
            r.header.frame_id = f"{name}_range_link"
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = 0.5
            r.min_range = 0.02
            r.max_range = self.range_max
            r.range = float(self._cast(f"{name}_range"))
            self.range_pubs[name].publish(r)

    def _publish_scan(self, now):
        site = self.data.site("lidar")
        pnt = np.array(site.xpos, dtype=np.float64)
        R = np.array(site.xmat, dtype=np.float64).reshape(3, 3)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "lidar_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        n = self.n_lidar
        scan.angle_increment = (scan.angle_max - scan.angle_min) / n
        scan.range_min = 0.05
        scan.range_max = self.range_max

        ranges = []
        for i in range(n):
            a = scan.angle_min + i * scan.angle_increment
            local = np.array([math.cos(a), math.sin(a), 0.0])
            vec = R @ local
            self._gid[0] = -1
            dist = mujoco.mj_ray(self.model, self.data, pnt, vec,
                                 RAY_GROUP, 1, -1, self._gid)
            ranges.append(float(dist) if 0.0 <= dist <= self.range_max else self.range_max)
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    # --------------------------------------------------------------- helpers
    def _time_msg(self, t: float) -> TimeMsg:
        msg = TimeMsg()
        msg.sec = int(t)
        msg.nanosec = int((t - int(t)) * 1e9)
        return msg


def main():
    try:
        rclpy.init()
        rclpy.spin(MujocoDriver())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
