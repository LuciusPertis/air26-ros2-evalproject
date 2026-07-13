"""Shared arm constants + JointTrajectory builders for the behavior nodes.

All arm behaviors talk to the driver by publishing trajectory_msgs/JointTrajectory
on /arm_controller/joint_trajectory using these joint names (part of the frozen
interface). Poses and swipe geometry are gathered here so Task 4 tweaks
(different arc, height, direction) have one obvious place to live.
"""

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_TOPIC = "/arm_controller/joint_trajectory"

# order matters: positions in every trajectory point follow this order
ARM_JOINTS = ["arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint"]

# --- named poses [yaw, shoulder, elbow] (rad) ---
POSE_STOW = [0.0, 1.10, -1.40]    # folded compact
POSE_READY = [0.0, -0.30, 0.30]   # raised, centered, ready to swipe

# --- swipe geometry (Task 4 modification point lives in the swipe node) ---
YAW_SWEEP = 1.20                  # how far to sweep left/right of center (rad)
PLANE_SHOULDER = {                # shoulder pitch that sets the swipe height
    "ground": 0.60,              # paddle low, near the ground
    "canopy": -0.80,             # paddle high, overhead
}
PLANE_ELBOW = {
    "ground": 0.00,
    "canopy": -0.20,
}


def _duration(seconds: float) -> Duration:
    d = Duration()
    d.sec = int(seconds)
    d.nanosec = int((seconds - int(seconds)) * 1e9)
    return d


def _point(positions, t_from_start: float) -> JointTrajectoryPoint:
    p = JointTrajectoryPoint()
    p.positions = [float(x) for x in positions]
    p.time_from_start = _duration(t_from_start)
    return p


def make_pose_trajectory(positions, duration=1.5) -> JointTrajectory:
    """Single-point trajectory that moves the arm to `positions`."""
    traj = JointTrajectory()
    traj.joint_names = list(ARM_JOINTS)
    traj.points = [_point(positions, duration)]
    return traj


def make_swipe_trajectory(plane: str, left_to_right: bool,
                          step: float = 1.0) -> JointTrajectory:
    """Scripted swipe across the given plane and direction.

    Four phases: raise to the plane at the start edge, sweep across, lift off,
    return to center. `plane` is "ground" or "canopy".
    """
    shoulder = PLANE_SHOULDER[plane]
    elbow = PLANE_ELBOW[plane]
    start_yaw = YAW_SWEEP if left_to_right else -YAW_SWEEP
    end_yaw = -start_yaw

    traj = JointTrajectory()
    traj.joint_names = list(ARM_JOINTS)
    traj.points = [
        _point([start_yaw, shoulder, elbow], 1.0 * step),            # 1. arrive at start edge
        _point([end_yaw, shoulder, elbow], 2.2 * step),              # 2. sweep across
        _point([end_yaw, shoulder - 0.4, elbow], 2.8 * step),        # 3. lift off
        _point(POSE_READY, 3.8 * step),                              # 4. back to ready
    ]
    return traj
