# AIR26 ROS2 Final Evaluation

A take-home evaluation for the AIR26 ROS2 workshop. You get a **working simulated
car-like rover** with a clearing arm; you complete **four tasks** by editing the
shared packages. Grading is on your changes, not on getting the demo to run — it
already runs out of the box.

> New here? Read **[STUDENT_GUIDE.md](STUDENT_GUIDE.md)** for the robot overview
> and the four tasks. This README is just fork-and-run.

## What's in the box

A 4-wheel Ackermann rover (front wheels **steer**, rear wheels **drive**) with a
3-DOF swiper arm, a rear lidar, and a ring of 8 ultrasonic range sensors — plus
teleop, four working behaviors, odometry, and a robot-info service. It runs on a
**fake driver** (no physics) on `main`; engine branches (`mujoco`, `gazebo`) swap
in a real simulator behind an identical ROS interface.

## Requirements

- ROS 2 **Humble**
- `colcon`, `xacro`, `twist_mux`, `robot_state_publisher`, `rviz2`
  (all standard Humble binaries)

## Run it

```bash
# from the repo root
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch air26_bringup demo.launch.py
```

You get RViz with the robot, its TF tree, `/clock`, and all sensor topics. In
another terminal (sourced the same way):

```bash
# drive it
ros2 run air26_teleop teleop_keyboard          # w/s drive, a/d steer, x stop, q quit

# trigger behaviors
ros2 service call /set_arm_pose      air26_interfaces/srv/SetArmPose      "{pose: 1}"        # ready
ros2 service call /swipe_arm         air26_interfaces/srv/SwipeArm        "{plane: 0, direction: 0}"
ros2 service call /set_reactive_mode air26_interfaces/srv/SetReactiveMode "{enable: true}"
ros2 action  send_goal /backup_to_clearing air26_interfaces/action/BackupToClearing \
    "{clearance_threshold_m: 0.5, timeout_s: 15.0}" --feedback

# ask the robot about itself (Task 3)
ros2 service call /describe_robot air26_interfaces/srv/DescribeRobot "{query: ''}"
```

Run headless (no RViz) with `ros2 launch air26_bringup demo.launch.py rviz:=false`.

## Packages

| Package | Role | Your task |
|---|---|---|
| `air26_interfaces` | custom srv/action (editable) | supports T1/T3/T4 |
| `air26_description` | URDF/xacro + RViz | **T1** |
| `air26_odometry` | wheel odometry (integration stubbed) | **T2** |
| `air26_robot_info` | robot-info service (introspection stubbed) | **T3** |
| `air26_behaviors` | the 4 working behaviors | **T4** |
| `air26_teleop` | keyboard teleop + twist_mux | — |
| `air26_bringup` | top-level launch + params | — |
| `air26_fake_driver` | M0 no-physics driver | — |

See **[AIR26_SCAFFOLD_SPEC.md](AIR26_SCAFFOLD_SPEC.md)** for the full build spec and
**[PROGRESS.md](PROGRESS.md)** for the frozen ROS interface (topic/frame/service
names shared across engine branches).
