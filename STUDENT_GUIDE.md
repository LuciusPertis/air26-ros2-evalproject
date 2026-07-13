# AIR26 Final Evaluation — Student Guide

You've forked a working simulated robot. Your job is to complete **four tasks** by
editing the packages below. The robot already drives, senses, and clears obstacles
out of the box — run it first, understand it, then make your changes.

---

## The robot

A small car-like rover with a clearing arm — think "walking through a forest and
brushing branches aside."

- **Base:** 4 wheels. The **front two steer**, the **rear two drive**. It steers
  like a car (Ackermann), *not* like a differential-drive robot — this matters for
  Task 2.
- **Arm:** 2–3 joints ending in a flat paddle, for swiping light obstacles on the
  ground or overhead.
- **Sensing:** a **lidar at the rear**, plus several **ultrasonic range sensors**
  (front, sides, rear, and a couple angled upward for the canopy). Use whichever
  suit your task — you don't have to use all of them.

Everything runs in simulation. You picked an engine branch when you forked
(**MuJoCo** or **Gazebo**); the ROS interface is identical either way, so tasks 2,
3, and 4 are the same code on both.

---

## Running it

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch air26_bringup demo.launch.py
```

You'll get RViz, the robot, its TF tree, and all sensor topics. Drive it with the
teleop node, and trigger the behaviors (below) with `ros2 service call` /
`ros2 action send_goal`. Poke around with `ros2 topic list`, `ros2 topic echo`,
`ros2 param list`, and `ros2 run tf2_tools view_frames` before you edit anything.

---

## Project structure

Packages you'll **edit** are marked. Everything else is here for context.

| Package | What it is | Your task |
|---|---|---|
| `air26_interfaces` | Custom services/actions/messages. **You may edit these** — add fields if a task needs them. | (supports T1/T4) |
| `air26_description` | The robot's URDF/xacro, meshes, sensor definitions. | **Task 1** |
| `air26_odometry` | Turns wheel motion into a position estimate. Integration is left blank. | **Task 2** |
| `air26_robot_info` | A small server that reports facts about the robot. Skeleton only. | **Task 3** |
| `air26_behaviors` | The four working behaviors (backup, swipe, stow/ready, reactive). | **Task 4** |
| `air26_teleop` | Keyboard/joystick driving + command arbitration. | — |
| `air26_bringup` | Launch files that wire everything together. | — |
| `air26_<engine>` | The physics driver + world for your branch. | — |

**A note on interfaces:** the behavior services/actions aren't hidden ROS built-ins
— they live in `air26_interfaces` as plain `.srv`/`.action` files you can open and
change. If a task makes you want an extra field (a swipe speed, a number of
repeats), add it there and rebuild. The truly standard types (`Twist`, `Range`,
`LaserScan`, `Odometry`, `JointState`) are left as-is.

---

## The behaviors (already working — study them for Task 4)

- **Backup to clearing** — one command reverses the rover in a **straight line**
  until the space behind it is clear (or a time limit is hit). It's an *action*, so
  you get live feedback and can cancel it.
- **Swipe** — the arm sweeps across, on the ground or overhead, left-to-right or
  right-to-left.
- **Stow / Ready** — move the arm to a folded or a raised-and-ready pose.
- **Reactive swipe** — when something light appears in front of / above the rover,
  it swipes automatically. Off by default; toggle it on, and it has a cooldown so
  it doesn't swipe non-stop.

---

## Your four tasks

### Task 1 — Change the robot's body (`air26_description`)
Do **any one** of these (more is welcome):
- Resize or reshape an existing link.
- Add a new link.
- Add a new joint or a new sensor.
- Write a controller or publisher for something you added above.

Check your work: `check_urdf` on the expanded description, and confirm your new
frame shows up in `view_frames` / RViz. There's a marked spot in the URDF for your
additions.

### Task 2 — Fill in the odometry (`air26_odometry`)
The node already reads the wheel joints and the robot's dimensions, but the
function that turns wheel motion into a pose estimate is empty. Implement it.

- This is a **car-like (bicycle-model)** robot: the rear axle provides forward
  speed, the front steering angle sets how fast the heading turns. It is **not**
  diff-drive — don't use the diff-drive equations.
- Follow **REP-103** (x forward, y left, z up).
- Publish `nav_msgs/Odometry` **and** the `odom → base_link` transform — and only
  that transform (`robot_state_publisher` already owns the wheels). Publishing it
  twice is the classic bug.
- You are **not** given the true position — that's the point. Estimate it.

Check your work: drive a known pattern (straight line, then a steady circle) and
see whether your estimated path matches what the robot actually did.

### Task 3 — A robot-info server (`air26_robot_info`)
Make a small service that prints useful facts about the robot to the terminal —
e.g. how many joints/links it has, a joint's limits, or a link's mass — by reading
and parsing the robot's description. A minimal example is provided; extend it.

- The robot description is *latched* — the subscriber QoS is already set correctly
  for you, so your callback will actually receive it.
- Nice touch: make it report on whatever you added in Task 1.

Check your work: `ros2 service call` your service and confirm the terminal output.

### Task 4 — Change a behavior (`air26_behaviors`)
Modify **one** behavior in a way that shows you understand it. For example:
- Give the reactive swipe a second trigger zone, or change what sets it off.
- Change the swipe geometry (arc, height, direction sequence).
- Retune the backup (different stop condition, add a short pause, etc.).
- Add a step to a behavior (e.g. ready the arm before swiping).

Each behavior has a comment marking an easy place to start. Keep it running without
crashing, and make the change actually do something visible.

---

## Tips

- Run everything with simulated time; the launch files already handle this.
- If a service/topic name isn't what you expect, list it (`ros2 service list`,
  `ros2 topic list`) rather than guessing.
- Rebuild after editing interfaces or C++/description files:
  `colcon build --symlink-install && source install/setup.bash`.
- A short screen recording of your working changes is the easiest way to show them
  off.
