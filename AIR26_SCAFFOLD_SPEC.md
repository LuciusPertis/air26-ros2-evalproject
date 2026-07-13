# AIR26_SCAFFOLD_SPEC

**Build spec for the AIR26 ROS2 workshop final-evaluation repository.**
This file is the source of truth for the *scaffolding agent*. It is **not** a
changelog and should not be edited to track progress — keep agent progress in a
separate `PROGRESS.md`. Do not rename or overwrite this file while working.

---

## 0. What this repo is

A take-home evaluation for the AIR26 ROS2 workshop. Students **fork** this repo,
run a working simulated robot, and complete four tasks by editing the shared
packages. Grading is subjective; the repo's job is to (a) run out-of-the-box as a
satisfying demo, and (b) leave four clean, well-marked gaps for students to fill.

**Robot:** 4-wheel car-like base — front two wheels **steer**, rear two wheels are
**driven** (both spun at equal velocity **but modeled as two separate joints**,
no differential) — plus a **2–3 DOF arm** ending in a flat "swiper" paddle for
clearing light obstacles. Sensing: **one lidar at the rear**, plus ~**8 range
sensors** (`sensor_msgs/Range`): front-center / front-left / front-right, left,
right, rear, and 1–2 up-angled "canopy" sensors for the overhead swipe.

**Kinematics note for the description + odometry:** this is an **Ackermann /
bicycle** vehicle, not diff-drive. Rear-axle traction, front steering. Steady
turns will scrub the rigid rear pair — that's expected; handle it in the physics
config (see per-engine notes), not by hiding it.

---

## 1. Hard rules for the agent

1. **Do NOT implement the four student tasks.** Scaffold them as runnable stubs
   with explicit `# TODO(student)` markers and comments pointing at the math /
   docs. The tasks are: (T1) URDF edits, (T2) odometry integration, (T3) the
   robot-info server, (T4) behavior-logic modification. Everything *around* these
   must work; the gaps themselves must not be solved. Specifics in §4.
2. **The ROS interface is an invariant across engine branches.** Identical topic
   names, frame ids, service/action names, and parameter names on the MuJoCo and
   Gazebo branches. Only the description-vs-MJCF, the physics driver, and the
   controller wiring may differ. If a name would drift between branches, stop and
   flag it instead.
3. **Everything above the driver/topic line lives in shared packages** (§3), byte-
   identical across branches. Engine-specific code is quarantined to one package.
4. **Custom interfaces live in `air26_interfaces`** and are meant to be edited by
   students. Use them for all behavior-facing services/actions. Keep the genuinely
   standard types standard: `geometry_msgs/Twist`, `sensor_msgs/Range`,
   `sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `sensor_msgs/JointState`,
   `trajectory_msgs/JointTrajectory`. Do **not** wrap those.
5. **Sim time everywhere.** The driver publishes `/clock`; every node runs with
   `use_sim_time:=true`. Follow **REP-103** (x-forward, y-left, z-up) and
   **REP-105** frames (`odom` → `base_link` owned by T2; `base_link` → wheels/arm/
   sensors owned by `robot_state_publisher`). Never double-publish a transform.

---

## 2. Build order (milestones)

Build the shared skeleton once, then the MuJoCo engine, then Gazebo. Webots is a
stretch goal only.

- **M0 — Skeleton (branch `main`):** all shared packages (§3), interfaces (§5),
  URDF description, launch glue, teleop + twist_mux, behavior nodes working
  against a *fake driver* (a tiny node that echoes commanded joint states so the
  graph is testable with no physics). Acceptance: `colcon build` clean;
  `check_urdf` passes on expanded xacro; `ros2 launch air26_bringup demo.launch.py`
  brings up RViz + TF + all topics/services/actions with the fake driver.
- **M1 — MuJoCo (branch `mujoco`):** real MuJoCo driver + MJCF world + light
  obstacles; replaces the fake driver. Everything in §6 must work. This is the
  reference implementation — get it fully right before touching Gazebo.
- **M2 — Gazebo (branch `gazebo`):** Gazebo world + `gz_ros2_control` +
  `ackermann_steering_controller` (available in Humble binaries) or an equivalent
  custom controller, + `joint_trajectory_controller` for the arm. Same interface
  as M1.
- **M3 — Webots (stretch, branch `webots`):** only if M0–M2 are solid.

Merge policy: shared packages are developed on `main` and merged into engine
branches; engine branches never modify shared package public interfaces.

---

## 3. Repository layout

```
air26_eval/
├── AIR26_SCAFFOLD_SPEC.md        # this file (do not overwrite)
├── PROGRESS.md                   # agent progress log (separate from this spec)
├── README.md                     # short: what it is, how to fork, how to run
├── STUDENT_GUIDE.md              # student-facing task guide (provided separately)
└── src/
    ├── air26_interfaces/         # SHARED — custom srv/action/msg, student-editable
    ├── air26_description/        # SHARED — urdf/xacro, meshes, sensor macros   (T1)
    ├── air26_teleop/             # SHARED — teleop + twist_mux config
    ├── air26_bringup/            # SHARED — top-level launch + params
    ├── air26_odometry/           # SHARED — odom node, integration STUBBED       (T2)
    ├── air26_robot_info/         # SHARED — robot-info server, STUBBED           (T3)
    ├── air26_behaviors/          # SHARED — 4 behavior nodes, WORKING            (T4)
    ├── air26_fake_driver/        # SHARED — M0 test driver (no physics)
    ├── air26_mujoco/             # ENGINE — MuJoCo driver + MJCF + obstacles     (mujoco branch)
    ├── air26_gazebo/             # ENGINE — Gazebo world + ros2_control          (gazebo branch)
    └── air26_webots/             # ENGINE — stretch                             (webots branch)
```

---

## 4. The four student tasks — scaffold, don't solve

**T1 — URDF change (`air26_description`).** Ship a clean, working xacro. Keep it
**readable**: macro-ize the range sensors so the URDF isn't a wall of near-
duplicate blocks. Students may resize/reshape a link, add a link, add a joint or
sensor, or write a controller/publisher for what they added. Leave a
`## student additions go here` anchor comment. **Do implement** the base robot
fully; the "gap" here is just headroom, not a stub.

**T2 — Odometry (`air26_odometry`).** Ship a node that subscribes to
`/joint_states` and reads `wheel_radius`, `wheelbase`, `track_width` as params.
The integration function is **stubbed**: `compute_odometry()` returns identity/zero
pose with a `# TODO(student)` and a comment block stating the **bicycle model**
(rear-axle-midpoint speed + front steering angle → yaw rate; integrate in SE(2);
REP-103). It must publish a valid (frozen) `nav_msgs/Odometry` + `odom→base_link`
TF so nothing downstream crashes. **Do not** subscribe to any ground-truth pose.
**Do not** implement the integration.

**T3 — Robot-info server (`air26_robot_info`).** Ship a node skeleton with a
service (custom `DescribeRobot.srv`, §5) that is meant to read `/robot_description`,
parse the URDF, and print structural info (joint/link count, a joint's limits, a
link's mass) to the terminal. Provide a **minimal working example** — e.g. echo one
hardcoded parameter — and a `# TODO(student)` to extend it into real URDF
introspection. Pre-empt the trap: `/robot_description` is latched
(`transient_local`); ship the subscriber QoS correctly so students don't lose an
hour to an empty callback. **Do not** implement full introspection.

**T4 — Behavior modification (`air26_behaviors`).** All four behaviors ship
**fully working** (§6). The task is for students to *modify* one meaningfully
(add a waypoint/condition, add a second reactive zone, change swipe geometry,
retune backup). Mark each behavior's "obvious modification point" with a comment.

---

## 5. `air26_interfaces` (editable by students)

Define these; keep field comments generous since students will read and edit them.

**`srv/SwipeArm.srv`**
```
uint8 PLANE_GROUND=0
uint8 PLANE_CANOPY=1
uint8 plane
uint8 DIR_LEFT_TO_RIGHT=0
uint8 DIR_RIGHT_TO_LEFT=1
uint8 direction
---
bool success
string message
```

**`srv/SetArmPose.srv`**
```
uint8 POSE_STOW=0
uint8 POSE_READY=1
uint8 pose
---
bool success
string message
```

**`srv/SetReactiveMode.srv`**
```
bool enable
---
bool success
string message
```

**`srv/DescribeRobot.srv`**  (T3 — students extend both the srv and the node)
```
string query        # e.g. a joint name, link name, or "" for a summary
---
bool success
string report       # human-readable; the node also prints this to the terminal
```

**`action/BackupToClearing.action`**
```
# goal
float32 clearance_threshold_m   # stop once rear min-range exceeds this
float32 timeout_s               # hard cap on backup duration
---
# result
bool cleared
bool timed_out
float32 final_clearance_m
float32 elapsed_s
---
# feedback
float32 current_clearance_m
float32 elapsed_s
```

---

## 6. Behavior + command-routing spec (`air26_behaviors`, `air26_teleop`)

**Base command arbitration.** Only two producers touch base velocity: teleop and
backup. Route both through **`twist_mux`**:
- `teleop/cmd_vel` — low priority (default driver).
- `backup/cmd_vel` — high priority; outranks teleop only while the backup action
  is active, then releases.
- mux output → the driver's base command topic.

**Arm command path (engine-agnostic).** Behaviors publish
`trajectory_msgs/JointTrajectory` on a fixed topic (e.g. `/arm_controller/joint_trajectory`).
Gazebo honors it via `joint_trajectory_controller`; the MuJoCo driver subscribes to
the same topic and tracks the target. Arm behaviors never touch `cmd_vel`, so they
never contend with the mux.

**The four behaviors (all shipped working):**
1. **Backup-to-clearing** — a `BackupToClearing` **action** server. Reverse
   **straight**, steering centered. Stop when rear lidar/ultrasonic min-range
   exceeds `clearance_threshold_m` **or** `timeout_s` trips. Cancelable; publish
   `current_clearance_m` + `elapsed_s` as feedback. No maneuvering/steering during
   backup — it is a straight reverse only.
2. **Manual swipe** — `SwipeArm` **service**. Executes a scripted arm trajectory
   for the requested `plane` (ground/canopy) and `direction`.
3. **Stow / Ready** — `SetArmPose` **service**. Two named joint configurations.
4. **Reactive swipe** — watches the front/canopy `Range` sensors and auto-calls
   the swipe trajectory. Gated by a **`SetReactiveMode` toggle (default OFF)** and
   a per-swipe **cooldown/debounce** so it doesn't chatter while an obstacle sits
   in range. This node's trigger condition is the marked T4 modification point.

---

## 7. Engine-specific notes

**MuJoCo (`air26_mujoco`, reference).** Lightweight driver on the `mujoco` python
bindings: load the MJCF, step the sim on a timer, publish `/clock`, `/joint_states`,
lidar (`LaserScan`) and the range sensors (`Range`); subscribe to the mux base
command + arm trajectory topic and map to actuators. **The driver's own odometry
publisher is a dummy** (identity) — real odom is the student's T2 node, which reads
`/joint_states`. Rear no-diff: drive both rear joints at equal velocity; give the
rear wheels reduced lateral friction so steady turns scrub cleanly instead of
fighting. Obstacles: **light** free bodies (small low-mass boxes/capsules) on the
ground and a couple at canopy height — no leaf meshes; they only need to be light
enough that a swipe displaces them.

**Gazebo (`air26_gazebo`).** SDF world with the same light obstacle props.
`gz_ros2_control` + `ackermann_steering_controller` (Humble binary) for the base,
or a custom controller if you prefer to leave T1.4 more open;
`joint_trajectory_controller` for the arm. Same friction handling for the rear
scrub. Keep all topic/frame/param names identical to the MuJoCo branch.

---

## 8. Acceptance checklist (per engine branch)

- `colcon build --symlink-install` clean; `check_urdf` passes on expanded xacro.
- `ros2 launch air26_bringup demo.launch.py` → RViz shows the robot, full TF tree,
  `/clock` advancing, all range + lidar topics live.
- Teleop drives the base; backup action reverses-and-stops on clearance/timeout and
  is cancelable; swipe/stow/ready services execute; reactive toggle works with
  cooldown.
- T2 node publishes a (frozen) `Odometry` + `odom→base_link` without crashing.
- T3 service returns its minimal example without crashing.
- Topic/service/action/param/frame names are identical to the other engine branch.
