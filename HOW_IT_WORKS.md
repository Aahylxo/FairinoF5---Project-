# How it works

The whole project explained from nothing, in the order it fits together. For the
"why is it like this?" questions see [WHY.md](WHY.md); for every detail see
[REFERENCE.md](REFERENCE.md).

---

## In one paragraph

We commissioned a **FAIRINO FR5** — a 6-axis collaborative arm, 5 kg payload, 922 mm
reach, controller at `192.168.58.2`. It runs on **ROS 2 Humble** on Ubuntu 22.04 inside
**WSL2**, and can be driven four ways: **text commands** to the vendor's command server
(Method A), **MoveIt** planning in RViz (Method B), **RViz sliders** with no MoveIt, and
**your hand**, tracked by a RealSense camera. Everything except Method A goes through
**ros2_control**, which streams `ServoJ` to the robot over Ethernet.

---

## 1. ROS 2 in plain words

ROS 2 lets separate programs talk to each other. Each program is a **node**. Nodes
exchange messages; nobody needs to know who is on the other end. The camera, the hand
tracker, the planner and the robot driver are separate programs — the driver does not
know or care that a hand is steering it.

| Term | Meaning | In this project |
|---|---|---|
| **Node** | One running program | `ros2_control_node`, `move_group`, `robot_state_publisher` |
| **Topic** | A named one-way stream (publish / subscribe) | `/joint_states`, `/fairino5_controller/joint_trajectory` |
| **Service** | One request, one reply | `/fairino_remote_command_service` (Method A) |
| **Action** | A long goal with feedback, cancellable | `FollowJointTrajectory` — how MoveIt sends a plan |
| **Parameter** | A setting read at start-up | `update_rate: 100` |
| **Launch file** | Starts many nodes at once | `demo.launch.py`, `rviz_real.launch.py`, `teleop.launch.py` |
| **Package** | A folder colcon can build (`package.xml`) | `fairino_hardware`, `fairino_msgs`, … |
| **TF** | The tree of coordinate frames | `base_link` → … → `wrist3_link` |
| **DDS** | The network layer underneath | Fast DDS, localhost only |

**Topic vs service vs action:** a topic is a radio broadcast (joint states, 100 times a
second, nobody replies). A service is a phone call (`MoveJ(JNT2,5,0,0)` → `cmd_res='0'`).
An action is ordering a taxi (a goal, progress updates, can be cancelled, told when done).

---

## 2. The workspace: build and source

```
~/fairino_380_ws/
├── src/frcobot_ros2/   the vendor's driver, messages, robot model, MoveIt config
├── build/  install/  log/   made by colcon
└── *.py, *.launch.py   this project's own scripts, run directly
```

- **`colcon build --symlink-install`** finds every package in `src/`, builds them in
  dependency order (the driver is C++), and puts the result in `install/`.
  `--symlink-install` links `install/` to the source instead of copying it, so **editing a
  YAML, xacro or URDF needs no rebuild** — just relaunch. Only C++ changes need a rebuild
  (`--packages-select fairino_hardware` after `patch_driver.sh`).
- **Two `source` lines, in every terminal:** `/opt/ros/humble/setup.bash` (ROS itself)
  then `install/local_setup.bash` (our packages). Forget the second and you get
  `Package not found`, or RViz shows axes but no arm.

The four vendor packages:

| Package | What it is |
|---|---|
| `fairino_msgs` | Message and service types (`RemoteCmdInterface.srv`) |
| `fairino_hardware` | The driver: `ros2_cmd_server` (Method A) and the ros2_control plugin (everything else), on the vendor's `libfairino` SDK |
| `fairino_description` | The robot model: meshes and `fairino5_v6.urdf` |
| `fairino5_v6_moveit2_config` | MoveIt config and `demo.launch.py` |

---

## 3. Two ways the robot is commanded

**Never both at once** — each opens its own connection to the controller.

| | Method A | ros2_control (MoveIt, sliders, hand) |
|---|---|---|
| Start | `ros2 run fairino_hardware ros2_cmd_server` | `demo.launch.py` or `rviz_real.launch.py` |
| You send | A text command to a service | A trajectory to `fairino5_controller` |
| Robot API | `MoveJ` / `MoveL` — one move, the robot plans it | `ServoJ` — a position every 10 ms |
| Feedback | `/nonrt_state_data`, degrees | `/joint_states`, radians |

**Method A example** — always register a target, then move to it:

```bash
ros2 service call /fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'JNTPoint(2,90,-90,90,-90,-90,0)'}"
ros2 service call /fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'MoveJ(JNT2,5,0,0)'}"          # target 2, 5 % speed, tool 0, frame 0
```

`cmd_res='0'` means *accepted*, not *arrived*. `ResetAllError()` clears a latched alarm.

---

## 4. ros2_control, MoveIt and RViz

```
 RViz MotionPlanning ──Plan/Execute──▶ move_group (MoveIt: KDL IK, OMPL planner)
                                            │ FollowJointTrajectory action
 slider_bridge.py / hand_teleop.py ─────────┤ (or a trajectory topic, skipping MoveIt)
                                            ▼
                         fairino5_controller  (JointTrajectoryController, 100 Hz)
                                            │ position command
                         FairinoHardwareInterface  write(): ServoJ, cmdT 8 ms
                                            │ Ethernet
                                     FR5 controller  (must be in AUTO)
                                            │
                         read(): GetActualJointPosDegree → joint_state_broadcaster
                                            │ /joint_states
                         robot_state_publisher → /tf → RViz draws the arm
```

- **RViz only displays.** It cannot command anything.
- **MoveIt plans** a collision-free path and borrows RViz for its panel.
- **ros2_control** is the bridge to the motors: a *hardware plugin* with `read()` and
  `write()`, *controllers* on top, a *controller manager* running the loop at 100 Hz.
- **URDF** = what the robot *is* (links, joints, limits, meshes). **SRDF** = what to *do*
  with it (planning group `fairino5_v6_group`, named poses, collision pairs to skip).

**The sim/real switch** is one line in
`fairino5_v6_robot.ros2_control.xacro`: `mock_components/GenericSystem` (a fake robot
that instantly reaches every command) or `fairino_hardware/FairinoHardwareInterface` (the
real driver). The repo is committed on simulation. [Why one line does that](WHY.md#why-the-plugin-switch)

**The joint order trap:** this driver publishes `/joint_states` as j1, j2, j4, j5, j3,
j6. Every script here matches by **name**; matching by index swaps three joints.

---

## 5. Kinematics (`fr5_kinematics.py`)

Standard DH, one 4×4 transform per joint, `T = Rz(θ)·Tz(d)·Tx(a)·Rx(α)`:

| Joint | a (mm) | d (mm) | α | Soft limit (°) |
|---|---|---|---|---|
| 1 | 0 | 152 | +90° | ±175 |
| 2 | −425 | 0 | 0 | −265 … 85 |
| 3 | −395 | 0 | 0 | ±150 |
| 4 | 0 | 102 | +90° | −265 … 85 |
| 5 | 0 | 102 | −90° | ±175 |
| 6 | 0 | 100 | 0 | ±175 |

- **Forward kinematics** multiplies the six transforms: one answer. Home
  (0, −90, 90, −90, −90, 0) puts the tool about 696 mm from the base.
- **Inverse kinematics** is closed-form (the FR5 has a UR-style geometry): shoulder
  left/right × elbow up/down × wrist flip = **up to 8 solutions**, each verified by FK,
  filtered by soft limits, and the one nearest the current pose is used. A wrist
  singularity (sin θ5 ≈ 0) pins θ6 = 0.
- **Self-test:** 2000 random poses, FK → IK → FK, **zero failures**
  (`python3 fr5_kinematics.py selftest`). The DH offsets were also checked against the
  URDF (152, 425, 395, 102, 102, 100 mm).

MoveIt uses its own numerical solver (KDL). The hand control needs all eight branches
every frame, deterministically, so it uses the closed form.

---

## 6. Hand control (`hand_teleop.py`)

Your hand's position drives the **tool** position; IK turns that into joint angles.
25 times a second:

1. RealSense colour (+ aligned depth) → MediaPipe finds 21 hand landmarks.
2. Hand offset from the pose captured at **Capture Neutral**, scaled.
3. Clamped: 300 mm per axis from neutral; 180 < z < 900 mm; 250 < reach < 870 mm.
4. IK → nearest solution → reject any branch flip over 30°.
5. Low-pass, 5 °/s per joint, soft limits → one-point trajectory to the controller.

**Pinch** thumb and index to move — a deadman switch; open your fingers and it stops.
Four filters each catch something different (camera noise, residual drift, IK jitter,
everything else), and the guards — lead clamp, bad-feedback refusal, frozen-feedback
stop, auto-stop on 8° error — exist because the arm shook twice.
[The full story](WHY.md#why-the-arm-shook)

Launch with `ros2 launch ~/fairino_380_ws/teleop.launch.py` (camera + teleop, depth on;
`depth:=false` if frames drop over WSL).

---

## 7. RViz sliders on the real arm, no MoveIt

The prof's brief: move the robot from RViz alone. Out of the box the slider window
(`joint_state_publisher_gui`) only moves the *picture*. Three files make it drive the arm:

| File | Job |
|---|---|
| `rviz_real.launch.py` | Starts the robot with **no MoveIt**: robot_state_publisher, ros2_control, the two controllers, plain RViz |
| `slider_controllers.yaml` | The vendor's controller config plus `open_loop_control: true` — used only by that launch file |
| `slider_bridge.py` | Opens the slider window and turns slider moves into safe, smooth controller commands |

```
slider window ──/slider_joint_states──▶ slider_bridge.py ──trajectory──▶ fairino5_controller
                                              ▲                                 │
                                              └──────── /joint_states ◀─────────┘
```

What the bridge does, and why:

- **The slider window is remapped** to `/slider_joint_states`. On `/joint_states` it
  would fight the robot's real feedback.
- **The sliders start at the arm's real pose**, and nothing moves until every slider is
  within 3° of the arm.
- **It sends short straight segments, 10 a second, only while a slider is moving** —
  at most 1° from where the controller is, timed for a constant 5 °/s. A segment that
  short means a stalled control loop catching up can jump at most 1° in one cycle,
  under the FR5's speed limit.
- **Every goal is time-stamped**, and the controller runs **open loop**, so each segment
  continues the last one instead of restarting from a noisy measurement. Without these
  the arm vibrated. [Why](WHY.md#why-the-sliders-vibrated)
- **Safety:** soft limits (j3 ±150°), refuse all-zero feedback, stop if the arm stops
  following, stop for good if feedback is gone for 10 s.

What you see: drag a slider and the real arm glides there at 5 °/s; RViz follows the
**real arm**, not the sliders, because it draws `/joint_states`. **Center** returns the
sliders to the pose the arm had when the bridge started — a real move.

---

## 8. Things that went wrong, and what they taught

| Symptom | Cause | Lesson |
|---|---|---|
| MoveIt said SUCCEEDED, arm still | Robot in **MANUAL** mode — refuses `ServoJ` only | When software says fine and the robot won't move, look at the robot. [Story](WHY.md#why-the-robot-refuses-commands-error-14) |
| "Command speed of axis N exceeds the limit" | Vendor speed limits above 180 °/s; no `ServoMoveStart()`; a big goal plus a loop stall | Fixed limits, `patch_driver.sh`, 1° slider segments |
| Arm shook (hand control) | Command ran ahead of the arm; driver reported all zeros | Lead clamp; refuse bad feedback |
| Arm vibrated (sliders) | Goals restarted every 40 ms; unstamped goals made the controller hold a cycle each time | Stamped goals, open loop, send only on change |
| `/joint_states` in ~1 s bursts | Reliable subscription on WSL waits for lost samples | Subscribe best-effort. [Why](WHY.md#why-joint_states-arrived-in-bursts) |
| `update_rate` 125 Hz | Changed to "tidy" two numbers; removed WSL's timing slack | A working system is not an invitation to tidy it |

---

*Run it: [RUN_SIMULATION.md](RUN_SIMULATION.md) · [RUN_REAL_ROBOT.md](RUN_REAL_ROBOT.md)*
