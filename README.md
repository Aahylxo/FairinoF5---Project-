# FAIRINO FR5 — ROS 2 Commissioning & Teleoperation

Robotics Mechanisms course project. Commissioning a FAIRINO FR5 collaborative arm,
driving it from ROS 2 Humble, and teleoperating it from human arm movement captured by an
Intel RealSense D455.

**Repo:** https://github.com/Aahylxo/FairinoF5---Project-

---

> # ▶ New here? Read [`FOLLOW_ALONG.md`](FOLLOW_ALONG.md) instead.
>
> That document takes a Windows laptop with **nothing installed** and walks it, step by
> numbered step, to a robot arm controlled by hand movement. Every command, every
> expected result, and what to do when something fails.
>
> **Sections A–E of it need no robot at all** — the whole system runs in simulation on
> any Windows laptop with a RealSense camera.
>
> **This document is the reference behind it:** why each step exists, how the system
> works, the robot's parameters, and the full troubleshooting tables. Read it when you
> want to understand something, not when you want to get running.

---

## Contents

**[PART 0 — MAIN INSTRUCTIONS](#part-0--main-instructions)** ← always start here

**[PART 1 — Common setup](#part-1--common-setup)**
- [1.1 One-time install](#11-one-time-install-already-done)
- [1.2 Every-session startup](#12-every-session-startup)
- [1.3 Verify communication](#13-verify-communication)

**[PART 2 — METHOD A: Terminal control](#part-2--method-a-terminal-control)**
- [2.1 Start the command server](#21-start-the-command-server)
- [2.2 Joint moves](#22-joint-moves-movej)
- [2.3 Linear moves](#23-linear-moves-movel)
- [2.4 Jogging](#24-jogging)
- [2.5 Stopping](#25-stopping-method-a)

**[PART 3 — METHOD B: RViz + MoveIt control](#part-3--method-b-rviz--moveit-control)**
- [3.1 Install MoveIt and build](#31-install-moveit-and-build)
- [3.2 Test in simulation first](#32-test-in-simulation-first)
- [3.3 Switch to real hardware](#33-switch-to-real-hardware)
- [3.4 Launch and verify](#34-launch-and-verify)
- [3.5 Moving the arm](#35-moving-the-arm)
- [3.6 Stopping](#36-stopping-method-b)

**[PART 4 — Camera: Intel RealSense D455](#part-4--camera-intel-realsense-d455)**
- [4.1 What we are doing and why](#41-what-we-are-doing-and-why)
- [4.2 Physical mount](#42-physical-mount)
- [4.3 One-time install](#43-one-time-install)
- [4.4 Every session — the camera](#44-every-session--the-camera)
- [4.5 Why the reduced settings](#45-why-the-reduced-settings)
- [4.6 Verify the stream](#46-verify-the-stream)
- [4.7 D455 limits](#47-d455-limits--read-before-planning-any-vision-work)
- [4.8 The Python dependency trap](#48-the-python-dependency-trap)

**[PART 5 — Hand tracking and teleoperation](#part-5--hand-tracking-and-teleoperation)**
- [5.0 Two methods](#50-two-methods)
- *Method 1 — joint mapping:*
  - [5.1 Why three stages](#51-why-this-is-built-in-three-stages)
  - [5.2 Phase 1 — the tracker](#52-phase-1--the-tracker)
  - [5.3 Phase 2 — the mapping panel](#53-phase-2--the-mapping-panel)
  - [5.4 Phase 3 — the safety layer](#54-phase-3--the-safety-layer)
  - [5.5 Running the whole pipeline](#55-running-the-whole-pipeline)
  - [5.6 Test in three stages](#56-test-in-three-stages--do-not-skip-stage-2)
- *Method 2 — Cartesian IK:*
  - [5.7 Cartesian hand teleoperation](#57-method-2--cartesian-hand-teleoperation)
  - [5.8 Trackers, modes and tuning](#58-trackers-modes-and-tuning)
    - [The open-loop oscillation, and the fix](#-the-open-loop-oscillation-and-the-fix) ← read before running on real hardware

**[QUICK START — teleoperation in simulation](#quick-start--teleoperation-in-simulation)**

**[PART 6 — Robot parameters](#part-6--robot-parameters)**

**[PART 7 — Status](#part-7--status)**

**[PART 8 — Troubleshooting](#part-8--troubleshooting)**
---
---

# QUICK START — teleoperation in simulation

## The short version — Method 2, two terminals

After attaching the camera (step 0 below) and confirming the xacro is on mock (step 0b):

```bash
# terminal 1 -- the robot
cd ~/fairino_380_ws && source /opt/ros/humble/setup.bash && source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

```bash
# terminal 2 -- camera + vision + control
source /opt/ros/humble/setup.bash
ros2 launch ~/fairino_380_ws/teleop.launch.py deadband:=20.0
```

Then, in the window that opens: **GO TO HOME POSE** → wait for `at home` →
**CAPTURE NEUTRAL POSE** → **CLICK TO START MOVING** → pinch thumb and index, and move
your hand.

In RViz, once: **Add → By topic → `/fr5_workspace` → MarkerArray**, then File → Save
Config. The green sphere is your hand's target.

Two variants worth knowing:

```bash
# whole-arm tracking -- stand back, both shoulders and hips in frame
ros2 launch ~/fairino_380_ws/teleop.launch.py tracker:=pose deadband:=20.0

# on the REAL robot -- hold-to-move, half scale, supervisor present
ros2 launch ~/fairino_380_ws/teleop.launch.py hold:=true scale:=0.5
```

That is Method 2 (Parts 5.7 and 5.8), and it is the one to demonstrate. **Before the
first run on real hardware, read the oscillation note in Part 5.8.** The longer sequence
below is Method 1, which uses five terminals and is kept for comparison.

---

## The long version — Method 1, five terminals

Everything already installed, nothing connected to the real robot. Five terminals
in VS Code (WSL), plus one admin PowerShell. **Every Ubuntu terminal starts with:**

```bash
source /opt/ros/humble/setup.bash
```

Full explanations: camera in Part 4, pipeline in Part 5.

### 0 — Forward the camera into WSL (admin PowerShell)

```powershell
usbipd list
usbipd attach --wsl --busid 2-4
```

Must then read **Attached**.
`Not shared` → `usbipd bind --force --busid 2-4` first.
`no device with busid` → unplugged, or on another port; use the BUSID the list shows.

### 0b — Confirm the config is on the SIMULATED robot

```bash
grep plugin ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

Must print `mock_components/GenericSystem`. If not:

```bash
sed -i 's|fairino_hardware/FairinoHardwareInterface|mock_components/GenericSystem|' \
  ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

### 1 — Camera

```bash
pgrep -af realsense          # MUST be empty; else: pkill -f realsense2_camera_node
lsusb | grep -i intel        # must show the D455

ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x360x15 \
  enable_depth:=false enable_infra1:=false enable_infra2:=false
```

Wait for `RealSense Node Is Up!`, then leave it alone.

### 2 — Tracker

```bash
python3 ~/fairino_380_ws/arm_tracker.py
```

Green skeleton, `RIGHT ARM TRACKED`.

### 3 — Simulated robot

```bash
cd ~/fairino_380_ws
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

RViz opens. **Do not touch Plan or Execute** — teleoperation bypasses MoveIt.

### 4 — Mapping panel

```bash
python3 ~/fairino_380_ws/joint_mapper.py
```

### 5 — Teleoperation

```bash
python3 ~/fairino_380_ws/teleop_node.py --toggle
```

### 6 — Drive it

1. Mapper: **j3** → source **`shoulder_elev`**, gain `1.00`, offset `0.0`.
   Leave the other five on `(none)`.
2. Stand ~2 m back, whole torso in frame.
3. Click **Capture neutral pose**.
4. Raise and lower your arm — the j3 target should move.
5. Teleop status should be green: *ready*. Click **CLICK TO START MOVING**.
6. Raise and lower your arm. The FR5 moves in RViz, lagging behind you — that is
   the 10 °/s rate limit working.

Wrong direction → tick **invert**. Too small → raise **gain**. Twitchy →
restart teleop with `--filter-hz 1.0`.

**Use `shoulder_elev`, not `shoulder_azim`.** Azimuth is meaningless when the arm is
near vertical or hanging down, and jumps around on noise there (Part 5.2).

### Shutdown

`Ctrl+C` each terminal, then optionally:

```powershell
usbipd detach --busid 2-4
```

### The two rules that cause every camera failure

1. **`attach` every session.** It never persists across reboots, WSL restarts or unplugs.
2. **One camera node, ever.** A second gives `errno=16 busy`, which triggers a hardware
   reset and drops the usbipd attachment entirely.



---
---

# PART 0 — MAIN INSTRUCTIONS

**Do these every single time, before any method, before touching a keyboard.**

### 0.1 People

- [ ] **A supervisor or instructor is present.** Not in the building — in the room.
      The FR5 weighs ~22 kg and reaches 922 mm. It can hit you and it can hit itself.
- [ ] Everyone in the room knows the arm is about to be powered.

### 0.2 Power on, in this order

- [ ] **Turn on the power supply.**
- [ ] **Turn on the red switch on the black control box** beside the FAIRINO robot.
- [ ] Wait up to one minute for the controller to boot.
- [ ] **Check the indicator below the gripper is GREEN.**
      Not green → **release the emergency stop switch**, then check again.

### 0.3 Connect

- [ ] **Plug the Ethernet cable** from the FAIRINO emergency-stop box into your laptop.
- [ ] Set the laptop's Ethernet adapter — **in Windows, not in WSL**:

  Settings → Network & Internet → Ethernet → IP assignment → **Edit** → **Manual** →
  IPv4 **On**

  | Field | Value |
  |---|---|
  | IP address | `192.168.58.10` |
  | Subnet mask | `255.255.255.0` |
  | Gateway | *leave blank* |

  `192.168.58.2` is the **controller's** address. Never set your laptop to that.

### 0.4 Safety check before any motion

- [ ] **Physical emergency stop within reach of your hand**, not the robot's
- [ ] Swept volume clear for the **whole arm and tool**, including behind and below
- [ ] Nobody inside the work envelope
- [ ] You know the current joint angles (Part 1.3)
- [ ] Target checked against the joint limits table (Part 6)
- [ ] Speed set to **5 %** — never higher while learning

### 0.5 Two methods — never both at once

Method A and Method B each open their **own RPC connection to the controller on the same
port**. Starting the second while the first runs fails with:

```
机械臂SDK连接失败！请检查端口时候被占用
(SDK connection failed! Check whether the port is occupied)
```

Shut one down completely before starting the other. Check with:

```bash
pgrep -af ros2_cmd_server
```

Nothing printed = clear to start Method B.

### 0.6 Why the robot moves so slowly

Every command in this document uses speed **5**, meaning 5 % of maximum. The FR5 can do
180°/s per axis and 1 m/s at the tool. At 5 % it crawls — which is correct for
commissioning and for learning. **Do not raise it.**

### 0.7 What `cmd_res='0'` actually means

It means *the API accepted the command*. It does **not** prove the arm moved where you
intended. Always verify with feedback afterwards. Never re-send a command because
"nothing happened" — at 5 % it may still be moving.

---
---

# PART 1 — Common setup

Needed by both methods.

## 1.1 One-time install (already done)

Recorded so the project is reproducible from a clean Ubuntu 22.04 install.

**Host environment:** Windows 11 + WSL2 + Ubuntu 22.04 + ROS 2 Humble.
The FAIRINO driver is Linux-only — the vendor ships a compiled `.so` — which is why WSL
exists here at all.

### Install ROS 2 Humble

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop
sudo apt install -y git build-essential cmake python3-colcon-common-extensions \
  python3-rosdep ros-humble-ros2-control ros-humble-ros2-controllers
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Check:** `echo $ROS_DISTRO` prints `humble`.

### Install the Python dependencies

```bash
sudo apt install -y python3-pip python3-tk ros-humble-cv-bridge
cd ~/fairino_380_ws
pip3 install -r requirements.txt
```

**Do not add `--break-system-packages`.** Ubuntu 22.04 ships pip 22.0.2, which does not
have that flag, and 22.04 does not enforce the restriction it exists for. Adding it makes
pip reject the whole command.

**The exact version pins in `requirements.txt` are load-bearing.** ROS 2 Humble is a
numpy-1.x world and current pip wheels have moved to numpy 2; unpinned installs break the
vision stack in several different ways. Part 4.8 documents each failure and why.

### Clone the vendor driver — pinned tag

```bash
mkdir -p ~/fairino_380_ws/src
cd ~/fairino_380_ws/src
git clone --branch V3.0.0_RobotV3.8.0 https://github.com/FAIR-INNOVATION/frcobot_ros2.git
cd frcobot_ros2
git describe --tags --always    # V3.0.0_RobotV3.8.0
git rev-parse --short HEAD      # 7c683cf
```

**Do not `git pull` to main.** This tag matches controller firmware v3.8.0.1.

This one clone gives you three things:

| Package | What it is |
|---|---|
| `fairino_hardware` | The driver. Talks to the controller over Ethernet. |
| `fairino_description` | The 3D model — STL meshes + `fairino5_v6.urdf` |
| `fairino5_v6_moveit2_config` | MoveIt settings, planner config, `demo.launch.py` |

(The repo also ships configs for FR3/10/16/20/30. Use the **fairino5** ones.)

### Build the driver

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fairino_hardware --symlink-install
source install/local_setup.bash
ros2 pkg prefix fairino_hardware
```

Expected: `2 packages finished` (`fairino_msgs`, `fairino_hardware`).

`-Wmaybe-uninitialized` warnings and `1 package had stderr output: fairino_hardware`
are **expected** in this tagged release. Not errors. Do not try to fix them.

## 1.2 Every-session startup

**Nothing below survives closing a terminal. Run it every time.**

### Reconnect VS Code to Ubuntu

`Ctrl+Shift+P` → `WSL: Connect to WSL using Distro` → **Ubuntu-22.04**

Bottom-left must read `WSL: Ubuntu-22.04`. Faster next time: `Ctrl+R` → pick
`fairino_380_ws`.

### Source every new terminal

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
```

Both lines, every terminal. `.bashrc` only handles the first.

**Tip:** VS Code renames a terminal tab to whatever is running in it. A tab labelled
`python3` is usually a running `ros2 launch` — same bash terminal, different label.

### Check the network

```bash
ip route
ping -c 3 192.168.58.2
```

```bash
python3 - <<'PY'
import socket
try:
    with socket.create_connection(("192.168.58.2", 8081), 3):
        print("Feedback TCP port 8081 reachable")
except OSError as error:
    print("Connection failed:", error)
PY
```

**The TCP check matters more than the ping.** WSL2 sometimes drops ICMP while TCP works
fine. TCP success = connected, whatever ping said. Both failing = stop, see Part 8.

### Controller web UI (optional cross-check)

`http://192.168.58.2` — username `admin`, password `123`.
Useful for comparing joint angles against what ROS reports.

## 1.3 Verify communication

What you run here depends on the method, but **the principle is identical and
non-negotiable: confirm ROS and the physical robot agree before commanding anything.**

- Method A reads `/nonrt_state_data`
- Method B reads `/joint_states`

In both cases, convert to degrees and compare against the pendant or the web UI.
**If they disagree, stop.** You are not reading the robot you think you are.

---
---

# PART 2 — METHOD A: Terminal control

Send text commands over a ROS 2 service. This is what the instructor demonstrated.

**Characteristics:** one discrete command at a time, the controller plans and executes it,
then it stops. Natural pause between commands. Lower risk than Method B.

**Underlying API:** `MoveJ` / `MoveL`.

## 2.1 Start the command server

### Terminal A — the server

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
ros2 run fairino_hardware ros2_cmd_server
```

Leave running. `socket create failed` at startup is **expected noise** in this release,
not a failure, as long as the checks below pass.

### Terminal B — commands and checks

Split with `Ctrl+Shift+5`, then source both lines again.

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
```

Expect the topic `/nonrt_state_data` and the service `/fairino_remote_command_service`.

### Terminal C — stop command, ready

Open a third terminal, source both lines, and **type this without pressing Enter**:

```bash
ros2 service call /fairino_remote_command_service fairino_msgs/srv/RemoteCmdInterface "{cmd_str: 'ImmStopJOG()'}"
```

Stopping should be one keypress, not thirty seconds of typing.

### Read the robot state

```bash
timeout 10s ros2 topic echo /nonrt_state_data --once
timeout 10s ros2 topic echo /nonrt_state_data --once --field j1_cur_pos
timeout 10s ros2 topic hz /nonrt_state_data
```

Joint angles here are in **degrees**. Cross-check `j1_cur_pos`…`j6_cur_pos` against the
controller display before going further.

## 2.2 Joint moves (MoveJ)

> Part 0.4 checklist done. Supervisor present. Area clear.

Always **two steps**: register a target (safe — only stores numbers), then move to it.

### Move to home position

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
python3 ~/fairino_380_ws/fairino_home.py
```

Home pose: `JNTPoint(1, 0, -90, 90, -90, -90, 0)`

### Rotate Joint 1 to +90°

**Step 1 — register the target**

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'JNTPoint(2,90,-90,90,-90,-90,0)'}"
```

Expect `cmd_res='0'`.

**Step 2 — check the path is clear, then move**

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'MoveJ(JNT2,5,0,0)'}"
```

**Step 3 — verify**

```bash
ros2 topic echo /nonrt_state_data --once --field j1_cur_pos
```

Should approach 90°.

### Command reference

| Command | Arguments |
|---|---|
| `JNTPoint(id, j1, j2, j3, j4, j5, j6)` | target id, then all six angles in **degrees** — always all six |
| `MoveJ(target, speed, tool, user)` | `JNT2` = registered target 2, `5` = 5 % speed, `0` = tool number, `0` = user/workpiece frame |

**`MoveJ` does not move the tool in a straight line.** It interpolates in joint space, so
the tool path curves. A `MoveJ` that looks fine on paper can swing an elbow into
something.

## 2.3 Linear moves (MoveL)

**Step 1 — read the current pose so you know your starting point**

```bash
ros2 topic echo /nonrt_state_data --once
```

**Step 2 — register a Cartesian target.** All six values required: XYZ position and ABC
orientation. This example moves 50 mm in −X from a pose near X = −182.799 mm:

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'CARTPoint(1,-232.799,-489.828,619.715,73.053,-41.118,61.738)'}"
```

**Step 3 — check what got stored**

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'GET(CART,1)'}"
```

**Step 4 — execute**

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'MoveL(CART1,5,0,0)'}"
```

`MoveL` moves the tool in a **straight line**.

## 2.4 Jogging

Small incremental motion, one joint at a time.

```
StartJOG(ref, joint, direction, speed, acceleration, distance)
```

| Argument | Meaning |
|---|---|
| `ref = 0` | Joint-space jog |
| `joint = 1` | Joint 1 |
| `direction = 1` | Positive joint direction |
| `speed = 5` | 5 % speed |
| `acceleration = 5` | 5 % acceleration |
| `distance = 1` | Max 1° displacement |

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'StartJOG(0,1,1,5,5,1)'}"
```

**Send once.** Observe. Compare feedback before and after. Record the starting angle
first so you can return.

Argument order is for the V3.8.0 C++ SDK wrapper; other bindings differ.

### Joint directions (recorded in the lab)

| Joint | Positive direction |
|---|---|
| J1 | Anticlockwise |
| J2 | Anticlockwise |
| J3 | Anticlockwise |
| J4 | **Clockwise** |
| J5 | *(not recorded)* |
| J6 | **Clockwise** |

A wrong sign sends the arm the opposite way from what you pictured. This is the easiest
way to crash it.

## 2.5 Stopping (Method A)

**The physical e-stop is the real stop.** Software stops travel over the same link that
may be the problem.

Immediate stop:

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'ImmStopJOG()'}"
```

Decelerating stop for a joint jog:

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'StopJOG(1)'}"
```

---
---

# PART 3 — METHOD B: RViz + MoveIt control

Plan motions graphically in a 3D view and execute them.

**RViz is the window. MoveIt is the planner behind it.** They are one method together,
not two.

**Characteristics:** MoveIt streams joint targets to `ServoJ` at 100 Hz and the arm
tracks them continuously. **There is no per-command stop.** A trajectory that clips an
obstacle executes smoothly into it.

**This is the higher-risk mode. Supervisor presence is not optional here.**

Also: **RViz shows no obstacles unless you add them.** The planning scene is empty by
default, so MoveIt considers the entire workspace free — table, mount, cables, you.

## 3.1 Install MoveIt and build

**Install first, then build.** Building before MoveIt is installed will fail.

```bash
sudo apt install -y ros-humble-moveit
```

Large download, ~10 minutes.

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fairino_description fairino5_v6_moveit2_config --symlink-install
source install/local_setup.bash
ros2 pkg prefix fairino5_v6_moveit2_config
```

Last line must print a path ending `/install/fairino5_v6_moveit2_config`.

## 3.2 Test in simulation first

```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

RViz opens with the FR5 model. Drag the marker, Plan, Execute.

**This is a fake arm** — the config ships with `mock_components/GenericSystem`, and the
controller is not involved at all. Confirm the workflow works here before touching real
hardware. `Ctrl+C` to stop.

**This repository is committed in the mock state on purpose.** A fresh clone cannot
accidentally drive a physical robot — switching to real hardware is a deliberate,
explicit step (3.3). If you switch to real hardware, **switch it back before
committing.**

**Why this step matters:** everything you learn about the RViz interface, you learn
safely. Only the plugin line changes afterwards.

## 3.3 Switch to real hardware

One line changes. From the vendor's own file:

```xml
<hardware>
    <!-- By default, set up controllers for simulation. This won't work on real hardware -->
    <plugin>mock_components/GenericSystem</plugin>
</hardware>
```

**To switch to the real robot:**

```bash
sed -i 's|mock_components/GenericSystem|fairino_hardware/FairinoHardwareInterface|' \
  ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

**To switch back to simulation:**

```bash
sed -i 's|fairino_hardware/FairinoHardwareInterface|mock_components/GenericSystem|' \
  ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

**Always confirm which mode you are in** — there is no indication in RViz itself:

```bash
grep plugin ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

Because the workspace was built with `--symlink-install`, config files are symlinked and
the edit takes effect immediately. **No rebuild needed.**

### Why this works

Everything else in the chain is already wired correctly by the vendor:

```
MoveIt → joint_trajectory_controller → FairinoHardwareInterface → ServoJ → arm
```

- `fairino_hardware.xml` declares the real plugin as a `SystemInterface`
- It exports one position command + one position state per joint
- `ros2_controllers.yaml` already configures `fairino5_controller` as a
  `joint_trajectory_controller`
- `moveit_controllers.yaml` already points MoveIt at it via `FollowJointTrajectory`
- `read()` calls `GetActualJointPosDegree`, converts degrees → radians
- `write()` calls `ServoJ`

Only the plugin line was missing.

## 3.4 Launch and verify

**Part 0 complete. Nothing else connected:**

```bash
pgrep -af ros2_cmd_server      # must print nothing
```

### Terminal A — launch

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

### Read the startup log — do not skip

**1. SDK connected**

```
机械臂SDK连接成功！              (SDK connection successful)
```

Instead seeing `机械臂SDK连接失败！请检查端口时候被占用` means the port is occupied —
something else holds the connection. See Part 0.5.

**2. Initial command position**

```
初始指令位置: <six numbers in radians>
机械臂硬件启动成功!              (hardware started successfully)
```

**These six numbers must match where the arm physically is.** Multiply by 57.2958 for
degrees and compare against the pendant.

Seeing `读取初始关节角度错误，硬件无法启动！` means the read failed and the plugin
**refused to activate**. This is **correct behaviour, not a bug to work around.** The
vendor's comment says an incorrect feedback position would cause a severe deviation and
an accident. Fix the communication; never force it.

### Terminal B — verify

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
ros2 control list_hardware_components
ros2 control list_controllers
```

`list_hardware_components` must show:

```
plugin name: fairino_hardware/FairinoHardwareInterface
state: id=3 label=active
```

**`active` is the key word.** That plugin cannot reach `active` unless it successfully
connected to the controller and read the real joint angles — it returns an error
otherwise. So `active` = genuinely connected to the physical robot.

(Ignore `name: FakeSystem`. That is a leftover label in the xacro that nobody renamed.
The `plugin name` line is what matters.)

`list_controllers` must show both as `active`:

```
fairino5_controller        joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster    joint_state_broadcaster/JointStateBroadcaster          active
```

### Confirm the model matches reality

```bash
ros2 topic echo /joint_states --once
```

**⚠ The name array is NOT in numerical order.** It reads `j1, j2, j4, j5, j3, j6`, and
positions follow *that* order. Matching by index naively swaps j3 and j4.

Worked example from a real reading at home position:

| Joint | Radians | Degrees |
|---|---|---|
| j1 | 0.0000759 | ≈ 0° |
| j2 | −1.570762 | ≈ −90° |
| j3 | +1.571104 | ≈ +90° |
| j4 | −1.570728 | ≈ −90° |
| j5 | −1.573029 | ≈ −90.1° |
| j6 | 0.0000456 | ≈ 0° |

= `0, −90, 90, −90, −90, 0` — the home position. Correct.

`velocity: .nan` and `effort: .nan` are **expected** — the vendor's hardware interface
only exports position; velocity and effort are commented out in their source.

`A message was lost!!!` once at startup is a DDS notice from `--once` attaching
mid-stream. Harmless. Constantly during motion is worth investigating.

**If RViz's pose does not match the physical arm, `Ctrl+C` immediately.**

### Terminal C — stop command, ready

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
```

Then **type without pressing Enter**:

```bash
ros2 control switch_controllers --deactivate fairino5_controller
```

## 3.5 Moving the arm

### Set the speed sliders — every launch

MotionPlanning panel → **Planning** tab → *Options*:

- **Velocity Scaling: 0.05**
- **Accel. Scaling: 0.05**

**They default to 1.0, which is full speed.** They reset on every launch. This is the
single most important thing to change before your first Plan.

### Use the Joints tab for your first moves

MotionPlanning panel → **Joints** tab → six sliders, one per joint.

**Safer than dragging the orange marker.** The marker makes MoveIt solve inverse
kinematics, which can return a solution that swings the elbow somewhere you did not
expect. A joint slider changes exactly what you tell it.

Move **one** slider — j1 — by about **5 degrees**. Nothing else.

**The real arm should not move while you drag.** You are only setting the goal. If it
twitches, stop — something is wrong.

### Plan, inspect, execute

1. **Planning** tab → **Plan** (**not** "Plan & Execute")
2. An orange ghost arm animates the proposed path. **Watch the whole thing.**
3. If the ghost does anything unexpected, press Plan again for a different solution.
   IK has multiple solutions; the tool reaching the right place does not mean the arm
   took a sane route.
4. Hand on the physical e-stop. Terminal C visible.
5. **Execute**

The gap between Plan and Execute is where you catch a bad trajectory. That is the whole
reason not to use "Plan & Execute".

### Verify

```bash
ros2 topic echo /joint_states --once
```

j1 should have changed by ~0.087 rad (5°). Everything else unchanged.

Do several small joint moves before going near the drag marker.

## 3.6 Stopping (Method B)

**Physical e-stop is the real stop.**

| Method | Command / action | Notes |
|---|---|---|
| RViz | **Stop** button, MotionPlanning panel | fastest in-GUI option |
| Deactivate controller | `ros2 control switch_controllers --deactivate fairino5_controller` | MoveIt can no longer command; arm holds position |
| Kill the launch | `Ctrl+C` in Terminal A | stops the stream; slower, depends on network |

**Deactivating the controller is NOT the same as making the robot safe to touch.** The
hardware interface stays active and holds the connection; a re-activation can move it.
**Before putting hands on the robot, press the physical e-stop.**

To resume after deactivating (release the e-stop first):

```bash
ros2 control switch_controllers --activate fairino5_controller
```

Then re-check `/joint_states` matches the arm — if it was pushed by hand while free,
RViz's idea of where it is will be stale.

### A known cosmetic crash on exit

On `Ctrl+C` you may see a wall of red including:

```
class_loader: SEVERE WARNING!!! Attempting to unload library...
move_group: Segmentation fault
[ERROR] [move_group-2]: process has died ... exit code -11
```

This is a long-standing MoveIt bug during shutdown. It crashes *while tearing itself
down*, after the work is done. **What matters is the lines above it:**

```
Successful 'deactivate' ... 'shutdown' hardware ...
ros2_control_node: process has finished cleanly
```

That is the robot being released in the right order. Nothing is damaged.

---
---

# PART 4 — Camera: Intel RealSense D455

## 4.1 What we are doing and why

The camera watches the operator so that human arm movement can drive the robot (Part 5).

**The camera plugs into the laptop by USB. The robot connects by Ethernet. Two completely
separate paths** — the robot does not know the camera exists.

```
Laptop ─── Ethernet ──→ FR5 controller   (commands)
Laptop ─── USB ───────→ RealSense D455   (video)
```

**The obstacle:** WSL2 has no USB support by default. Windows owns the USB device and
Ubuntu-inside-WSL cannot see it. `usbipd-win` (free, open source) forwards it in over a
virtual connection.

## 4.2 Physical mount

Mounted on `wrist3_link` (the last link), lens pointing the same way as the tool.

- Cable routed with slack, taped along the forearm
- Must survive j6 rotating ±175° without snagging or pinching
- **A yanked cable mid-motion drops the camera and needs a re-attach (4.5)**

Tape is fine for a test, but the camera's pose relative to the flange is not repeatable,
so any coordinate work will drift when it shifts. Permanent use needs a rigid mount and a
URDF link.

## 4.3 One-time install

### Windows

Admin PowerShell (Win+X → Terminal (Admin)):

```powershell
winget install --exact dorssel.usbipd-win
```

**Then close PowerShell and open a new admin window.** The installer adds `usbipd` to
PATH, and an already-open window still has the old PATH. This is why `usbipd list` fails
the first time with "not recognized".

Plug the camera in, then:

```powershell
usbipd list
usbipd bind --busid 2-4
```

Use whatever BUSID the RealSense shows — it changes with the USB port.

If a later `attach` says **"Device busy (exported) ... appears to be used by Windows"**,
take it from the Windows driver with:

```powershell
usbipd bind --force --busid 2-4
```

### Ubuntu

```bash
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description \
  ros-humble-cv-bridge python3-pip python3-tk usbutils
cd ~/fairino_380_ws
pip3 install -r requirements.txt
```

**Do not add `--break-system-packages`.** Ubuntu 22.04 ships pip 22.0.2, which does not
have that flag, and 22.04 does not enforce the restriction it exists for. The flag makes
pip reject the whole command.

**The version pins in `requirements.txt` are not optional** — see 4.8.

## 4.4 Every session — the camera

**1. Plug the RealSense into the laptop.**

**2. Check nothing is already running:**

```bash
pgrep -af realsense
```

Must print nothing. If it prints something, `pkill -f realsense2_camera_node`.

**3. Attach it.** Admin PowerShell:

```powershell
usbipd list
usbipd attach --wsl --busid 2-4
```

Confirm the row then reads **Attached**, not `Shared` or `Not shared`.

**`bind` is once. `attach` is every time** you unplug the camera, reboot, restart WSL, or
after any failure that resets the device.

**4. Confirm Ubuntu sees it:**

```bash
lsusb | grep -i intel
ls -la /dev/video*
```

Expect `Intel(R) RealSense(TM) Depth Camera 455` and `/dev/video0` through `/dev/video5`.
Nothing → go back to step 3.

**5. Launch — once, and only once:**

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x360x15 \
  enable_depth:=false enable_infra1:=false enable_infra2:=false
```

Wait for `RealSense Node Is Up!`. Leave it running.

> **One camera node, ever.** Starting a second while the first runs gives
> `VIDIOC_S_FMT failed, errno=16 Device or resource busy`, then a hardware reset, then
> `The device has been disconnected!` — and the usbipd attachment drops with it. Recovery
> is `pkill`, re-attach, relaunch. Always `pgrep -af realsense` before launching.

## 4.5 Why the reduced settings

**Do not use the default launch.** The D455 defaults to depth 848×480×30 and colour
1280×720×30, far more than the virtual USB connection can carry:

```
XXX Hardware Notification:Incomplete video frame detected!
Size 541448 out of 814335 bytes (66%),Warn,Frame Corrupted
```

Frames arrive half-finished and it gets worse over time. Depth is disabled above because
the tracker only uses colour; less bandwidth, fewer dropped frames.

If you need depth, start small:

```bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=480x270x15 \
  rgb_camera.color_profile:=640x360x15 \
  enable_infra1:=false enable_infra2:=false
```

## 4.6 Verify the stream

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/camera/color/image_raw
```

Expect a steady ~15 Hz:

```
average rate: 14.506
  min: 0.053s max: 3.105s std dev: 0.13135s window: 4045
```

The topic is `/camera/camera/...` — the node nests the namespace.

An occasional large `max` is a startup hiccup. Repeated large gaps during arm motion mean
the USB cable is being tugged.

## 4.7 D455 limits — read before planning any vision work

| | |
|---|---|
| Ideal depth range | 0.6 m – 6 m |
| **Minimum depth distance** | **~0.52 m** |
| Depth accuracy | <2 % error at 4 m |
| Depth FOV | 87° × 58° |
| RGB FOV | 90° × 65° |
| Stereo baseline | 95 mm |

**The 52 cm minimum matters here.** The arm's reach is 922 mm, so with the camera on the
wrist, anything near the tool sits inside the blind zone — depth returns holes and zeros.
Wrist-mounted depth works for things reasonably far from the gripper, not close-up
grasping. **Colour works at any distance**; only depth has the minimum.

**The IMU does not work over WSL.** `No HID info provided, IMU is disabled` — USB/IP does
not pass the HID interface through. No workaround in WSL.

## 4.8 The Python dependency trap

ROS 2 Humble is from 2022 and its apt packages (`matplotlib`, `cv_bridge`, `python3-opencv`)
are all built against **numpy 1.x**. Current pip wheels have moved to **numpy 2**. The two
cannot coexist, and `pip3 install <name>` with no version always pulls the numpy-2 side.

Symptoms, in the order they appear if you let it happen:

| Error | Cause |
|---|---|
| `no such option: --break-system-packages` | pip 22.0.2 on Ubuntu 22.04 predates the flag |
| `AttributeError: _ARRAY_API not found` | numpy 2 loaded against a numpy-1.x C extension |
| `mediapipe is not installed` (but it is) | the import failed on the above, misreported |
| `opencv-contrib-python 5 requires numpy>=2` | opencv 5 is numpy-2 only |
| `module 'mediapipe' has no attribute 'solutions'` | mediapipe 1.x **deleted** the Solutions API |

**Every `pip install` re-resolves dependencies and can silently upgrade numpy again** —
installing mediapipe pulls in jax and scipy, which drag numpy 2 back. That is why
`requirements.txt` pins exact versions, and why numpy must be pinned **last** if you ever
install by hand.

The working set:

```
numpy==1.26.4
opencv-contrib-python==4.10.0.84
mediapipe==0.10.14
```

`mediapipe==0.10.14` is the last release with the `mp.solutions` API that `arm_tracker.py`
uses. Moving to mediapipe 1.x would require rewriting the tracker for the Tasks API plus
downloading a separate `.task` model file.

**On a slow connection**, mediapipe is ~35 MB and pip gives up quickly:

```bash
pip3 install --default-timeout=120 --retries 10 -r requirements.txt
```

---
---

# PART 5 — Hand tracking and teleoperation

**Goal:** the operator's right arm is tracked by the camera and drives the robot's joints
in real time.

```
your arm → camera → arm_tracker.py  → /human_arm_angles
                  → joint_mapper.py → /fr5_joint_targets
                  → teleop_node.py  → SAFETY → /fairino5_controller/joint_trajectory → robot
```

## 5.0 Two methods

There are two working teleoperation methods in this repository. **Both are kept** — they
take different approaches to the same problem and are worth comparing.

| | **Method 1 — joint mapping** (5.1 – 5.6) | **Method 2 — Cartesian IK** (5.7) |
|---|---|---|
| Input | your shoulder and elbow **angles** | your hand **position** in 3D |
| Output | chosen joints driven directly | all six joints, solved by IK |
| Config | 6 dropdowns, gains, offsets | 3 axis choices and a scale |
| Floor protection | not possible | one line (`z_min`) |
| Uses the FK/IK work | no | yes, centrally |
| Processes | 3 (+ camera) | 1 (+ camera, one launch file) |
| Scripts | `arm_tracker.py`, `joint_mapper.py`, `teleop_node.py` | `hand_teleop.py`, `teleop.launch.py` |

**Method 2 is the better one** and the one to demonstrate. Method 1 is kept because it is
simpler to explain, it is what the pipeline was built up from, and the failure that
motivated Method 2 (5.7) is a genuinely instructive result.

The safety chain — deadman, rate limiting, smoothing, clamping, freeze on tracking loss —
is the same in both.

## 5.1 Why this is built in three stages

Pose estimation jitters. A landmark flickers a few pixels, the computed elbow angle jumps
15°, and that goes straight to `ServoJ` on a 22 kg arm. **The safety layer is not a
finishing touch — it is the project.**

| Phase | Script | What it does | Robot? |
|---|---|---|---|
| 1 | `arm_tracker.py` | track the arm, compute angles | no |
| 2 | `joint_mapper.py` | map human angles → robot joints | no |
| 3 | `teleop_node.py` | safety layer, drive the robot | yes |

Each phase runs as its own node and talks over ROS topics, so you can run 1 and 2 with no
robot anywhere, and 3 against a **simulated** robot before any hardware is involved.

## 5.2 Phase 1 — the tracker

```bash
source /opt/ros/humble/setup.bash
python3 ~/fairino_380_ws/arm_tracker.py
```

Options:

```bash
--source webcam          # a plain webcam instead of the RealSense
--width 1920             # bigger window (default 1280, 0 = native)
--no-mirror              # see yourself as others see you
--model-complexity 2     # slower, more accurate
--min-visibility 0.5     # looser landmark confidence threshold
```

`q` or `Esc` to quit.

**Stand back ~2 m so your whole torso is in frame.** It needs both shoulders *and both
hips* to work out which way you are facing.

### What it measures

MediaPipe Pose finds 33 body landmarks. We use the right shoulder (12), elbow (14) and
wrist (16), plus the left shoulder and both hips to build a reference frame.

| Angle | 0° | 90° | 180° |
|---|---|---|---|
| **shoulder_elev** | arm hanging straight down at your side | horizontal | straight up |
| **shoulder_azim** | pointing straight out to your right | reaching forward | — (−90° = back) |
| **elbow_flex** | fully folded, hand at shoulder | right angle | arm completely straight |
| **wrist_roll** | arbitrary reference — unreliable, see below | | |

**There is no pose where all four read zero.** Each angle has its own zero and they do not
coincide. The one that surprises people is the elbow: **a straight arm is 180, not 0.**
That is the anatomical convention — the angle *at* the joint. If you want "straight = 0",
compute `180 − elbow_flex`.

**Azimuth is meaningless when the arm hangs down.** The arm points almost straight down, so
its left/right and forward/back components are tiny and the number jumps on noise. Do not
map azimuth to a robot joint if you will be working with your arm low.

### Three design decisions that matter

**Detection runs on the unmirrored frame.** MediaPipe labels landmarks by the anatomy it
sees, so running it on a mirrored image makes it call the operator's **left** arm "right".
The script mirrors only the picture you see, never the picture it measures. Getting this
wrong silently tracks the wrong arm and inverts the whole mapping.

**Angles are computed in a frame attached to your torso, not the camera.** The script
builds x (out to your right), y (up your spine), z (forward from your chest) from the
shoulder and hip landmarks. Turning or stepping sideways does not change the numbers —
only moving your arm does. Without this, walking across the room would command the robot.

**Lost tracking publishes `NaN`, not 0.** A joint that cannot be seen confidently goes red
and reads `---`. Phase 3 treats `NaN` as *freeze*; a zero would be a real command to a
real pose.

### Output

`/human_arm_angles`, `Float32MultiArray`:

```
[shoulder_elev, shoulder_azim, elbow_flex, wrist_roll, tracking_ok]
```

### The wrist problem

**MediaPipe's body model cannot see forearm rotation reliably.** It is inferred from a few
hand landmarks and is noisy. Shoulder elevation, shoulder azimuth and elbow flexion are
solid; wrist roll is not.

The brief asks for three joints — shoulder, elbow, wrist — and the third is the weak link.
Either map the two shoulder angles plus elbow, or drive the wrist from something else.
**Worth raising with the instructor.**

## 5.3 Phase 2 — the mapping panel

```bash
source /opt/ros/humble/setup.bash
python3 ~/fairino_380_ws/joint_mapper.py
```

A GUI with the live human angles across the top and six rows, j1 to j6.

Per row:

| Control | What it does |
|---|---|
| **source** | which human angle drives this joint, or `(none)` |
| **invert** | flip the direction |
| **gain** | robot degrees per human degree |
| **offset** | where the joint sits when the input is zero |
| **target** | the computed result, live |

The target column colour-codes itself:

- **green** — inside conservative working limits
- **orange** — outside those but still legal
- **red CLAMPED** — hit an FR5 soft limit and was cut off
- **red tracking lost** — that landmark is not visible

Limits come from the FR5 spec, **not** the URDF — so j3 caps at ±150°, not the wrong
±162° the vendor ships (Part 6.3).

### Capture neutral pose

Stand comfortably, press the button, and the mapping becomes *change from that pose*
rather than absolute angle. "Arm relaxed at my side" maps to "robot at home" instead of
some arbitrary pose.

**This is the safer design** and it is what Phase 3 expects. It also sidesteps having to
memorise which angle has which zero.

### Choosing a mapping

Mechanically any human angle can drive any robot joint. Some feel natural and some do not.
Starting points worth trying:

| Human | Robot | Why |
|---|---|---|
| shoulder_azim | j1 | both rotate the whole arm about a vertical axis |
| shoulder_elev | j2 or j3 | both raise and lower the arm |
| elbow_flex | j3 | both are the elbow |

Save/Load writes `joint_mapping.json` next to the scripts.

**Output:** `/fr5_joint_targets` — six joint targets in degrees plus a validity flag.
Nothing consumes it until Phase 3 runs.

## 5.4 Phase 3 — the safety layer

```bash
source /opt/ros/humble/setup.bash
python3 ~/fairino_380_ws/teleop_node.py --dry-run
```

Options:

```bash
--dry-run                # compute everything, publish nothing
--max-speed 5            # degrees per second per joint (default 10, max 45)
--filter-hz 1.0          # lower = smoother but laggier (default 2.0)
--tolerance 10           # max gap allowed when engaging (default 15 deg)
```

### The safety chain

```
/fr5_joint_targets
    |
    +-- validity check      NaN / tracking lost / stale -> FREEZE
    +-- clamp               to the FR5 SOFT limits
    +-- low-pass filter     absorbs pose-estimation jitter
    +-- slew-rate limit     caps degrees per second, whatever you do
    +-- deadman             motion only while the button is held
    |
/fairino5_controller/joint_trajectory
```

| Guard | What it stops |
|---|---|
| **Deadman button** | hold the mouse down to move; release = stop, instantly |
| **Slew-rate limit** | caps every joint at `--max-speed` °/s no matter how fast you move |
| **Low-pass filter** | pose jitter never becomes robot shake |
| **Clamp** | FR5 soft limits, applied after the mapping |
| **Engage gate** | refuses to start if a target is more than `--tolerance` from the real joint angle, so it cannot lurch on the first command |
| **Freeze on loss** | tracking lost, `NaN`, or stale data → stops and holds position |
| **Starts from actual** | on engage the command begins at the real joint angles, not the target |

**Verified behaviour:** fed an instantaneous 60° step with `--max-speed 10`, the command
takes **6 seconds** to arrive, moving at exactly 10 °/s throughout. The operator's arm can
move as fast as it likes; the robot cannot.

### Hold mode vs toggle mode

**Default is hold** — press and keep holding the button; release stops. This is the
safer design: letting go is the natural reaction if something goes wrong, and it cannot
be left on by accident.

**`--toggle`** gives click-on / click-off instead. Easier when you need both hands free
or want to watch RViz without worrying about the pointer. A separate red **STOP** button
appears beside it.

**`Esc` or `Space` stops immediately in either mode.**

A note on why hold mode is bound the way it is: it deliberately does *not* disengage when
the pointer leaves the button. An earlier version did, and nudging the mouse a few pixels
while watching RViz silently dropped out of motion. It does still stop on window focus
loss, which is intentional — alt-tabbing away should not leave the robot live.

For stage 3 on real hardware, prefer hold mode unless you have a specific reason not to.

## 5.5 Running the whole pipeline

Four terminals. **Each needs `source /opt/ros/humble/setup.bash` first, and each process
is started once.**

```bash
# 1 — camera
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x360x15 \
  enable_depth:=false enable_infra1:=false enable_infra2:=false

# 2 — vision
python3 ~/fairino_380_ws/arm_tracker.py

# 3 — mapping panel
python3 ~/fairino_380_ws/joint_mapper.py

# 4 — teleoperation
python3 ~/fairino_380_ws/teleop_node.py --dry-run
```

## 5.6 Test in three stages — do not skip stage 2

### Stage 1 — no robot at all

Run the four terminals above with `--dry-run`. Assign a source to j1 in the mapper, move
your arm, and watch the target and commanded columns. Nothing is published anywhere.

**What you are checking:** the mapping does what you expect, and the numbers are stable
when you hold still.

### Stage 2 — the simulated robot

**This is the important one, and it is completely safe.** Put the config back to
simulation:

```bash
sed -i 's|fairino_hardware/FairinoHardwareInterface|mock_components/GenericSystem|' \
  ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro

grep plugin ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

Confirm it reads `mock_components/GenericSystem`, then:

```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

Now run `teleop_node.py` **without** `--dry-run`. **Your arm drives the FR5 in RViz.**
Fake robot, real everything else — the mapping, the directions, the speed limits, the
deadman, the freeze-on-loss all get proven with nothing that can hurt anyone.

**Expect to redo the mapping several times here.** That is the point of this stage.

Do not move to stage 3 until the arm in RViz does what you intend, every time.

### Stage 3 — the real robot

Only after stage 2 feels correct, and only with:

- [ ] A supervisor present
- [ ] Hand on the physical e-stop
- [ ] Part 0 completed
- [ ] `--max-speed` still at its default or lower
- [ ] Workspace clear for the whole arm

Switch the plugin to the real hardware interface (Part 3.3), launch `demo.launch.py`,
verify as in Part 3.4, then run `teleop_node.py`.

**Remember the planning scene is empty** — MoveIt and this node know nothing about the
table, the mount, the cables or you. Collision checking is not protecting you here.
---

## 5.7 METHOD 2 — Cartesian hand teleoperation

**One process, one window.** `hand_teleop.py` replaces `arm_tracker.py` +
`joint_mapper.py` + `teleop_node.py`, and `teleop.launch.py` starts it together with the
camera.

```bash
# terminal 1 -- the robot
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py

# terminal 2 -- camera + vision + control
source /opt/ros/humble/setup.bash
ros2 launch ~/fairino_380_ws/teleop.launch.py
```

Launch arguments:

```bash
ros2 launch ~/fairino_380_ws/teleop.launch.py \
    dry_run:=true       # compute everything, publish nothing
    hold:=true          # hold-to-move instead of click-to-toggle
    scale:=1.5          # hand motion amplification
    max_speed:=5.0      # degrees per second per joint
    filter_hz:=1.0      # lower is smoother, laggier
```

Or run the script directly: `python3 ~/fairino_380_ws/hand_teleop.py --dry-run`

### Why Cartesian instead of joint-to-joint

Method 1 maps your joint angles to the robot's joint angles. That works, but:

- your arm and the FR5 are not the same shape, so every mapping is arbitrary
- each joint needs a hand-tuned offset, gain and direction
- **it has no idea where the tool actually ends up**

That last point is not theoretical. During testing, j2 and j3 were both mapped to
`shoulder_elev` with offset 0. Raising an arm commanded j2 to +150°, which the clamp cut
to its +85° soft limit — a 175° swing from home. The arm went into the floor with every
joint still legally inside its limits. **Joint limits do not know where the floor is.**

Working in Cartesian space fixes both problems:

| | Method 1 (joint) | Method 2 (Cartesian) |
|---|---|---|
| Input | 3 human joint angles | hand position in 3D |
| Configuration | 6 dropdowns, 6 gains, 6 offsets | 3 axis choices and a scale |
| Floor protection | not possible | one line: `z_min` |
| Uses the IK | no | yes — it is the core of it |
| 3-DOF → 6-DOF | 3 joints driven, 3 fixed | fully determined by IK |
| Processes | 3 | 1 |

### The 3-DOF to 6-DOF problem

MediaPipe reliably gives three usable numbers from an arm: two at the shoulder and one at
the elbow. The FR5 has six joints. Method 1 answers this by driving three joints and
leaving three alone.

Method 2 answers it properly. **Three numbers of hand position, plus a fixed tool
orientation, make a complete 6-DOF pose** — three for position, three for orientation.
Inverse kinematics then returns all six joint angles. Nothing is left unused and nothing
is arbitrary.

The orientation is whatever the robot had when you pressed **Capture neutral**. That
guarantees the starting pose is reachable, and means you never have to specify an
orientation by hand.

### The chain, step by step

```
camera -> MediaPipe -> torso frame -> offset from neutral -> axis map + scale
       -> offset clamp -> workspace clamp -> IK -> flip check
       -> low-pass -> slew limit -> joint clamp -> controller
```

| Step | What happens |
|---|---|
| **1. Image** | `/camera/camera/color/image_raw` at 15 Hz, converted by `cv_bridge` |
| **2. Landmarks** | MediaPipe Pose returns 33 landmarks; `pose_world_landmarks` are metric 3D positions in metres |
| **3. Torso frame** | built from both shoulders and both hips: x right, y up the spine, z forward from the chest. The wrist is expressed in this frame, so turning or stepping sideways commands nothing |
| **4. Offset** | hand position minus the position captured at neutral. Relative, so there is nothing to calibrate |
| **5. Axis map** | right/up/forward assigned to robot X/Y/Z, inverted as configured, times scale, metres → mm, added to the neutral tool position |
| **6. Offset clamp** | ±300 mm per axis from neutral, keeping motion local |
| **7. Workspace clamp** | absolute limits: `z > 180`, `z < 900`, `250 < r < 870` |
| **8. IK** | target point + fixed orientation → `fr5_kinematics.ik()` → up to 8 branches, filtered to the FR5 soft limits, nearest to the current pose chosen |
| **9. Flip check** | compared to the PREVIOUS frame's solution; a jump over 30° means IK switched branch, so hold |
| **10. Low-pass** | absorbs pose-estimation jitter |
| **11. Slew limit** | hard cap of `max_speed` °/s per joint |
| **12. Publish** | `JointTrajectory` on `/fairino5_controller/joint_trajectory` |

### Operating it — three buttons, in order

**0. GO TO HOME POSE** — drives the arm to `0, −90, 90, −90, −90, 0` through the same
rate limiter. Wait for `at home`.

**1. CAPTURE NEUTRAL POSE** — records your hand position and the robot's tool pose.

**2. CLICK TO START MOVING** — engages. Click again, press `Esc`, `Space`, or the red
STOP button to stop.

**Always home before capturing.** The home pose puts the tool 696 mm from the base at
z = 477: 174 mm of reach headroom and 297 mm above the floor. Capturing from a
stretched-out pose leaves nothing. In testing, a neutral captured at r = 817, z = 180 sat
*exactly* on the floor limit with 53 mm of reach left, and every direction hit a limit
immediately. The panel now warns if you capture within 120 mm of a limit.

### Which hand motion does what

| Your hand | Robot |
|---|---|
| toward the camera | arm extends outward |
| away from the camera | arm folds back toward its base |
| up / down | tool up / down |
| left / right | tool swings sideways |

**The arm reaches along NEGATIVE x.** The home tool position is x = −497, so "extend
outward" means *more negative* x. That is why robot X is inverted by default.

### Reliability of each axis

**Up/down and left/right are solid.** They come almost directly from pixel positions.

**Forward/back is the weak one.** MediaPipe infers depth from a single colour image —
apparent limb foreshortening — and it is substantially noisier than the other two. Test
in the order up/down, then left/right, then forward/back.

**The proper fix, if forward/back is not good enough:** the D455 has a real depth sensor,
which Part 4.5 turns off to save USB bandwidth. Re-enabling it at low resolution and
reading the true distance to the wrist would replace the estimate with a measurement.
Roughly twenty lines of code plus a bandwidth gamble.

### What is being positioned

The **tool flange** — the end of the last link, where a gripper or the camera bolts on.
The DH chain runs base → six joints → flange, the final 100 mm being d₆ past the wrist
joint.

Your wrist is the robot's flange. The robot works out how to bend its arm to put it
there. That is what inverse kinematics means, doing real work rather than sitting in a
report.

### Controls reference

| Control | Effect |
|---|---|
| **axis dropdowns** | which of right/up/forward drives robot X, Y, Z |
| **invert** | flip that axis |
| **scale** | hand motion amplification, 0.5 – 3.0 |
| `--max-speed` | °/s per joint, default 10 |
| `--filter-hz` | lower is smoother and laggier, default 2.0 |
| `--max-offset` | mm from neutral per axis, default 300 |
| `--max-jump` | degrees before a solution counts as a flip, default 30 |
| `Esc` / `Space` | stop, from anywhere in the window |

### Tuning

| Symptom | Fix |
|---|---|
| an axis goes backwards | tick its **invert** |
| movement too small | raise **scale** |
| twitchy | `filter_hz:=1.0` |
| can't keep up | `max_speed:=15` |
| `LIMIT: reach` constantly | press **GO TO HOME** and recapture |
| `LIMIT: floor` | you are reaching too low; recapture higher |
| `LIMIT: offset X/Y/Z` | past ±300 mm from neutral — recapture, or raise `--max-offset` |
| `held: that would flip the arm configuration` | the pose is near a configuration boundary; move back and approach differently |
| `no IK solution for that position` | geometrically unreachable with the captured orientation |

### Testing stages

Same three stages as Method 1 (Part 5.6), and stage 2 is still not skippable:

1. `dry_run:=true` — numbers only, nothing published
2. **Simulated robot** — xacro on `mock_components`, `demo.launch.py`, drive the arm in
   RViz
3. **Real robot** — supervisor present, hand on the physical e-stop

---
---
---

## 5.8 Trackers, modes and tuning

Method 2 grew a lot during testing. This section is the reference for every option and
why it exists.

### Two trackers

Set with `tracker:=`.

| | `hands` (default) | `pose` |
|---|---|---|
| What it watches | your hand, close to the camera | your whole arm, from ~2 m |
| Landmarks | 21, on the hand | 33 body, of which 6 are used |
| Where you stand | sitting, hand 40–60 cm from the lens | standing back, torso in frame |
| To move | **pinch** thumb and index | hold the button |
| Mapping modes | its own, fixed | torso / cylindrical / cartesian |

**`hands` is more precise** because the hand fills the frame instead of being a few dozen
pixels across the room. **`pose` looks better on video** because your whole arm is
visibly driving the robot.

#### How `hands` measures

| Quantity | How |
|---|---|
| Position | midpoint of wrist and middle knuckle — the palm centre. Chosen because it barely moves when the fingers curl, so pinching does not also shift your position |
| Size | average of knuckle span and wrist-to-knuckle length. Used as a distance proxy when depth is unavailable |
| Pinch | thumb-tip to index-tip distance **divided by hand size**, so the same gesture reads the same at any distance. Closes below 0.45, opens above 0.70 — two thresholds so it cannot chatter |
| Distance | sampled from the depth frame at the wrist (see below) |

### Three mapping modes (`pose` tracker only)

Set with `mode:=`, or the radio buttons in the panel.

**`torso` (default)** — body turn drives the base; the arm drives reach and height.

```
turn your shoulders        -> base rotates
push your hand towards the camera -> arm extends
raise / lower your hand    -> tool up / down
```

Swinging your arm does **not** rotate the base. That separation is the point: rotation is
the body's job, reach and height are the arm's.

Reach uses **shoulder-to-hand** distance, not elbow-to-hand. Your forearm is a fixed
length and cannot drive anything; shoulder-to-hand grows as the elbow straightens, which
is what "extending" feels like.

Measured honestly: body yaw is **not** less noisy than hand azimuth — both come out
around 0.85° of jitter for 4 mm of landmark error. What it buys is **independence**. Arm
movement and arm noise cannot rotate the base at all.

**`cylindrical`** — arm swing drives base rotation, one for one.

Your arm sweeps in an arc about your shoulder; the robot sweeps in an arc about its base.
Matching them directly gives **±90°** of base rotation against Cartesian's **±30°** —
about three times the range. Verified: a 60° arm swing produces exactly 60° at j1.

**`cartesian`** — hand XYZ maps to tool XYZ in straight lines.

The original mapping. Base rotation is driven by a sideways offset capped at ±300 mm,
which at a 507 mm radius is only ±30°. Kept for comparison; the other two are better.

### Using the depth camera

On by default (`use_depth:=true`), `hands` tracker only.

The depth stream runs at 480×270×15 and is **aligned to colour**, so a colour pixel and a
depth pixel are the same point in space. A hand landmark's pixel is looked up directly in
the depth frame, and with the camera intrinsics that becomes a true metric 3D position.

A median over a 7×7 patch is used, not a single pixel: depth images have holes at edges
and on dark or shiny surfaces, and a hole reads as **zero**, which would otherwise look
like "right at the lens".

| Reach jitter, hand at 50 cm | |
|---|---|
| inferred from apparent hand size | 18.5 mm |
| **measured depth** | **~2 mm** |

About **9× better** on the axis that was worst — and the mapping becomes metric: move your
hand 10 cm, the tool moves 100 mm × scale.

**The D455 cannot see closer than about 40–50 cm.** Inside that the overlay says
`no depth here` and it falls back to the size estimate automatically.

### The four filters

They sit at different points in the chain and catch different things. One alone is not
enough.

| | Where | Catches |
|---|---|---|
| `hand_smooth_hz` | on the raw measurement | landmark and torso-frame noise |
| `deadband` | on the target | the residual drift a filter cannot remove |
| `filter_hz` | after the IK | jitter the IK itself introduces |
| `max_speed` | at the output | everything else, bounded by physics |

**Why the deadband exists.** A filter only *reduces* noise — it never reaches zero, so
the target creeps forever and the robot faithfully follows. A deadband sets a floor:
below it, nothing happens at all. Measured with 3 mm of landmark noise, holding still:

```
no filtering                 3.07 mm wander,  6.65 mm/frame
smoothing only               1.69 mm wander,  2.55 mm/frame
smoothing + 12 mm deadband   0.00 mm wander,  0.00 mm/frame
```

That is what makes "holding still" actually still.

### ⚠ The open-loop oscillation, and the fix

**On real hardware the arm shook in place and had to be emergency-stopped.** Worth
understanding, because simulation cannot reproduce it.

The command was rate-limited from its **own previous value**, not from where the arm
actually was. In simulation the mock arm reaches every commanded position instantly, so
the two never diverge. On real hardware the arm lags — and the command marched on
regardless. Each new goal then asked the robot to close the whole accumulated gap within
one cycle.

Simulated, with the arm stalled 2 s then following, speed limit 5 °/s:

| | command lead | velocity demanded |
|---|---|---|
| open-loop | unbounded | **80 °/s** |
| fixed 2.5° clamp | 2.5° | 25 °/s |
| **derived from the speed limit** | **0.60°** | **6.0 °/s** |

Sixteen times the limit. The arm accelerates hard, overshoots, is corrected, overshoots
back — shaking in place.

**Two fixes, both in `hand_teleop.py`:**

- **Lead clamp.** The command may never sit more than `max_speed × horizon` ahead of the
  real arm, computed from the speed setting. That bounds the demanded velocity to roughly
  the limit you set.
- **Following check.** If the arm falls more than 8° behind, it stops by itself and says
  `STOPPED: the arm is not following`. Previously nothing would have stopped it.

Homing has the same guards.

**Also fixed at the same time:** `ros2_controllers.yaml` had `update_rate: 100` while the
driver declares an 8 ms (125 Hz) ServoJ cycle. Now 125.

### Configuration-flip rejection

Several joint solutions reach the same tool pose, and the nearest can still be a different
arm configuration — elbow up instead of down. Executing that swings the whole arm through
a wide arc to reach the same point.

The check compares against the **previous frame's IK result**, not against the current
command. Comparing to the command would make any sustained movement look like a flip,
because the command deliberately lags; an earlier version did exactly that and froze the
arm a second or two into every move. Over 480 frames of simulated continuous motion:
**0 false refusals, worst frame-to-frame joint change 2.5°**.

### Workspace shown in RViz

`hand_teleop.py` publishes markers on `/fr5_workspace`. Add them once in RViz:
**Add → By topic → `/fr5_workspace` → MarkerArray**, then **File → Save Config**.

| Marker | Meaning |
|---|---|
| blue translucent cylinder | the reach limit |
| red disc | the floor guard |
| amber inner cylinder | the column above the base, kept clear |
| **green sphere** | the live target — where your hand is sending the tool |

The sphere is the useful one. The gap between it and the flange **is the lag**: the rate
limiter means the sphere leads and the arm follows. Flange sitting inside the sphere means
it is keeping up. Sphere pressed against the blue cylinder means you are at the reach
limit.

Before this, the limits were invisible walls you discovered by hitting them.

### Home-position check

If the arm is more than 15° from home, the home button turns **red** and reads
`ARM IS nn deg FROM HOME`, and **CAPTURE NEUTRAL is disabled** until it is homed.

Capturing a neutral pose while stretched out is the single most common way to make this
feel broken — every direction hits a limit immediately. Measured: a neutral captured at
r = 817 mm, z = 180 mm sat *exactly* on the floor limit with 53 mm of reach left. From
home you get 174 mm of reach headroom and 297 mm above the floor.

**It does not home by itself, deliberately.** GO TO HOME is a joint-space move that does
not avoid obstacles, so a person has to look at the robot and decide the path is clear.
`--allow-any-start` overrides the gate.

### Every option

```bash
ros2 launch ~/fairino_380_ws/teleop.launch.py \
    tracker:=hands          # hands | pose
    mode:=torso             # torso | cylindrical | cartesian  (pose only)
    source:=ros             # ros | webcam
    use_depth:=true         # measured distance vs inferred from hand size
    dry_run:=false          # compute everything, publish nothing
    hold:=false             # hold-to-move instead of click-to-toggle
    scale:=1.0              # motion amplification
    max_speed:=5.0          # degrees per second per joint
    filter_hz:=2.0          # post-IK smoothing, lower is smoother
    hand_smooth_hz:=2.5     # input smoothing, lower is steadier
    deadband:=12.0          # ignore target movement below this, mm
    max_jump:=30.0          # degrees before a solution counts as a flip
    device:=0               # camera index when source:=webcam
```

Script-only flags: `--max-lead`, `--auto-stop`, `--max-offset`, `--max-azimuth`,
`--home-tolerance`, `--allow-any-start`, `--no-pinch`, `--no-depth`.

### Settings that worked

```bash
# simulation, whole-arm tracking
ros2 launch ~/fairino_380_ws/teleop.launch.py tracker:=pose deadband:=20.0

# simulation, hand tracking with depth
ros2 launch ~/fairino_380_ws/teleop.launch.py deadband:=20.0

# real hardware -- hold mode, half scale, 5 deg/s
ros2 launch ~/fairino_380_ws/teleop.launch.py hold:=true scale:=0.5

# very damped, if the room or lighting is poor
ros2 launch ~/fairino_380_ws/teleop.launch.py \
    deadband:=25.0 hand_smooth_hz:=1.2 filter_hz:=0.8 scale:=0.7
```

---
---

# PART 6 — Robot parameters

## 6.1 DH parameters (FR5)

| Joint | θᵢ | aᵢ (mm) | dᵢ (mm) | αᵢ |
|---|---|---|---|---|
| 1 | θ₁ | 0 | 152 | +90° (π/2) |
| 2 | θ₂ | −425 | 0 | 0° |
| 3 | θ₃ | −395 | 0 | 0° |
| 4 | θ₄ | 0 | 102 | +90° (π/2) |
| 5 | θ₅ | 0 | 102 | −90° (−π/2) |
| 6 | θ₆ | 0 | 100 | 0° |

Link lengths 425 mm + 395 mm dominate the 922 mm reach. All six θ are variable
(revolute).

For RoboAnalyzer, in metres: 0.152, −0.425, −0.395, 0.102, 0.102, 0.100.

**Forward kinematics:** joint angles → tool pose. Chain the six transforms.
**Inverse kinematics:** tool pose → joint angles. Multiple solutions exist; the controller
picks one, which is why a `MoveL` can reach the right point in an unexpected arm
configuration.

### Cross-check — the URDF agrees with the DH table

| URDF joint origin | DH equivalent |
|---|---|
| j2 at z = 0.152 | d₁ = 152 mm ✓ |
| j3 at x = −0.425 | a₂ = −425 mm ✓ |
| j4 at x = −0.39501 | a₃ = −395 mm ✓ |
| j5 at z = 0.1021 | d₄ = 102 mm ✓ |
| j6 at z = 0.102 | d₅ = 102 mm ✓ |

The model MoveIt plans with agrees with the DH parameters.

**To verify your own FK:** take the six joint values from `/joint_states`, run your FK on
them, and compare the tool pose against what MoveIt reports for the end-effector frame.
Agreement means both your DH table and your implementation are right.

### `fr5_kinematics.py` — FK and IK implementation

Pure maths, no robot, no ROS needed except for the `live` mode.

```bash
# forward: six joint angles in degrees -> tool pose
python3 fr5_kinematics.py fk 0 -90 90 -90 -90 0

# inverse: tool pose -> every valid joint solution
python3 fr5_kinematics.py ik --xyz -102 -497 -5 --rpy 0 0 -90

# prove FK and IK agree over thousands of random poses
python3 fr5_kinematics.py selftest

# read the live robot and run FK on its actual joint angles
python3 fr5_kinematics.py live
```

**Forward kinematics** chains the six standard-DH transforms
`T_i = Rz(θ) · Tz(d) · Tx(a) · Rx(α)`.

**Inverse kinematics is closed-form, not iterative.** The FR5 has the same kinematic
structure as a UR-style arm — shoulder offset, two parallel elbow links, offset
non-spherical wrist — which has an analytic solution with up to **8 branches**: shoulder
left/right × elbow up/down × wrist flip. No initial guess, no convergence failures.

`--near` marks the solution needing the least total joint movement from where the arm
currently is. That matters for real commanding: several valid solutions reach the same
pose, and you want the one that doesn't flip the elbow through the table.

Solutions are filtered against the **FR5 soft limits**, not the URDF values — see the j3
defect below.

**Verification:** `selftest` generates random reachable joint sets, runs FK, feeds the
pose back through IK, and checks a branch reproduces the original. Current result:

```
2000 poses, worst position error 2.3e-11 mm, worst orientation error 1.7e-06 deg,
original pose recovered 2000/2000, failures 0/2000
```

**Caveat for the `live` comparison.** The DH table's θ zero does not necessarily match
the controller's joint zero. If `live` disagrees with MoveIt's end-effector pose by a
constant per-joint amount, that is a **zero-offset**, not a broken implementation. The
`selftest` result above proves the maths is internally consistent; comparing against the
robot additionally tests the offsets.

## 6.2 FR5 specification

| Parameter | Value |
|---|---|
| Payload | 5 kg |
| Reach | 922 mm |
| Repeatability | ±0.03 mm |
| Max joint speed | 180°/s |
| Typical TCP speed | 1 m/s |
| Weight | ≈22 kg |
| Communication | TCP/IP, Modbus, Profinet, EtherNet/IP, EtherCAT |

## 6.3 Joint limits

| Axis | Soft limit | Hard limit |
|---|---|---|
| 1 | +175° / −175° | +179° / −179° |
| 2 | +85° / −265° | +89° / −269° |
| 3 | **+150° / −150°** | **+152° / −152°** |
| 4 | +85° / −265° | +89° / −269° |
| 5 | +175° / −175° | +179° / −179° |
| 6 | +175° / −175° | +179° / −179° |

**Check every target against this table.** Axes 2 and 4 are asymmetric — −265° is valid,
+265° is not.

### ⚠ Known defect — joint 3 limit mismatch

| Source | Joint 3 range |
|---|---|
| `fairino5_v6.urdf` | **±162°** (±2.8274 rad) |
| FR5 spec, soft limit | ±150° |
| FR5 spec, hard limit | ±152° |

±162° is the **FR16/FR20/FR30** figure, not the FR5's. **MoveIt plans against the URDF,
so it can generate a trajectory that drives j3 past the FR5's own soft limit.**

Either keep well clear of j3 extremes, or fix it in
`src/frcobot_ros2/fairino_description/urdf/fairino5_v6.urdf`:

```xml
<!-- joint j3 -->
<limit lower="-2.6180" upper="2.6180" .../>   <!-- ±150° -->
```

Rebuild `fairino_description` after editing the URDF.

### ⚠ Known defect — ServoJ rate mismatch

`ros2_controllers.yaml` sets `update_rate: 100` (10 ms per cycle), but `write()` calls
`ServoJ(..., 0.008, ...)` — declaring an 8 ms cycle. ServoJ expects to be fed at the rate
it is told.

**Symptoms:** jerky motion, or `ServoJ指令下发错误,错误码:<n>` in the log.

**Fix:** set `update_rate: 125` in
`src/frcobot_ros2/fairino5_v6_moveit2_config/config/ros2_controllers.yaml`.

## 6.4 Frames

Seven coordinate frames: Coordinate 0 at the base, 1–6 at each joint. Frame orientation
determines what "positive X" means in a `CARTPoint` — it is not the room's X.

---
---

# PART 7 — Status

*Last updated: 14 November 2026.*

## Verified on hardware

- Ethernet link to controller at `192.168.58.2`; feedback TCP port 8081 reachable
- Driver builds from tag `V3.0.0_RobotV3.8.0`
- **Method A:** `ros2_cmd_server` runs; `JNTPoint`+`MoveJ` and `CARTPoint`+`MoveL`
  execute real motion; home position script runs
- **Method B:** real hardware plugin loads and reaches `active`; both controllers active;
  joint-slider plan and execute performed on the real arm
- Joint rotation directions recorded for J1–J4, J6
- URDF geometry cross-checked against the DH table
- **Camera:** D455 forwarded into WSL via usbipd; colour stream stable at ~14.5 Hz at
  640×360×15; a second D455 from the lab also works; aligned depth at 480×270×15 works
  and gives ~2 mm reach precision
- **Kinematics:** FK and IK verified against each other over 2000 random poses,
  0 failures (Part 6.1)
- **Method 1** (joint mapping) drove the simulated robot — j3 followed shoulder elevation
- **Method 2** (Cartesian IK) drove the simulated robot in all three mapping modes and
  with both trackers
- **Method 2 drove the real robot.** Hand position controlled the physical arm through
  the closed-form IK with workspace clamping, flip rejection and rate limiting
- MoveIt motion planning executed on the real arm from RViz
- DDS misconfiguration on the lab workstation diagnosed and fixed (Part 8)

## Measured

| | |
|---|---|
| IK round-trip | 2000 / 2000 poses, 0 failures |
| Flip rejection, 480 frames of continuous motion | 0 false refusals, worst joint change 2.5° |
| Hold-still wander, no filtering | 3.07 mm |
| Hold-still wander, smoothing + 12 mm deadband | **0.00 mm** |
| Reach jitter, inferred from hand size | 18.5 mm |
| Reach jitter, measured depth | **~2 mm** |
| Base rotation range, cylindrical mode | ±90° (vs ±30° Cartesian) |
| Commanded velocity before the lead clamp, 5 °/s limit | 80 °/s |
| Commanded velocity after the lead clamp | 6.0 °/s |

## Fixed, worth knowing about

- **The open-loop oscillation** — the arm shook in place on real hardware and had to be
  emergency-stopped. Cause and fix in Part 5.8. **Read that before the next hardware
  session.**
- `ros2_controllers.yaml` `update_rate` was 100 Hz against a declared 8 ms (125 Hz)
  ServoJ cycle. Now 125
- Mirror bug — the tracker followed the wrong arm because detection ran on the flipped
  frame
- Flip guard compared against the lagging command instead of the previous IK result,
  freezing the arm a second into every move
- Workspace clamping added after the arm was driven toward the floor by a joint mapping
  that satisfied every joint limit. **Joint limits do not know where the floor is.**

## Open

- **`/joint_states` stalls on the real hardware setup.** The plugin reaches `active`, all
  six interfaces show `[claimed]`, both controllers are `active`, the topic exists in
  `ros2 topic list` — but nothing is published, and teleop reports `FROZE`. The
  controller manager's update loop appears to be blocked inside the hardware interface's
  `read()`. Likely robot-side rather than software-side: e-stop engaged, teach pendant in
  manual mode, or a pendant application holding the controller. **To diagnose:** capture
  the full Terminal 1 startup log, and cross-check with Method A
  (`ros2_cmd_server` + `/nonrt_state_data`) — if Method A reads state fine, the problem
  is in the ros2_control plugin, not the connection.
- **J5 rotation direction** not recorded
- **j3 URDF limit** not corrected (±162° should be ±150°)
- **Camera not in the URDF** — MoveIt does not know it exists, so collision checking
  ignores it
- **IMU** unavailable in WSL
- **Python demo node** for Method A — motion is still typed by hand

## Not built, but designed

Ideas discussed and deliberately left out:

- Record and replay (lead-through programming)
- Plane-lock mode — 2 DOF only, for a first demo
- Colour-coded video border and audio feedback for engage state
- Unifying the hand tracking point to the palm centre on all three axes

---
---

# PART 8 — Troubleshooting

## Connection

| Symptom | Cause / fix |
|---|---|
| `ping 192.168.58.2` fails | Cable; controller off; laptop NIC not on `192.168.58.x`; check `ip route`. If the **TCP port check passes**, you are fine — WSL2 drops ICMP sometimes |
| Port 8081 unreachable but ping works | Controller still booting; wait a minute |
| Gripper indicator not green | Emergency stop engaged — release it |

## ROS

| Symptom | Cause / fix |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `Package 'X' not found` searching only `['/opt/ros/humble']` | workspace not sourced — `source ~/fairino_380_ws/install/local_setup.bash` in **this** terminal |
| `-Wmaybe-uninitialized` warnings on build | Expected. Vendor code. Ignore |
| VS Code lost the WSL connection | Normal between sessions. `Ctrl+Shift+P` → `WSL: Connect to WSL using Distro` |
| `rcl node's rmw handle is invalid`, or `robot_state_publisher` restarting in a loop, or nodes that cannot see each other's topics | **Broken DDS configuration.** See the fix immediately below this table. Hit on the lab workstation |
| `ros2 topic list` shows the topic but `ros2 topic hz` says *does not appear to be published yet* | Either the DDS fix below, or the publisher really is stalled — if it is `/joint_states`, see Part 7 |

### The DDS fix

ROS 2 nodes find each other through a DDS middleware. A leftover or broken CycloneDDS
configuration file makes nodes fail to start, or start and never see each other.

```bash
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
```

To make it permanent:

```bash
cat >> ~/.bashrc <<'EOF'
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
EOF
```

**Every terminal must match.** Two nodes on different middlewares, or one with
`ROS_LOCALHOST_ONLY=1` and one without, simply never see each other — and nothing
reports an error, which is what makes this one hard to spot. Check with:

```bash
echo $RMW_IMPLEMENTATION $ROS_LOCALHOST_ONLY
# expect:  rmw_fastrtps_cpp 1
```

`ROS_LOCALHOST_ONLY=1` keeps all traffic on the loopback interface. On a machine with
one Ethernet port and no wireless card this also avoids DDS trying to discover peers
across the robot's network.


## Method A

| Symptom | Cause / fix |
|---|---|
| `socket create failed` at server start | Expected in this release. Confirm with the Part 1.3 checks |
| Service call hangs | `ros2_cmd_server` not running |
| `cmd_res='0'` but no motion | Command accepted, not executed. Check e-stop, controller mode, joint limits. **Do not re-send blindly** |
| Robot moves the wrong way | Joint direction sign — see Part 2.4 |

## Method B

| Symptom | Cause / fix |
|---|---|
| `机械臂SDK连接失败！...端口被占用` | Another process holds the connection — `pgrep -af ros2_cmd_server` and kill it |
| `读取初始关节角度错误` | Read failed, plugin refused to activate. **Correct behaviour** — fix comms, never force |
| No Chinese text in the log at all | Still on the mock plugin — check with the `grep` in Part 3.3 |
| Segfault + `class_loader` wall on `Ctrl+C` | Known cosmetic MoveIt bug. Check the lines above it say `shutdown ... finished cleanly` |
| Arm moves fast | Scaling sliders reset to 1.0 on launch — set to 0.05 |
| MoveIt plans through the table | Planning scene is empty by default |

## Camera

| Symptom | Cause / fix |
|---|---|
| `usbipd` not recognized | Close PowerShell, open a new **admin** window (PATH) |
| Camera not in `lsusb` | `usbipd attach --wsl --busid <id>` again — attach does not persist |
| `Device busy (exported) ... used by Windows` | `usbipd bind --force --busid <id>`, then attach |
| `usbipd list` shows `Shared` not `Attached` | bound but not forwarded — run `attach` |
| `VIDIOC_S_FMT failed, errno=16 Device or resource busy` | **a second camera node is running** — `pkill -f realsense2_camera_node`, re-attach, launch once |
| `xioctl(...) Timer expired` then `device has been disconnected` | knock-on from the above; the failed hardware reset drops the usbipd attachment. Full recovery: `pkill`, re-attach, relaunch |
| `Frame Corrupted` / `Incomplete video frame` | Too much bandwidth for virtual USB — use the reduced profiles in Part 4.5 |
| `No HID info provided, IMU is disabled` | Expected. USB/IP does not pass HID. No workaround in WSL |
| Depth returns holes near the tool | Closer than the 52 cm minimum — see Part 4.7 |
| Stream drops during arm motion | USB cable being tugged — check routing and slack |

## Python / vision stack

All of these come from the numpy 1.x vs 2.x split — see Part 4.8.

| Symptom | Cause / fix |
|---|---|
| `no such option: --break-system-packages` | pip 22.0.2 on Ubuntu 22.04 predates it. Drop the flag |
| `AttributeError: _ARRAY_API not found` | numpy 2 loaded against numpy-1.x extensions — `pip3 install "numpy==1.26.4"` |
| `mediapipe is not installed` (but it is) | the import failed on the above; fix numpy |
| `opencv-contrib-python 5 requires numpy>=2` | `pip3 install "opencv-contrib-python==4.10.0.84"` |
| `module 'mediapipe' has no attribute 'solutions'` | mediapipe 1.x deleted that API — `pip3 install "mediapipe==0.10.14"` |
| numpy keeps coming back as 2.x | every `pip install` re-resolves deps. Pin numpy **last**, or just use `requirements.txt` |
| `ReadTimeoutError ... files.pythonhosted.org` | slow connection — `pip3 install --default-timeout=120 --retries 10 ...` |
| `cv_bridge is not installed` | `sudo apt install -y ros-humble-cv-bridge` |
| `tkinter is not installed` | `sudo apt install -y python3-tk` |

## Tracker and teleoperation

| Symptom | Cause / fix |
|---|---|
| `No images on /camera/... yet` | RealSense node not running, or wrong topic — check `ros2 topic list \| grep camera` |
| `NO ARM DETECTED` | Stand further back — both shoulders and both hips must be visible |
| Angles read `---` | Landmark confidence below threshold — improve lighting, or lower `--min-visibility` |
| Skeleton on the wrong arm | Detection must run unmirrored — fixed in the current `arm_tracker.py`, see Part 5.2 |
| Azimuth jumps around wildly | Arm is hanging down, where azimuth is ill-defined — Part 5.2 |
| Mapper: `waiting for /human_arm_angles` | `arm_tracker.py` is not running |
| Teleop: `refused: no /joint_states` | the robot (real or mock) is not launched |
| Teleop: `refused: j<n> off by <x>` | the engage gate — the mapping points somewhere far from the arm's real pose. Adjust offset, or capture neutral |
| Teleop: `FROZE: tracking lost` | you left the camera's view, or a landmark dropped out |
| Teleop: `FROZE` with the camera clearly working | `/joint_states` is not arriving. `ros2 topic hz /joint_states` — if it says *not published yet*, the robot side is the problem, not the vision side. See Part 7 |
| **The arm shakes in place** | Stop it. This was a real bug — the command ran ahead of the arm. Fixed; make sure you are on the current `hand_teleop.py` (`grep -c lead_for hand_teleop.py` must not be 0) and read Part 5.8 |
| `STOPPED: the arm is not following` | Working as designed — the arm fell more than 8° behind. Something is physically holding it, or the controller is not keeping up. Clear the obstruction before re-engaging |
| Overlay says `no depth here` | Hand is closer than the D455's ~40 cm minimum, or on a surface that returns no depth. It falls back to the size estimate by itself |
| `CAPTURE NEUTRAL` greyed out, home button red | The arm is more than 15° from home. Press GO TO HOME first — Part 5.8 |
| Nothing visible in RViz where the target should be | The markers are not added. **Add → By topic → `/fr5_workspace` → MarkerArray**, then File → Save Config |
| Base drifts while you hold still | Raise `deadband:=20.0` and lower `hand_smooth_hz:=1.5` |

---

## Repository layout

```
fairino_380_ws/
├── src/
│   └── frcobot_ros2/                    vendor driver @ V3.0.0_RobotV3.8.0
│       ├── fairino_hardware/            driver + ros2_cmd_server + real HW plugin
│       ├── fairino_msgs/                RemoteCmdInterface.srv
│       ├── fairino_description/         STL meshes + fairino5_v6.urdf
│       └── fairino5_v6_moveit2_config/  MoveIt config + demo.launch.py
├── FOLLOW_ALONG.md                      START HERE — blind step-by-step setup
├── hand_teleop.py                       METHOD 2 — Cartesian teleoperation, all-in-one
├── teleop.launch.py                     METHOD 2 — camera + teleop in one launch
├── arm_tracker.py                       Method 1 phase 1 — vision, arm angles
├── joint_mapper.py                      Method 1 phase 2 — mapping GUI
├── teleop_node.py                       Method 1 phase 3 — safety + robot output
├── fr5_kinematics.py                    forward + inverse kinematics
├── fairino_home.py                      move to home position (Method A)
├── joint_mapping.json                   saved mapping (created by joint_mapper.py)
├── requirements.txt                     pinned Python deps; apt deps in comments
├── .gitignore                           excludes build/ install/ log/
└── README.md
```

`build/`, `install/` and `log/` are generated — never committed.

### Updating the repo

```bash
cd ~/fairino_380_ws
git add .
git commit -m "what changed"
git push
```

---

## Sources

- `fairino_1st_installation.docx.pdf` — commissioning log
- `fairino_for_configured_computers.pdf` — operating procedure, command reference
- `DH.docx` — DH parameters, RoboAnalyzer model
- FAIRINO FR5 specification sheet
- Vendor source: `frcobot_ros2` @ `V3.0.0_RobotV3.8.0`
