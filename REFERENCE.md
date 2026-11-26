# Reference

Full detail on everything. **For getting started, see [README.md](README.md).** For short operating guides, see [RUN_SIMULATION.md](RUN_SIMULATION.md) and [RUN_REAL_ROBOT.md](RUN_REAL_ROBOT.md). For plain-language reasons, see [WHY.md](WHY.md).

---


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

## Project brief — where each requirement is documented

The course brief (12.11.2026) asks for three stages of work and six documentation items.
This maps each to where it lives.

### Stages

| | Required | Where | Status |
|---|---|---|---|
| 1 | Configure the robot in Ubuntu Linux | Parts 1.0–1.4, 2, 3 | **Done** — real arm driven via Method A and Method B |
| 2 | RViz set up, sensor data visualised | Parts 3.4, **3.7**, 4.9 | RViz standalone and with MoveIt **done**; depth point cloud **pending a D455** |
| 3 | MoveIt configured, motion and trajectory planning demonstrated | Part 3 | **Done** — planned and executed on the real arm |

### Documentation items

| Required | Where |
|---|---|
| Ubuntu and ROS 2 setup | `FOLLOW_ALONG.md` Sections A–B · Part 1.1 |
| Required dependencies and package installations | Part 1.0 (versions) · Part 1.1 · `requirements.txt` · Part 4.8 |
| Workspace and package creation | **Part 1.4** |
| Robot URDF and configuration files | **Part 3.0** · Part 6.1 (DH cross-check) |
| Sensor integration and RViz visualization | **Part 3.7** (RViz by itself) · Part 4 · **Part 4.9** |
| MoveIt configuration and trajectory planning | Part 3 |
| Commands, troubleshooting steps, screenshots | Throughout · Part 8 (troubleshooting) · **Part 9 (screenshots)** |

**Reproducible from scratch:** `FOLLOW_ALONG.md` is a 49-step blind walkthrough from a
bare Windows laptop to a moving arm. Every step states what you should see and what to do
when you do not. It is the file to hand someone who has never touched this project.

**Exact versions:** `bash versions.sh | tee VERSIONS.txt` records every one of them from
the machine it runs on. Part 1.0 lists the pins and why each is pinned.

---

## Contents

**[PART 0 — MAIN INSTRUCTIONS](#part-0--main-instructions)** ← always start here

**[PART 1 — Common setup](#part-1--common-setup)**
- [1.0 Versions — record these first](#10-versions--record-these-first)
- [1.1 One-time install](#11-one-time-install-already-done)
- [1.4 Workspace and package structure](#14-workspace-and-package-structure)
- [1.2 Every-session startup](#12-every-session-startup)
- [1.3 Verify communication](#13-verify-communication)

**[PART 2 — METHOD A: Terminal control](#part-2--method-a-terminal-control)**
- [2.1 Start the command server](#21-start-the-command-server)
- [2.2 Joint moves](#22-joint-moves-movej)
- [2.3 Linear moves](#23-linear-moves-movel)
- [2.4 Jogging](#24-jogging)
- [2.5 Stopping](#25-stopping-method-a)

**[PART 3 — METHOD B: RViz + MoveIt control](#part-3--method-b-rviz--moveit-control)**
- [3.0 The configuration files](#30-the-configuration-files)
- [3.1 Install MoveIt and build](#31-install-moveit-and-build)
- [3.2 Test in simulation first](#32-test-in-simulation-first)
- [3.3 Switch to real hardware](#33-switch-to-real-hardware)
- [3.4 Launch and verify](#34-launch-and-verify)
- [3.5 Moving the arm](#35-moving-the-arm)
- [3.6 Stopping](#36-stopping-method-b)
- [3.7 RViz on its own — no MoveIt](#37-rviz-on-its-own--no-moveit)
- [3.8 RViz sliders on the real arm — no MoveIt](#38-rviz-sliders-on-the-real-arm--no-moveit)

**[PART 4 — Camera: Intel RealSense D455](#part-4--camera-intel-realsense-d455)**
- [4.1 What we are doing and why](#41-what-we-are-doing-and-why)
- [4.2 Physical mount](#42-physical-mount)
- [4.3 One-time install](#43-one-time-install)
- [4.4 Every session — the camera](#44-every-session--the-camera)
- [4.5 Why the reduced settings](#45-why-the-reduced-settings)
- [4.6 Verify the stream](#46-verify-the-stream)
- [4.7 D455 limits](#47-d455-limits--read-before-planning-any-vision-work)
- [4.8 The Python dependency trap](#48-the-python-dependency-trap)
- [4.9 Showing sensor data in RViz](#49-showing-sensor-data-in-rviz)

**[PART 5 — Hand tracking and teleoperation](#part-5--hand-tracking-and-teleoperation)**
- [5.0 How this works, in one picture](#50-how-this-works-in-one-picture)
- [5.0 How this works](#50-how-this-works-in-one-picture)
- [5.7 Cartesian hand teleoperation](#57-method-2--cartesian-hand-teleoperation)
  - [5.8 Trackers, modes and tuning](#58-trackers-modes-and-tuning)
    - [The open-loop oscillation, and the fix](#-the-open-loop-oscillation-and-the-fix) ← read before running on real hardware

**[QUICK START — teleoperation in simulation](#quick-start--teleoperation-in-simulation)**

**[PART 6 — Robot parameters](#part-6--robot-parameters)**

**[PART 7 — Status](#part-7--status)**

**[PART 8 — Troubleshooting](#part-8--troubleshooting)**

**[PART 9 — Screenshots](#part-9--screenshots)**
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
first run on real hardware, read the oscillation note in Part 5.8.**

To see RViz **without** MoveIt — the robot model and transforms, configured by hand — see
Part 3.7.

---

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

## 1.0 Versions — record these first

Versions are the first thing to check when something works on one laptop and not another.
Run this on the machine the demonstration will happen on:

```bash
cd ~/fairino_380_ws
bash versions.sh | tee VERSIONS.txt
```

It prints everything below, plus the camera, the controller link, and the line count and
checksum of every script. Commit `VERSIONS.txt`.

### The versions this project is pinned to

| | Version | Why this one |
|---|---|---|
| Windows | 11 | WSL2 needs build 19041+ |
| WSL | 2 | WSL1 cannot forward USB |
| Ubuntu | **22.04 LTS** (Jammy) | **ROS 2 Humble does not exist for 24.04.** This is not a preference |
| ROS 2 | **Humble Hawksbill** | The last distro the vendor driver targets |
| Python | 3.10.x | Ships with 22.04. Do not replace it |
| pip | 22.0.2 | Ships with 22.04. Predates `--break-system-packages` — see 4.8 |
| gcc | 11.4.0 | Ships with 22.04 |
| CMake | 3.22.1 | Ships with 22.04 |

### Pinned Python packages

Exact pins, from `requirements.txt`. **These are load-bearing** — Part 4.8 explains each.

| Package | Pin | Why |
|---|---|---|
| numpy | **1.26.4** | Humble's apt packages are all built against numpy 1.x. numpy 2 breaks them |
| mediapipe | **0.10.14** | Last release with the `mp.solutions` API. 1.x deleted it |
| opencv-contrib-python | **4.10.0.84** | opencv 5 is numpy-2 only |

### Pinned vendor driver

| | Value |
|---|---|
| Repository | `https://github.com/FAIR-INNOVATION/frcobot_ros2.git` |
| Tag | **`V3.0.0_RobotV3.8.0`** |
| Commit | `7c683cf` |
| Matching controller firmware | **v3.8.0.1** |

**Do not `git pull` to main.** The tag and the firmware are a matched pair.

> **If `versions.sh` says `NO .git FOLDER`**, the driver source is there but its history
> is not, so nothing can prove which version was built. Check what you have:
>
> ```bash
> ls -d ~/fairino_380_ws/src/frcobot_ros2/.git
> cat ~/fairino_380_ws/.gitmodules 2>/dev/null
> ```
>
> To restore it without touching the working files, re-clone beside it and compare:
>
> ```bash
> cd /tmp
> git clone --branch V3.0.0_RobotV3.8.0 \
>     https://github.com/FAIR-INNOVATION/frcobot_ros2.git fr5-reference
> diff -rq /tmp/fr5-reference ~/fairino_380_ws/src/frcobot_ros2 \
>     --exclude=.git | head -40
> ```
>
> Anything listed is a local change to vendor code. The only one this project
> intends are the plugin line in `fairino5_v6_robot.ros2_control.xacro`,
> `joint_limits.yaml` (velocities lowered to 3.0), and
> `fairino_hardware_interface.cpp` (the `ServoMoveStart` patch).
> `ros2_controllers.yaml` should read `update_rate: 100` — the vendor's original value.
> **Anything else on that list is unexplained and should be understood before a
> demonstration.**

### Robot and camera

| | Value |
|---|---|
| Robot | FAIRINO FR5, 6-axis collaborative arm |
| Payload / reach | 5 kg / 922 mm |
| Controller IP | `192.168.58.2` |
| Laptop IP | `192.168.58.1`, mask `255.255.255.0` |
| Camera | Intel RealSense D455 |
| Colour stream | 640×360 @ 15 fps |
| Depth stream | 480×270 @ 15 fps, aligned to colour |

Controller and servo firmware are read from the **teach pendant**, under
*System → About / Version*. There is no command that reads them over the network —
record them by hand and put them in `VERSIONS.txt`.

---

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

## 1.4 Workspace and package structure

A **colcon workspace** is a folder with a `src/` directory in it. Everything else —
`build/`, `install/`, `log/` — colcon generates. That is the whole concept.

```
~/fairino_380_ws/          <- the workspace
├── src/                   <- the only folder you ever put source in
│   └── frcobot_ros2/      <- the vendor's repository, cloned here
├── build/                 <- generated by colcon. Never committed
├── install/               <- generated by colcon. Never committed
└── log/                   <- generated by colcon. Never committed
```

**No package was created from scratch for this project.** The vendor ships the ROS 2
packages; this project clones them at a pinned tag and adds standalone Python scripts
at the workspace root. That was deliberate — a hand-rolled package wrapping the vendor's
driver would be another thing to keep in sync for no benefit.

For reference, had one been needed:

```bash
cd ~/fairino_380_ws/src
ros2 pkg create --build-type ament_python my_package \
    --dependencies rclpy sensor_msgs trajectory_msgs
```

### The four packages that come from the clone

```bash
ls ~/fairino_380_ws/src/frcobot_ros2
```

| Package | Build type | What it is |
|---|---|---|
| `fairino_msgs` | ament_cmake | Service definitions, chiefly `RemoteCmdInterface.srv` |
| `fairino_hardware` | ament_cmake | The driver: `ros2_cmd_server`, the `ros2_control` plugin, and the vendor's compiled `.so` |
| `fairino_description` | ament_cmake | The robot model — STL meshes and `fairino5_v6.urdf` |
| `fairino5_v6_moveit2_config` | ament_cmake | MoveIt configuration and `demo.launch.py` |

The repository also ships FR3/FR10/FR16/FR20/FR30 variants of the last two. **Use the
`fairino5` ones.** Launching an FR10 config against an FR5 gives a robot that looks
nearly right and reaches the wrong places.

### Building

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fairino_hardware --symlink-install
source install/local_setup.bash
```

`--packages-up-to fairino_hardware` builds only that package and what it depends on —
2 packages, not the whole repository. Build everything with a bare `colcon build` if you
want the MoveIt config too.

`--symlink-install` is why editing a config file takes effect without rebuilding: the
install tree points at the source files instead of copying them. **This is load-bearing
for Part 3.3**, where one line in a xacro switches between the simulated and the real
robot.

### Sourcing — the rule that catches everyone

Every new terminal needs **both** lines:

```bash
source /opt/ros/humble/setup.bash          # ROS itself
source ~/fairino_380_ws/install/local_setup.bash   # this workspace
```

Miss the second and you get `Package 'fairino5_v6_moveit2_config' not found`, with the
error showing it searched only `['/opt/ros/humble']`. That message is the diagnosis —
the search path tells you which line is missing.

---

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

## 3.0 The configuration files

MoveIt is configured entirely by files. Nothing is hard-coded. Before changing anything,
list what is actually there:

```bash
ls ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/
ls ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/launch/
ls ~/fairino_380_ws/src/frcobot_ros2/fairino_description/
```

**Do this rather than trusting the table below.** Vendor layouts shift between releases;
the filenames here are what tag `V3.0.0_RobotV3.8.0` ships, and the `ls` is how you
confirm it.

### What each file does

| File | What it defines |
|---|---|
| `fairino5_v6.urdf` | **The robot itself** — links, joints, axes, limits, meshes. The single source of truth for geometry |
| `*.srdf` | **Semantics** — which joints form the arm group, named poses like `home`, and which link pairs can never collide |
| `kinematics.yaml` | Which IK solver MoveIt uses, its timeout and resolution |
| `joint_limits.yaml` | Velocity and acceleration ceilings **for planning**. Separate from the URDF's position limits |
| `ompl_planning.yaml` | Planner settings (RRTConnect and friends). **Not in this package** — `MoveItConfigsBuilder` falls back to MoveIt's default OMPL settings, which is what runs |
| `ros2_controllers.yaml` | Controller definitions — `fairino5_controller`, `joint_state_broadcaster`, and the `update_rate` |
| `moveit_controllers.yaml` | How MoveIt hands trajectories to those controllers, via `FollowJointTrajectory` |
| `fairino5_v6_robot.ros2_control.xacro` | **The hardware plugin line** — simulated or real. Part 3.3 |

### URDF vs SRDF

The one that confuses people. The **URDF** says what the robot *is* — this link is
425 mm long, this joint rotates about Z between these angles. The **SRDF** says what to
*do* with it — these six joints are a planning group called `fairino5_v6_group`, this
set of angles is called `home`, these two links are always touching so stop checking them
for collisions.

MoveIt needs both. The URDF alone describes a robot nothing knows how to plan for.

### Checking the URDF agrees with the DH table

The URDF and the DH parameters in Part 6.1 describe the same robot two different ways.
They must agree, and this project checked that they do:

```bash
cd ~/fairino_380_ws
source install/local_setup.bash

# render the xacro to plain URDF and look at the joint origins
xacro src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.urdf.xacro \
  > /tmp/fr5.urdf
grep -A2 "<origin" /tmp/fr5.urdf | head -40

# check it parses at all
check_urdf /tmp/fr5.urdf
```

The link offsets should match the `d` and `a` columns of the DH table: 152, 425, 395,
102, 102, 100 mm. **This is the cross-check that proves the kinematics in
`fr5_kinematics.py` describe the robot RViz is drawing**, and it is worth showing the
examiner.

### Known defects in the vendor configuration

Found by inspection and recorded here rather than silently patched:

| Where | Defect | Status |
|---|---|---|
| `ros2_controllers.yaml` | `update_rate` — originally 100 Hz | **Not a defect.** Changed to 125, then changed back. See 6.3 — the change was a mistake |
| `joint_limits.yaml` | `max_velocity` 3.15 / 3.20 rad/s = 180.5 / 183.3 °/s, above the FR5's rated **180 °/s** | Lowered to 3.0 (171.9 °/s) |
| `fairino_hardware_interface.cpp` | `ServoJ` streamed with no `ServoMoveStart()` / `ServoMoveEnd()`, which the SDK requires | **Patched** — `patch_driver.sh` |
| `fairino_hardware_interface.cpp` | A refused `ServoJ` logs at INFO every cycle and `write()` returns OK, so MoveIt reports success while the arm stands still | **Patched** — logs once at ERROR |
| `fairino5_v6.urdf` | j3 limit was ±162°, but the FR5 specification says ±150° | **Fixed** — set to ±2.6180 rad (±150°), limit and safety-controller lines |
| `fairino5_v6_robot.ros2_control.xacro` | Committed with the plugin on real hardware while its comment said simulation | **Fixed** — committed on `mock_components/GenericSystem` |

---

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

## 3.7 RViz on its own — no MoveIt

**RViz and MoveIt are not the same thing**, and it is worth being able to show that.

**RViz is a 3D viewer.** An empty window that draws whatever you add to it — a robot,
camera data, transforms, markers. It commands nothing.

**MoveIt is a motion planner.** It has no window of its own, so it borrows RViz. Every
display in `demo.launch.py` was put there by MoveIt, not by you.

This section brings up RViz with nothing preconfigured and adds the displays by hand.

### One-time install

```bash
sudo apt update
sudo apt install -y ros-humble-joint-state-publisher-gui
```

> `apt update` first. A stale package index gives `404 Not Found` on the `.deb`.
> **Do not run `apt upgrade`** — it will offer to move MoveIt, ros2_control and the
> RealSense driver, all of which currently work and are recorded in `VERSIONS.txt`.

### Three terminals

```bash
# 1 -- publish the robot model
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
xacro src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.urdf.xacro \
  > /tmp/fr5.urdf
ros2 run robot_state_publisher robot_state_publisher /tmp/fr5.urdf
```

```bash
# 2 -- sliders, one per joint
source /opt/ros/humble/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

```bash
# 3 -- RViz, empty
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
rviz2
```

### Configure it yourself

1. **Global Options → Fixed Frame** → `base_link`
2. **Add → RobotModel** → set **Description Topic** to `/robot_description`
3. **Add → TF**

Drag a slider. The arm moves. **No MoveIt is running.**

### ⚠ Both source lines in the RViz terminal

Miss `source install/local_setup.bash` in terminal 3 and you get this:

```
RobotModel   Status: Error
```

TF still draws — the joint frames appear as red/green/blue triads — but **no arm**. The
meshes are referenced as `package://fairino_description/meshes/...`, and RViz cannot
resolve `package://` for a workspace it was never told about. The structure loads, every
mesh fails.

This is the clearest demonstration of what `local_setup.bash` actually does.

### What the TF display is showing

Each triad is one joint's coordinate frame — **red = X, green = Y, blue = Z** — with
arrows from parent to child. That is the DH table of Part 6.1 drawn in 3D. Where a `d`
offset moves a frame along Z or an `a` offset moves it along X, you are looking at it.

### This is view-only

The sliders publish `/joint_states`, which moves **the picture**. Nothing reaches the
robot. To actually move the arm you need something that sends a trajectory to the
controller — MoveIt's MotionPlanning panel, Method A, the teleoperation in Part 5, or the
slider bridge in 3.8.

**With the real robot connected, do not run `joint_state_publisher_gui` on its own.**
`/joint_states` then comes from the arm's own encoders, and a second publisher on the same
topic makes RViz flicker between the real position and the slider position.

| | Draws the robot | Commands the robot |
|---|---|---|
| RViz alone | yes | **no** |
| RViz + MoveIt (Part 3) | yes | yes |
| RViz + sliders through `slider_bridge.py` (3.8) | yes | yes |
| Method A (Part 2) | — | yes |
| Hand teleoperation (Part 5) | yes | yes |

## 3.8 RViz sliders on the real arm — no MoveIt

The brief asked for the arm to be moved from RViz alone, without MoveIt. Three files do
it:

| File | Job |
|---|---|
| `rviz_real.launch.py` | robot_state_publisher, ros2_control_node, both controller spawners and plain `rviz2`. **No `move_group`.** Uses `MoveItConfigsBuilder` only to read the robot description the way `demo.launch.py` does |
| `slider_controllers.yaml` | The vendor's `ros2_controllers.yaml` plus `open_loop_control: true`. Loaded only by `rviz_real.launch.py`; MoveIt keeps the original |
| `slider_bridge.py` | Starts `joint_state_publisher_gui` remapped to `/slider_joint_states` with its sliders preset to the arm's real pose, and turns slider moves into controller goals |

**What the bridge sends:** while a slider is moving, 10 times a second, one point on
`/fairino5_controller/joint_trajectory` — at most **1°** from where the controller's
reference is now, `time_from_start` set so every joint moves at **5 °/s** (`--max-speed`,
capped at 20), and `header.stamp` set to the send time. Nothing while the sliders are
still.

| Guard | Value |
|---|---|
| Pick-up: every slider within this of the arm before anything moves | 3° |
| Soft limits (FR5's, not the URDF's), with margin | as 6.3, 2° margin |
| Goal never further than this past the arm — fresh / paused feedback | 1.5° / 1° |
| All-zero or out-of-range `/joint_states` | refuse, latch |
| Asking to move, arm not moving | 3 s → latch |
| No `/joint_states` at all | 10 s → latch |

Operating steps: `RUN_SIMULATION.md` Part 2b, `RUN_REAL_ROBOT.md` Part 5b. The reasoning
behind each choice, including the versions that vibrated or tripped the speed alarm:
`WHY.md` — [sliders need a bridge](WHY.md#why-the-sliders-need-a-bridge),
[slider vibration](WHY.md#why-the-sliders-vibrated).

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

`mediapipe==0.10.14` is the last release with the `mp.solutions` API that `hand_teleop.py`
uses. Moving to mediapipe 1.x would require rewriting the tracker for the Tasks API plus
downloading a separate `.task` model file.

**On a slow connection**, mediapipe is ~35 MB and pip gives up quickly:

```bash
pip3 install --default-timeout=120 --retries 10 -r requirements.txt
```

---
---
## 4.9 Showing sensor data in RViz

Part 4.6 verifies the camera works. This section puts its data **inside RViz, next to the
robot** — which is the thing to demonstrate, because it is what "sensor data
visualization" means.

Three displays, in increasing order of how good they look.

### 1. The colour image

With the camera running (Part 4.4) and RViz open:

**`Add` → `By topic` → `/camera/camera/color/image_raw` → `Image` → OK**

A video panel appears in the corner of RViz. Lowest effort, and enough to prove the
stream is live.

### 2. The depth cloud — the one worth showing

Depth comes out as a 3D point cloud, drawn in the same space as the robot.

The camera node must publish it, which it does not do by default. Start it with the
point cloud enabled:

```bash
ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:=640x360x15 \
    depth_module.depth_profile:=480x270x15 \
    align_depth.enable:=true \
    pointcloud.enable:=true \
    enable_infra1:=false enable_infra2:=false
```

Then in RViz:

**`Add` → `By topic` → `/camera/camera/depth/color/points` → `PointCloud2` → OK**

Set the display's **Color Transformer** to `RGB8`. You now have a coloured 3D cloud of
whatever the camera sees.

> **If the cloud does not appear**, it is almost always the **Fixed Frame**. RViz draws
> everything relative to one frame; the robot's is `base_link` and the camera's is
> `camera_link`, and ROS has not been told how they relate (see the warning below). Set
> `Global Options → Fixed Frame` to `camera_link` and the cloud appears — the robot will
> then be the thing that vanishes.
>
> That is the honest state of this project, and worth saying out loud rather than
> hiding.

### ⚠ The camera is not in the URDF

**The robot model does not know the camera exists.** There is no link for it, so there is
no transform between `base_link` and `camera_link`. Three consequences:

- RViz cannot draw the robot and the point cloud in one coherent scene
- MoveIt's collision checking does not know the camera is there — **it will happily plan
  the arm through it**
- The cloud cannot be used as a collision object, which is the usual reason to want one

For a demonstration, a static transform is enough to put both in the same scene. Measure
where the camera actually sits relative to the robot base and publish it:

```bash
# x y z yaw pitch roll parent child      -- metres and radians
ros2 run tf2_ros static_transform_publisher \
    1.0 0.0 0.8  3.14159 0.0 0.0  base_link camera_link
```

Those numbers are an example: **camera 1 m in front of the base, 0.8 m up, facing back
towards the robot.** Measure your own mounting and substitute. Then set Fixed Frame back
to `base_link` and both appear together.

This is a demonstration aid, not a calibration. A proper job adds a camera link to the
URDF with a measured origin, which is listed as open work in Part 7.

### 3. The robot's own state

Already on by default, and worth naming because it is also sensor data: `/joint_states`
carries the arm's real encoder positions, and `RobotModel` in RViz is drawing from them.
When the arm is on real hardware, **the robot you see in RViz is a live readout of the
physical one**, not an animation.

```bash
ros2 topic echo /joint_states --once
```

### Save the RViz layout

Having added displays, **File → Save Config**, or they are gone next launch. Along with
the `/fr5_workspace` markers from Part 5.8, that is the demonstration layout.

---

---
---

# PART 5 — Hand tracking and teleoperation

**Goal:** the operator's hand is tracked by the camera, and its position in space becomes
a target the robot's tool follows in real time.

```
your hand → camera → hand_teleop.py → inverse kinematics → SAFETY
                                    → /fairino5_controller/joint_trajectory → robot
```

## 5.0 How this works, in one picture

One process does everything: vision, mapping, IK, safety, output.

| Stage | What happens |
|---|---|
| **Track** | MediaPipe finds your hand (or whole arm) in the camera frame |
| **Measure** | Its position becomes a 3D point — from the depth camera where available |
| **Map** | That point becomes a target position for the robot's tool |
| **Solve** | The closed-form IK (Part 6.1) turns the target into six joint angles |
| **Protect** | Deadband, smoothing, rate limit, joint clamp, lead clamp, auto-stop |
| **Send** | A joint trajectory goes to the controller |

**The IK is the heart of it.** Without a solver there is no way to turn "my hand is
*there*" into six joint angles, which is why Part 6.1 exists and why this method uses it
centrally.

> **An earlier approach mapped human joint angles straight onto robot joints** — your
> elbow drives j3, and so on. It is simpler to explain, and it was built and worked, but
> it has one fatal property: **it has no idea where the tool ends up.** During testing it
> drove the arm toward the floor while satisfying every joint limit, because joint limits
> do not know where the floor is. That failure is why this method solves for tool
> *position* instead, where a floor is one line of code. The old scripts remain in the
> repository's git history.

## 5.7 METHOD 2 — Cartesian hand teleoperation

**One process, one window.** `hand_teleop.py` does vision, mapping, IK, safety and output
in a single program, and `teleop.launch.py` starts it together with the camera.

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

**Changed at the same time, and later retracted:** `update_rate` was moved from 100 to
125 Hz. That was a mistake and it is back at 100 — see Part 6.3.

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

### ✅ Fixed — joint 3 limit mismatch

| Source | Joint 3 range |
|---|---|
| `fairino5_v6.urdf`, as shipped | ±162° (±2.8274 rad) |
| `fairino5_v6.urdf`, now | **±150° (±2.6180 rad)** |
| FR5 spec, soft limit | ±150° |
| FR5 spec, hard limit | ±152° |

±162° is the **FR16/FR20/FR30** figure, not the FR5's. MoveIt plans against the URDF, so
it could generate a trajectory that drove j3 past the FR5's own soft limit. The `<limit>`
and `<safety_controller>` lines of joint `j3` now say ±2.6180. With
`colcon build --symlink-install` no rebuild is needed; otherwise rebuild
`fairino_description`.

### ⚠ RETRACTED — the ServoJ "rate mismatch"

**Earlier versions of this document told you to change `update_rate` from 100 to 125.
That advice was wrong. Set it back to 100.**

```bash
cd ~/fairino_380_ws
sed -i 's/update_rate: 125/update_rate: 100/' \
  src/frcobot_ros2/fairino5_v6_moveit2_config/config/ros2_controllers.yaml
```

**The original reasoning:** `write()` calls `ServoJ(..., 0.008, ...)`, declaring an 8 ms
cycle, while the controller ran every 10 ms. Two numbers describing the same cycle and
disagreeing, so the controller was changed to match.

**Why that was wrong.** The robot computes speed as **position delta ÷ cmdT**, and cmdT
is hard-coded at 8 ms. At 100 Hz a command arrives every 10 ms but claims to need 8 ms,
so the arm always arrives early and waits. **That 2 ms is slack, not a bug.**

On a machine with no real-time guarantees — WSL has none — that margin is what absorbs a
late cycle. Remove it and a delayed cycle means the controller's spline has advanced
further, the next command carries a bigger delta, but still declares 8 ms. Speed =
bigger delta ÷ 8 ms, over the limit, and the robot faults.

**It is also how the change was made that was wrong.** Nothing was failing. There was no
symptom. A live setting on a 22 kg arm was changed because two numbers looked untidy
when read, the change was never tested, and it was written up here as a "fix" for a
defect identified by reading rather than by observing.

Another fork of this driver went the opposite way — changing cmdT to 10 ms to match a
100 Hz controller — which supports the reading that the controller rate is the one to
leave alone.

**Suspected consequence:** see Part 7, the error-14 refusals of 25 November.

## 6.4 Frames

Seven coordinate frames: Coordinate 0 at the base, 1–6 at each joint. Frame orientation
determines what "positive X" means in a `CARTPoint` — it is not the room's X.

---
---

# PART 7 — Status

*Last updated: 26 November 2026.*

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
- **Method 2** (Cartesian IK) drove the simulated robot in all three mapping modes and
  with both trackers
- **Method 2 drove the real robot.** Hand position controlled the physical arm through
  the closed-form IK with workspace clamping, flip rejection and rate limiting
- MoveIt motion planning executed on the real arm from RViz
- **RViz driven standalone** — `robot_state_publisher` + `joint_state_publisher_gui` +
  bare `rviz2`, with RobotModel and TF added by hand (Part 3.7)
- **RViz sliders moved the real arm with no MoveIt** — `rviz_real.launch.py` +
  `slider_bridge.py`, smooth at 5 °/s after the vibration fixes (Part 3.8), 26 Nov
- **The robot in MANUAL mode was the cause of "SUCCEEDED but no motion"** — switched to
  AUTO, MoveIt executes on the real arm, 26 Nov
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
| `/joint_states`, robot only, `ros2 topic hz` | 100.0 Hz, max gap 12 ms |
| `/joint_states` seen by a *reliable* subscriber | ~1 s bursts |
| Slider bridge, worst jump if the control loop stalls 1 s (simulated) | 1.0° in one cycle |

## Fixed, worth knowing about

- **The open-loop oscillation** — the arm shook in place on real hardware and had to be
  emergency-stopped. Cause and fix in Part 5.8. **Read that before the next hardware
  session.**
- `joint_limits.yaml` allowed 180.5–183.3 °/s against the FR5's rated 180 °/s, so MoveIt
  could plan trajectories the controller refuses. Lowered to 171.9 °/s
- `ServoJ` was streamed with no `ServoMoveStart()`. Patched
- A refused `ServoJ` was invisible: INFO level, every cycle, and `write()` returned OK.
  Now one ERROR naming the remedy
- **Slider vibration** — unstamped, closed-loop goals re-sent every 40 ms made the arm
  buzz in place; a long goal plus a stall tripped the axis 6 speed alarm. Fixed with
  stamped 1° segments, `open_loop_control`, send-on-change (Part 3.8,
  [WHY](WHY.md#why-the-sliders-vibrated))
- j3 URDF limit ±162° → ±150° (Part 6.3)
- Mirror bug — the tracker followed the wrong arm because detection ran on the flipped
  frame
- Flip guard compared against the lagging command instead of the previous IK result,
  freezing the arm a second into every move
- Workspace clamping added after the arm was driven toward the floor by a joint mapping
  that satisfied every joint limit. **Joint limits do not know where the floor is.**

## Removed

- **Method 1**, the three-stage joint-mapping teleoperation (`arm_tracker.py`,
  `joint_mapper.py`, `teleop_node.py`). It worked in simulation, but mapping human joint
  angles onto robot joints cannot know where the tool ends up — see the note in Part 5.0.
  Superseded by the Cartesian method. **Still in git history** if it is ever wanted.

## ⭐ Resolved — 26 November

**MoveIt executes on the real arm.** The blocker was the robot being in **manual mode**.

In manual the FR5 refuses the `ServoJ` stream (error 14) while still publishing
`/joint_states` and still accepting `MoveJ`. Every check on the computer passes and only
the MoveIt path fails, which is why it took three sessions.

Proved by elimination: the run immediately before had `update_rate` back at 100, the
joint limits lowered, the `ServoMoveStart` patch applied and the alarm cleared, and it
failed. The next run changed one thing — `robot_mode: 1 → 0` — and the arm moved.

**Operationally:** check `robot_mode` is `0` before every session.

```bash
timeout 10s ros2 topic echo /nonrt_state_data --once --field robot_mode
```

### The two speed limits multiply

| | |
|---|---|
| Web UI speed box | a global cap on the robot itself, applies to everything |
| MoveIt velocity scaling | scales the planned trajectory, **resets to 1.0 every launch** |

At 10% and 0.05 the arm runs at **0.5%** of maximum. Raise them one at a time.

## Open

- **`/joint_states` pauses as seen by subscribers.** With only the robot running,
  `ros2 topic hz /joint_states` shows a clean 100 Hz (max 12 ms); a script with a
  *reliable* subscription received it in ~1 s bursts. Subscribing best-effort
  (`qos_profile_sensor_data`) — now done in `hand_teleop.py`, `slider_bridge.py` and
  `nudge_j1.py` — mostly fixes it; pauses still appeared in some sessions, and a clean
  restart of everything helped. Likely the same thing as the stall below.
  [WHY.md](WHY.md#why-joint_states-arrived-in-bursts)
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
- **Camera not in the URDF** — MoveIt does not know it exists, so collision checking
  ignores it
- **IMU** unavailable in WSL
- **Python demo node** for Method A — `fairino_home.py` sends the arm home; any other
  Method A motion is still typed by hand
- **Hand control goals** — `hand_teleop.py` still sends unstamped, closed-loop goals
  every 40 ms; giving it the slider bridge's stamped, open-loop goals should remove the
  same kind of buzz

## Documentation

- `RUN_SIMULATION.md` — short operating guide, no robot needed
- `RUN_REAL_ROBOT.md` — short operating guide for the real arm, safety first
- `FOLLOW_ALONG.md` — 49 steps, bare Windows laptop to a moving arm, every step with an
  expected result and a failure box
- `README.md` — setup, the file list, and where the project stands
- `HOW_IT_WORKS.md` — the whole project explained from nothing
- `WHY.md` — every "why is it like this?"
- `REFERENCE.md` — this file: why each step exists, the parameters, the full
  troubleshooting reference
- `versions.sh` → `VERSIONS.txt` — every version, recorded from the machine itself
- `docs/screenshots/` — Part 9 lists what to capture and under what filename

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
| Skeleton on the wrong arm | Detection must run unmirrored. Fixed — if you see this, you are on an old copy of `hand_teleop.py` |
| Azimuth jumps around wildly | Arm is hanging down, where azimuth is ill-defined — Part 5.2 |
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
---
---

# PART 9 — Screenshots

The brief asks for **screenshots at each stage**. This is the list, with the filename to
save each under. Put them in `docs/screenshots/` in the repository:

```bash
mkdir -p ~/fairino_380_ws/docs/screenshots
```

**Taking one in WSL:** `Win` + `Shift` + `S` captures any region of the screen, including
Linux windows — they are ordinary Windows windows. Paste into Paint and save as `.png`,
or the screenshot lands in `~/Pictures/Screenshots` on the Windows side and can be copied
across:

```bash
cp /mnt/c/Users/<you>/Pictures/Screenshots/<file>.png \
   ~/fairino_380_ws/docs/screenshots/01-wsl-version.png
```

### The list

| # | Filename | What it shows | Stage |
|---|---|---|---|
| 01 | `01-wsl-version.png` | `wsl --version` and `lsb_release -a` | Setup |
| 02 | `02-ros-distro.png` | `echo $ROS_DISTRO` printing `humble` | Setup |
| 03 | `03-colcon-build.png` | The build finishing, `N packages finished` visible | Build |
| 04 | `04-pkg-list.png` | `ros2 pkg list \| grep fairino` — all four packages | Build |
| 05 | `05-versions.png` | `versions.sh` output, or `VERSIONS.txt` open in an editor | Versions |
| 06 | `06-rviz-sim.png` | RViz with the simulated arm, first launch | Simulation |
| 07 | `07-controllers.png` | `ros2 control list_controllers` — both `active` | Simulation |
| 08 | `08-moveit-plan.png` | **MoveIt mid-plan** — the orange goal state offset from the arm | MoveIt |
| 09 | `09-moveit-trajectory.png` | The planned trajectory drawn, before Execute | MoveIt |
| 10 | `10-rqt-graph.png` | `rqt_graph` — the node and topic graph | MoveIt |
| 11 | `11-camera-stream.png` | The RealSense colour stream live | Sensor |
| 12 | `12-rviz-pointcloud.png` | **Depth point cloud inside RViz** (Part 4.9) | Sensor |
| 13 | `13-fk-ik-selftest.png` | `fr5_kinematics.py selftest` — 2000/2000, 0 failures | Kinematics |
| 14 | `14-urdf-check.png` | `check_urdf` output, link offsets matching the DH table | Kinematics |
| 15 | `15-teleop-panel.png` | The teleoperation panel and the video window side by side | Teleop |
| 16 | `16-rviz-workspace.png` | RViz with the `/fr5_workspace` markers and the green target sphere | Teleop |
| 17 | `17-hardware-plugin.png` | The `grep plugin` from Part 3.3 showing `FairinoHardwareInterface` | Real robot |
| 18 | `18-real-joint-states.png` | `ros2 topic echo /joint_states` matching the physical arm | Real robot |
| 19 | `19-robot-moving.png` | A photo of the actual arm mid-move | Real robot |
| 20 | `20-git-log.png` | `git log --oneline` — the project's history | Documentation |

**8, 12, 16 and 19 are the four that matter** — they are the only ones that show
something working rather than something installed. If time is short, take those.

### Referencing them

Markdown embeds them directly, which is what makes a GitHub repository read well:

```markdown
![MoveIt planning a trajectory](docs/screenshots/08-moveit-plan.png)
```

### A note on 19

Have someone else hold the camera. **Your hand stays on the e-stop.**

---
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
├── FOLLOW_ALONG.md                      installing from scratch, 49 steps
├── RUN_SIMULATION.md                    how to run it with no robot
├── RUN_REAL_ROBOT.md                    how to run it on the real arm — READ FIRST
├── hand_teleop.py                       METHOD 2 — Cartesian teleoperation, all-in-one
├── teleop.launch.py                     METHOD 2 — camera + teleop in one launch
├── fr5_kinematics.py                    forward + inverse kinematics
├── fairino_home.py                      move to home position (Method A)
├── requirements.txt                     pinned Python deps; apt deps in comments
├── versions.sh                          records every version -> VERSIONS.txt
├── VERSIONS.txt                         generated by versions.sh -- commit it
├── docs/screenshots/                    screenshots, numbered -- Part 9
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
