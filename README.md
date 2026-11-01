# FAIRINO FR5 — ROS 2 Commissioning

Robotics Mechanisms course project. Commissioning a FAIRINO FR5 collaborative arm and
driving it from ROS 2 Humble with Python.

**Repo:** https://github.com/Aahylxo/FairinoF5---Project-

---

## Contents

1. [Safety — read before touching the robot](#1-safety)
2. [What this system is](#2-what-this-system-is)
3. [Hardware connection](#3-hardware-connection)
4. [One-time setup (already done)](#4-one-time-setup)
5. [Every-session startup](#5-every-session-startup)
6. [Verify communication](#6-verify-communication)
7. [Moving the robot](#7-moving-the-robot)
8. [Stopping the robot](#8-stopping-the-robot)
9. [Robot parameters — DH, limits, frames](#9-robot-parameters)
10. [Status — verified vs not](#10-status)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. Safety

The FR5 weighs ~22 kg and has a 922 mm reach. It can hit you, and it can hit itself.

**Do not run motion commands without a supervisor present.** The course guide states a
trained operator must verify the setup before any motion test. That is not a formality —
a 1° joint change moves the tool tip noticeably, and nothing in software checks whether
your head is in the way.

Before any motion command:

- [ ] Supervisor/instructor present
- [ ] Physical emergency stop within arm's reach of **you**, not the robot
- [ ] Swept volume clear for the **whole arm and tool**, not just the end effector
- [ ] Nobody inside the work envelope
- [ ] Current joint angles known (read them first — see §6)
- [ ] Target checked against joint limits (§9)
- [ ] Speed parameter is **5** (5%), never higher while learning

**Speed is why it looks slow.** Every command in this guide uses `5` = 5% of maximum.
The FR5 can do 180°/s per axis and 1 m/s at the tool. At 5% it crawls, which is correct
for commissioning. Do not raise it.

**A `cmd_res='0'` response means the API accepted the command.** It does not prove the
robot moved where you intended. Always verify with feedback afterwards. Never re-send a
command blindly because "nothing happened" — it may be moving slowly, or queued.

---

## 2. What this system is

Four layers, bottom to top:

| Layer | What it is | Where it runs |
|---|---|---|
| FR5 controller | Vendor control box, fixed IP `192.168.58.2` | In the robot base |
| `fairino_hardware` | Vendor ROS 2 driver, speaks TCP to the controller | Your laptop |
| `ros2_cmd_server` | Node exposing the robot as a ROS 2 service | Your laptop |
| Your commands | `ros2 service call`, or Python | Your laptop |

Communication is Ethernet. You send a command string like `MoveJ(JNT2,5,0,0)` to the
service `/fairino_remote_command_service`, the driver forwards it to the controller, the
controller moves the arm. Feedback comes back on the topic `/nonrt_state_data`.

**Host environment:** Windows + WSL2 + Ubuntu 22.04 + ROS 2 Humble.
The driver is Linux-only (the vendor ships a compiled `.so`), which is why WSL exists here.

**Controller web UI:** `http://192.168.58.2` — username `admin`, password `123`.
Useful for cross-checking joint angles against what ROS reports.

---

## 3. Hardware connection

1. Connect the Ethernet cable from the FAIRINO emergency-stop box to the laptop.
2. Switch on the controller (red switch).
3. Wait up to a minute. The gripper indicator below the arm should show **green**.
4. If it is not green, **release the emergency stop switch**.

Then set the laptop's Ethernet adapter to the `192.168.58.x` subnet (not `.2`, that's
the controller) — typically `192.168.58.10`, mask `255.255.255.0`.

---

## 4. One-time setup

Already completed on this machine. Recorded here so the project is reproducible from a
clean Ubuntu 22.04 install.

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

Check: `echo $ROS_DISTRO` prints `humble`.

### Clone the driver — pinned tag

```bash
mkdir -p ~/fairino_380_ws/src
cd ~/fairino_380_ws/src
git clone --branch V3.0.0_RobotV3.8.0 https://github.com/FAIR-INNOVATION/frcobot_ros2.git
cd frcobot_ros2
git describe --tags --always    # V3.0.0_RobotV3.8.0
git rev-parse --short HEAD      # 7c683cf
```

**Do not `git pull` to main.** This tag matches controller firmware v3.8.0.1. Newer
driver code may not.

### Confirm the controller IP compiled into the source

```bash
cd ~/fairino_380_ws/src/frcobot_ros2
grep -n 'CONTROLLER_IP' \
  fairino_hardware/include/fairino_hardware/data_type_def.h \
  fairino_hardware/include/fairino_hardware/fairino_hardware_interface.hpp
```

### Build

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fairino_hardware --symlink-install
source install/local_setup.bash
ros2 pkg prefix fairino_hardware
```

Expected: `2 packages finished` (`fairino_msgs`, `fairino_hardware`).

`-Wmaybe-uninitialized` warnings in `command_server.cpp` and
`1 package had stderr output: fairino_hardware` are **expected** in this tagged release.
Not errors. Do not try to fix them.

---

## 5. Every-session startup

Nothing below survives closing the terminal. Run it every time.

### Reconnect VS Code to Ubuntu

`Ctrl+Shift+P` → `WSL: Connect to WSL using Distro` → `Ubuntu-22.04`.
Bottom-left must read `WSL: Ubuntu-22.04`.

Faster: `Ctrl+R` → pick `fairino_380_ws [WSL: Ubuntu-22.04]`.

### Check the network

```bash
ip route
ping -c 3 192.168.58.2
```

Then check the feedback port is open:

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

If the ping fails, stop. Nothing else will work. See §11.

### Terminal A — start the command server

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
ros2 run fairino_hardware ros2_cmd_server
```

Leave this running. It prints `socket create failed` on startup — **expected noise in
this release**, not a failure, as long as the checks in §6 pass.

### Terminal B — everything else

Split the terminal in VS Code (`Ctrl+Shift+5`), then:

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
```

Every new terminal needs both `source` lines. `.bashrc` only handles the first one.

---

## 6. Verify communication

In Terminal B, before any motion:

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
```

Expect the topics `/nonrt_state_data`, `/parameter_events`, `/rosout` and the service
`/fairino_remote_command_service`.

Read the robot state:

```bash
timeout 10s ros2 topic echo /nonrt_state_data --once
```

Read one joint:

```bash
timeout 10s ros2 topic echo /nonrt_state_data --once --field j1_cur_pos
```

Check messages keep arriving:

```bash
timeout 10s ros2 topic hz /nonrt_state_data
```

**Cross-check `j1_cur_pos` … `j6_cur_pos` against the controller's own joint display**
(web UI or teach pendant). They must agree. Joint angles here are in **degrees**.

If they don't agree, you are not reading the robot you think you are. Stop.

---

## 7. Moving the robot

> Supervisor present. Checklist in §1 done. Area clear.

The pattern is always two steps: **register a target**, then **move to it**. Registering
is safe — it only stores numbers. Moving is not.

### Move to home position

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
python3 ~/fairino_380_ws/fairino_home.py
```

Home pose reference: `JNTPoint(1, 0, -90, 90, -90, -90, 0)`

### Joint move — rotate Joint 1 to +90°

Register the target:

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'JNTPoint(2,90,-90,90,-90,-90,0)'}"
```

Expect `cmd_res='0'`.

Then move — **check the path is clear first**:

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'MoveJ(JNT2,5,0,0)'}"
```

Verify:

```bash
ros2 topic echo /nonrt_state_data --once --field j1_cur_pos
```

Should approach 90°. Other joints stay near their targets.

**`MoveJ(target, speed, tool, user)`** — `JNT2` = registered joint target 2,
`5` = 5% speed, `0` = tool number, `0` = user/workpiece frame.

**`JNTPoint(id, j1, j2, j3, j4, j5, j6)`** — all six angles in degrees, always all six.

### Linear (Cartesian) move

Read the current pose first so you know where you're starting:

```bash
ros2 topic echo /nonrt_state_data --once
```

Register a Cartesian target — **all six values required**, XYZ position and ABC
orientation. This example moves 50 mm in −X from a pose near X = −182.799 mm:

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'CARTPoint(1,-232.799,-489.828,619.715,73.053,-41.118,61.738)'}"
```

Check what got stored:

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'GET(CART,1)'}"
```

Execute:

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'MoveL(CART1,5,0,0)'}"
```

`MoveL` moves the tool in a **straight line**. `MoveJ` doesn't — it interpolates in joint
space, so the tool path curves. A `MoveJ` that looks fine on paper can still swing an
elbow into something.

### Jog — small incremental motion

```
StartJOG(ref, joint, direction, speed, acceleration, distance)
```

| Argument | Meaning |
|---|---|
| `ref = 0` | Joint-space jog |
| `joint = 1` | Joint 1 |
| `direction = 1` | Positive joint direction |
| `speed = 5` | 5% speed |
| `acceleration = 5` | 5% acceleration |
| `distance = 1` | Max 1° displacement |

```bash
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface \
  "{cmd_str: 'StartJOG(0,1,1,5,5,1)'}"
```

Send **once**. Observe. Compare feedback before and after. Record the starting angle
before you jog so you can get back.

This argument order is for the V3.8.0 C++ SDK wrapper. Other language bindings differ.

### Joint directions (from the lab log)

| Joint | Positive direction |
|---|---|
| J1 | Anticlockwise |
| J2 | Anticlockwise |
| J3 | Anticlockwise |
| J4 | **Clockwise** |
| J5 | (not recorded) |
| J6 | **Clockwise** |

Getting a sign wrong sends the arm the opposite way from what you pictured. This is the
single easiest way to crash it.

---

## 8. Stopping the robot

**Physical emergency stop is always the real answer.** Software stops go through the same
link that may be the problem.

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

Have these typed into a spare terminal **before** you start moving, so stopping is one
Enter key, not thirty seconds of typing.

---

## 9. Robot parameters

### DH parameters (FR5)

| Joint | θᵢ | aᵢ (mm) | dᵢ (mm) | αᵢ |
|---|---|---|---|---|
| 1 | θ₁ | 0 | 152 | +90° (π/2) |
| 2 | θ₂ | −425 | 0 | 0° |
| 3 | θ₃ | −395 | 0 | 0° |
| 4 | θ₄ | 0 | 102 | +90° (π/2) |
| 5 | θ₅ | 0 | 102 | −90° (−π/2) |
| 6 | θ₆ | 0 | 100 | 0° |

Link lengths 425 mm + 395 mm dominate the 922 mm reach. All six θ are variable (revolute).

These are the numbers to enter into RoboAnalyzer (in metres: 0.152, −0.425, −0.395,
0.102, 0.102, 0.100) for forward and inverse kinematics analysis.

**Forward kinematics:** joint angles → tool pose. Chain the six transforms.
**Inverse kinematics:** tool pose → joint angles. Multiple solutions exist; the controller
picks one, which is why a `MoveL` can reach the right point in an unexpected arm
configuration.

Cross-check: run FK on the current joint angles from `/nonrt_state_data` and compare
against the Cartesian fields in the same message. If they match, your DH table is right.

### FR5 specification

| Parameter | Value |
|---|---|
| Payload | 5 kg |
| Reach | 922 mm |
| Repeatability | ±0.03 mm |
| Max joint speed | 180°/s |
| Typical TCP speed | 1 m/s |
| Weight | ≈22 kg |
| Communication | TCP/IP, Modbus, Profinet, EtherNet/IP, EtherCAT |

### Joint limits

| Axis | Soft limit | Hard limit |
|---|---|---|
| 1 | +175° / −175° | +179° / −179° |
| 2 | +85° / −265° | +89° / −269° |
| 3 | +150° / −150° | +152° / −152° |
| 4 | +85° / −265° | +89° / −269° |
| 5 | +175° / −175° | +179° / −179° |
| 6 | +175° / −175° | +179° / −179° |

**Check every target against this table before sending it.** Note axes 2 and 4 are
asymmetric — −265° is valid, +265° is not.

### Frames

Seven coordinate frames: Coordinate 0 at the base, 1–6 at each joint. See
`docs/frames.png`. Frame orientation determines what "positive X" means in a `CARTPoint`
— it is not the room's X.

---

## 10. Status

### Verified on hardware

- Ethernet link to controller at `192.168.58.2`
- Feedback TCP port 8081 reachable
- Driver builds from tag `V3.0.0_RobotV3.8.0`
- `ros2_cmd_server` starts and registers its service
- `/nonrt_state_data` publishes live joint and Cartesian feedback
- `JNTPoint` + `MoveJ` executes real motion
- `CARTPoint` + `MoveL` executes real linear motion
- Home position script runs
- Joint rotation directions recorded for J1–J4, J6

### Not yet done

- **RViz / MoveIt 2 on real hardware.** The RViz session run so far used
  `mock_components/GenericSystem` — MoveIt was planning for a *simulated* arm, not the
  FR5. Closing this is the main remaining work: load `fairino_hardware`'s real hardware
  interface in the `ros2_control` config so MoveIt's `FollowJointTrajectory` controller
  drives the physical robot.
- **`/joint_states`.** `/nonrt_state_data` is a custom message type, not
  `sensor_msgs/JointState`. MoveIt and RViz need the latter — something must bridge or
  replace it.
- **Python demo node.** Currently motion is typed as `ros2 service call` by hand. The
  deliverable needs a Python node. `fairino_home.py` is the starting point.
- **J5 direction** not recorded.
- **URDF check.** The vendor ships `fairino10_v6_moveit2_config` — for an FR10, not an
  FR5. Link lengths must be verified against the DH table in §9 before MoveIt planning is
  trustworthy.

---

## 11. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ping 192.168.58.2` fails | Cable; controller off; laptop NIC not on `192.168.58.x`; check `ip route` |
| Gripper indicator not green | Emergency stop engaged — release it |
| Port 8081 unreachable but ping works | Controller still booting; wait a minute and retry |
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `Package 'fairino_hardware' not found` | `source ~/fairino_380_ws/install/local_setup.bash` in *this* terminal |
| `socket create failed` at server start | Expected in this release. Confirm with §6 checks |
| `-Wmaybe-uninitialized` warnings | Expected. Vendor code. Ignore |
| Service call hangs | `ros2_cmd_server` not running in Terminal A |
| `cmd_res='0'` but no motion | Command accepted, not executed. Check e-stop, controller mode, joint limits. **Do not re-send blindly** |
| Robot moves wrong direction | Joint direction sign — see §7 table |
| VS Code lost WSL connection | Normal between sessions. `Ctrl+Shift+P` → `WSL: Connect to WSL using Distro` |

---

## Repository layout

```
fairino_380_ws/
├── src/
│   └── frcobot_ros2/          vendor driver @ V3.0.0_RobotV3.8.0
│       ├── fairino_hardware/  driver + ros2_cmd_server
│       ├── fairino_msgs/      RemoteCmdInterface.srv
│       └── fairino10_v6_moveit2_config/
├── .gitignore                 excludes build/ install/ log/
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
