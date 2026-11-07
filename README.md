# FAIRINO FR5 — ROS 2 Commissioning & Teleoperation

Robotics Mechanisms course project. Commissioning a FAIRINO FR5 collaborative arm,
driving it from ROS 2 Humble, and teleoperating it from human arm movement captured by an
Intel RealSense D455.

**Repo:** https://github.com/Aahylxo/FairinoF5---Project-

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

**[PART 5 — Hand tracking and teleoperation](#part-5--hand-tracking-and-teleoperation)**

**[PART 6 — Robot parameters](#part-6--robot-parameters)**

**[PART 7 — Status](#part-7--status)**

**[PART 8 — Troubleshooting](#part-8--troubleshooting)**

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

## 1.1 One-time install

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

The camera watches the operator and feeds video into ROS 2, so that human arm movement
can eventually drive the robot (Part 5).

**The camera plugs into the laptop by USB. The robot connects by Ethernet. These are two
completely separate paths** — the robot does not know the camera exists.

```
Laptop ─── Ethernet ──→ FR5 controller   (commands)
Laptop ─── USB ───────→ RealSense D455   (video)
```

**The obstacle:** WSL2 has no USB support by default. Windows owns the USB device, and
Ubuntu-inside-WSL cannot see it. The fix is `usbipd-win`, a free open-source tool that
forwards a USB device into WSL over a virtual connection.

## 4.2 Physical mount

Mounted on `wrist3_link` (the last link), lens pointing the same way as the tool.

- Cable routed with slack, taped along the forearm
- Must survive j6 rotating ±175° without snagging or pinching
- A yanked cable mid-motion drops the camera and needs a re-attach (4.4)

**Tape is fine for a test**, but the camera's pose relative to the flange is not
repeatable, so any coordinate work will drift when it shifts. For permanent use it needs
a rigid mount and a URDF link.

## 4.3 Install usbipd (one time, Windows)

Admin PowerShell:

```powershell
winget install --exact dorssel.usbipd-win
```

**Then close PowerShell and open a new admin window** — the installer adds `usbipd` to
PATH, and an already-open window still has the old PATH. This is why `usbipd list` fails
the first time.

## 4.4 Attach the camera (every session, Windows)

Plug the camera in, then in **admin** PowerShell:

```powershell
usbipd list
```

Find the Intel RealSense entry and note its BUSID:

```
BUSID  VID:PID    DEVICE                                        STATE
2-4    8086:0b5c  Intel(R) RealSense(TM) Depth Camera 455 ...   Not shared
```

```powershell
usbipd bind --busid 2-4
usbipd attach --wsl --busid 2-4
```

**`bind` is once. `attach` is every time** you unplug the camera, reboot, or restart WSL.

Confirm in Ubuntu:

```bash
lsusb | grep -i intel
```

Expect: `Bus 002 Device 002: ID 8086:0b5c Intel Corp. Intel(R) RealSense(TM) Depth Camera 455`

## 4.5 Install the driver (one time, Ubuntu)

```bash
sudo apt update
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description usbutils
```

This pulls in librealsense as a dependency.

## 4.6 Run the camera — reduced settings

**Do not use the default launch.** The D455's defaults are depth 848×480×30 and colour
1280×720×30, which is far more data than the virtual USB connection can carry. You get:

```
XXX Hardware Notification:Incomplete video frame detected!
Size 541448 out of 814335 bytes (66%),Warn,Frame Corrupted
```

Frames arriving half-finished, getting worse over time.

**Use this instead:**

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=480x270x15 \
  rgb_camera.color_profile:=640x360x15 \
  enable_infra1:=false \
  enable_infra2:=false
```

If corrupted frames still appear, drop depth entirely:

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=false \
  rgb_camera.color_profile:=640x360x15 \
  enable_infra1:=false \
  enable_infra2:=false
```

## 4.7 Verify the stream

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/camera/color/image_raw
```

Expect a steady ~15 Hz:

```
average rate: 14.506
  min: 0.053s max: 3.105s std dev: 0.13135s window: 4045
```

Note the topic is `/camera/camera/...` — the node nests the namespace.

An occasional large `max` is usually a startup hiccup. Repeated large gaps during arm
motion suggest the USB cable is being tugged.

To see the picture: add an **Image** display in RViz, set topic to the colour stream.

## 4.8 D455 limits — read before planning any vision work

| | |
|---|---|
| Ideal depth range | 0.6 m – 6 m |
| **Minimum depth distance** | **~0.52 m** |
| Depth accuracy | <2 % error at 4 m |
| Depth FOV | 87° × 58° |
| RGB FOV | 90° × 65° |
| Stereo baseline | 95 mm |

**The 52 cm minimum matters here.** The arm's total reach is 922 mm. With the camera on
the wrist, anything near the tool sits inside the blind zone — depth returns holes and
zeros. Wrist-mounted depth only works for things reasonably far from the gripper, not
close-up grasping. **Colour works at any distance**; only depth has the minimum.

**The IMU does not work over WSL.** The log shows `No HID info provided, IMU is disabled`
— USB/IP does not pass the HID interface through. There is no workaround in WSL.

---
---

# PART 5 — Hand tracking and teleoperation

**Goal:** the operator's right arm is tracked by the camera, and its movement drives the
robot's joints in real time.

**Status: Phase 1 complete. Phases 2 and 3 not started.**

## 5.1 Why this is built in phases

Pose estimation jitters. A landmark flickers a few pixels, the computed elbow angle jumps
15°, and in Method B that goes straight to `ServoJ` on a 22 kg arm. **The safety layer is
not a finishing touch — it is the project.**

| Phase | What it does | Robot involved? |
|---|---|---|
| **1** | Track the arm, draw a stick figure, compute angles | **No** |
| **2** | Map human angles → robot joints, with limits | **No** — prints only |
| **3** | Send to the robot, with the full safety layer | **Yes** |

## 5.2 Phase 1 — the tracker

**Install:**

```bash
pip3 install mediapipe --break-system-packages
sudo apt install -y ros-humble-cv-bridge
```

**Run** (with the RealSense node already running, Part 4.6):

```bash
source /opt/ros/humble/setup.bash
python3 ~/fairino_380_ws/arm_tracker.py
```

Options:

```bash
python3 ~/fairino_380_ws/arm_tracker.py --source webcam     # test without the RealSense
python3 ~/fairino_380_ws/arm_tracker.py --no-mirror         # unmirrored view
python3 ~/fairino_380_ws/arm_tracker.py --model-complexity 2  # slower, more accurate
```

`q` or `Esc` to quit.

**Stand back so your whole torso is in frame.** It needs both shoulders and both hips to
work out which way you are facing.

### What it measures

MediaPipe Pose finds 33 body landmarks. We use the right shoulder (12), elbow (14) and
wrist (16), plus the left shoulder and both hips to build a reference frame.

| Angle | 0° | 90° | 180° |
|---|---|---|---|
| **shoulder_elev** | arm hanging down | horizontal | straight up |
| **shoulder_azim** | out to your side | reaching forward | — (−90° = reaching back) |
| **elbow_flex** | fully folded | right angle | arm straight |
| **wrist_roll** | *unreliable — see below* | | |

### Two design decisions that matter

**Angles are computed in a frame attached to your torso, not the camera.** The script
builds x (out to your right), y (up your spine), z (forward from your chest) from the
shoulder and hip landmarks. So turning or stepping sideways does not change the numbers —
only actually moving your arm does. Without this, walking across the room would command
the robot.

**Lost tracking publishes `NaN`, not 0.** When a joint cannot be seen confidently it goes
red and reads `---`. Phase 3 must treat `NaN` as *freeze*. A zero would be a real command
to a real pose.

### Output

Published on `/human_arm_angles` as `Float32MultiArray`:

```
[shoulder_elev, shoulder_azim, elbow_flex, wrist_roll, tracking_ok]
```

so Phase 2 does not have to re-run the vision stack.

### The wrist problem

**MediaPipe's body model cannot see forearm rotation reliably.** It is inferred from a
few hand landmarks and is noisy. Shoulder elevation, shoulder azimuth and elbow flexion
are solid; wrist roll is not.

The project brief asks for three joints — shoulder, elbow, wrist — and the third is the
weak link. Options: map the two reliable shoulder angles plus elbow, or drive the wrist
from something else. **Worth raising with the instructor.**

## 5.3 Phase 2 — mapping (not built)

A configuration assigning each human angle to a robot joint:

- Which human angle → which robot joint
- Direction (can be inverted)
- Scaling (human range → robot range)
- Per-joint limits, clamped to the **FR5 soft limits** in Part 6 — not the URDF values,
  see the j3 discrepancy

Output: target joint angles, **printed only**. Verified against RViz in simulation before
anything is sent.

## 5.4 Phase 3 — the safety layer (not built)

Required before any of this touches hardware:

| Guard | Why |
|---|---|
| **Slew-rate limiting** | the robot moves far slower than your arm, regardless of how fast you move |
| **Smoothing filter** | absorbs pose jitter before it becomes motion |
| **Joint clamping** | hard limits at FR5 soft limits, applied after mapping |
| **Deadman key** | motion only while a key is held; release = stop |
| **Freeze on tracking loss** | `NaN` or `tracking_ok == 0` holds position, never jumps |
| **Startup pose match** | engage only when the human pose roughly matches the robot pose, or use relative mapping |

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

## Verified on hardware

- Ethernet link to controller at `192.168.58.2`
- Feedback TCP port 8081 reachable
- Driver builds from tag `V3.0.0_RobotV3.8.0`
- **Method A:** `ros2_cmd_server` runs; `JNTPoint`+`MoveJ` and `CARTPoint`+`MoveL`
  execute real motion; home position script runs
- **Method B:** real hardware plugin loads and reaches `active`; both controllers active;
  `/joint_states` matches the physical arm; joint-slider plan and execute performed
- Joint rotation directions recorded for J1–J4, J6
- URDF geometry cross-checked against the DH table
- **Camera:** D455 forwarded into WSL via usbipd; colour stream stable at ~14.5 Hz
  at 640×360×15
- **Phase 1 tracker:** right-arm skeleton and angles, publishing `/human_arm_angles`

## Not yet done

- **Phase 2 mapping** — human angle → robot joint configuration
- **Phase 3 safety layer** — slew limiting, smoothing, clamping, deadman, freeze
- **J5 direction** not recorded
- **j3 URDF limit** not yet corrected (±162° should be ±150°)
- **ServoJ rate mismatch** not yet corrected (100 Hz vs declared 8 ms)
- **Camera not in the URDF** — MoveIt does not know it exists, so collision checking
  ignores it and it does not appear in RViz
- **Depth stream** not validated under load; IMU unavailable in WSL
- **Python demo node** for Method A — motion is still typed by hand

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
| `Frame Corrupted` / `Incomplete video frame` | Too much bandwidth for virtual USB — use the reduced profiles in Part 4.6 |
| `No HID info provided, IMU is disabled` | Expected. USB/IP does not pass HID. No workaround in WSL |
| Depth returns holes near the tool | Closer than the 52 cm minimum — see Part 4.8 |
| Stream drops during arm motion | USB cable being tugged — check routing and slack |

## Tracker

| Symptom | Cause / fix |
|---|---|
| `mediapipe is not installed` | `pip3 install mediapipe --break-system-packages` |
| `cv_bridge is not installed` | `sudo apt install -y ros-humble-cv-bridge` |
| `No images on /camera/... yet` | RealSense node not running, or wrong topic — check `ros2 topic list \| grep camera` |
| `NO ARM DETECTED` | Stand further back — both shoulders and both hips must be visible |
| Angles read `---` | Landmark confidence below threshold — improve lighting, or lower `--min-visibility` |

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
├── arm_tracker.py                       Phase 1 human arm tracker
├── fairino_home.py                      move to home position
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
