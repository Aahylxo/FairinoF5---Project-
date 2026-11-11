# FR5 — Follow Along

**Start here.** This takes a Windows laptop with nothing installed and ends with a robot
arm you control by moving your hand.

Do the steps in order. Don't skip. Each step says what you should see — if you don't see
it, stop and read the **If it goes wrong** box under that step.

Every command is meant to be copied and pasted whole.

| Section | What you get | Time | Robot needed? |
|---|---|---|---|
| [A — Windows](#a--windows-setup) | Linux running on the laptop | 20 min | no |
| [B — Ubuntu](#b--ubuntu-setup) | ROS 2 installed | 30 min | no |
| [C — The code](#c--get-the-code) | project downloaded and built | 10 min | no |
| [D — Simulation](#d--run-the-simulation) | a robot arm on screen | 2 min | no |
| [E — Camera](#e--add-the-camera) | your arm driving the simulated robot | 10 min | no |
| [F — Real robot](#f--the-real-robot) | your arm driving the real FR5 | 15 min | **yes** |

**Sections A to E need no robot at all.** You can do the whole thing on any Windows
laptop with a RealSense camera.

The full explanation of everything here is in `README.md`. This document is only the
steps.

---
---

# A — Windows setup

## Step 1 — Check your Windows version

Press `Win + R`, type `winver`, press Enter.

You need **Windows 10 build 19041 or later**, or any Windows 11.

## Step 2 — Open PowerShell as administrator

Press `Win + X`, choose **Terminal (Admin)** or **Windows PowerShell (Admin)**.

**Everything in section A happens in this window.** It must say *Administrator* in the
title bar.

## Step 3 — Install WSL and Ubuntu 22.04

```powershell
wsl --install -d Ubuntu-22.04
```

**You should see:** a download, then a window asking you to create a UNIX username and
password.

The password shows nothing as you type. That is normal. Write the password down.

> **If it goes wrong**
>
> `wsl is not recognized` → the feature is off. Run these two, restart the laptop, then
> try Step 3 again:
> ```powershell
> dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
> dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
> ```
>
> `Windows Subsystem for Linux must be updated` → let it finish updating, restart if
> asked, then run Step 3 again.

## Step 4 — Confirm it installed

```powershell
wsl -l -v
```

**You should see:**

```
  NAME            STATE           VERSION
* Ubuntu-22.04    Stopped         2
```

**It must say 22.04 and VERSION 2.** Not 24.04 — ROS 2 Humble will not install on 24.04.

## Step 5 — Install VS Code

Download from `https://code.visualstudio.com`, install with the defaults.

## Step 6 — Install the WSL extension

Open VS Code → Extensions (`Ctrl+Shift+X`) → search **WSL** → install the one by
Microsoft.

## Step 7 — Connect VS Code to Ubuntu

In VS Code press `Ctrl+Shift+P`, type `WSL: Connect to WSL using Distro`, choose the
option **without** "in New Window", then choose **Ubuntu-22.04**.

**You should see:** the bottom-left corner of VS Code showing `WSL: Ubuntu-22.04`.

First time it downloads a small server into Ubuntu. Wait for it.

> This connection does **not** survive closing VS Code. Every session, repeat Step 7.
> Faster afterwards: `Ctrl+R` and pick the project from the recent list.

## Step 8 — Open a terminal inside Ubuntu

In VS Code press `` Ctrl+` `` (backtick, above Tab).

**You should see:** a prompt ending in `$` or `#`.

**Every command from here to the end of Section E goes in a VS Code terminal like this
one, not in PowerShell.** The few that need PowerShell say so explicitly.

---
---

# B — Ubuntu setup

## Step 9 — Add the ROS 2 software source

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

It may ask for the password you set in Step 3.

## Step 10 — Install ROS 2 Humble

```bash
sudo apt update && sudo apt install -y ros-humble-desktop
```

**This downloads about 2 GB. Allow 10–20 minutes.**

## Step 11 — Install everything else

```bash
sudo apt install -y git build-essential cmake \
  python3-colcon-common-extensions python3-rosdep \
  python3-pip python3-tk usbutils \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-moveit ros-humble-cv-bridge \
  ros-humble-realsense2-camera ros-humble-realsense2-description
```

Another large download. **`python3-pip` and `python3-tk` are not installed by default on
Ubuntu 22.04** — without them later steps fail.

## Step 12 — Make ROS load automatically

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "$ROS_DISTRO"
```

**You should see:** `humble`

> **If it goes wrong**
>
> Prints nothing → Step 10 did not finish. Run it again and watch for errors.

---
---

# C — Get the code

## Step 13 — Download the project

```bash
git clone https://github.com/Aahylxo/FairinoF5---Project-.git ~/fairino_380_ws
cd ~/fairino_380_ws
ls
```

**You should see:** `README.md`, `hand_teleop.py`, `fr5_kinematics.py`, `src`, and others.

> **It must go in `~/fairino_380_ws`.** The scripts and launch file use that exact path.

## Step 14 — Install the Python packages

```bash
cd ~/fairino_380_ws
pip3 install -r requirements.txt
```

**Do not add `--break-system-packages`.** Ubuntu 22.04's pip does not have that option
and the command will fail.

**The versions in `requirements.txt` are exact on purpose.** ROS 2 Humble needs numpy
1.x; current packages default to numpy 2 and break it. Do not install `mediapipe`,
`numpy` or `opencv` by hand.

> **If it goes wrong**
>
> `ReadTimeoutError` on a slow connection:
> ```bash
> pip3 install --default-timeout=120 --retries 10 -r requirements.txt
> ```
> Re-run as many times as needed; it resumes.

## Step 15 — Check the Python packages took

```bash
python3 -c "import numpy, mediapipe, cv2; print(numpy.__version__, mediapipe.__version__, cv2.__version__)"
```

**You should see:** `1.26.4 0.10.14 4.10.0` (the opencv number may differ slightly).

> **If it goes wrong**
>
> `_ARRAY_API not found`, or numpy reports 2.x → something upgraded numpy. Fix with:
> ```bash
> pip3 install "numpy==1.26.4"
> ```
> and check again. Numpy must be pinned **last**.

## Step 16 — Build

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

**Takes a few minutes.**

**You should see:** `Summary: N packages finished`.

**Compiler warnings are expected** — the vendor driver produces many
`-Wmaybe-uninitialized` warnings and a line saying
`1 package had stderr output: fairino_hardware`. These are not errors. Do not try to fix
them.

## Step 17 — Check the build

```bash
source ~/fairino_380_ws/install/local_setup.bash
ros2 pkg prefix fairino5_v6_moveit2_config
```

**You should see:** a path ending in `/install/fairino5_v6_moveit2_config`.

## Step 18 — Confirm you are set to the SIMULATED robot

```bash
grep plugin ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

**You should see:** `<plugin>mock_components/GenericSystem</plugin>`

> **If it says `fairino_hardware/FairinoHardwareInterface`**, you are pointed at a real
> robot. Switch back before continuing:
> ```bash
> sed -i 's|fairino_hardware/FairinoHardwareInterface|mock_components/GenericSystem|' \
>   ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
> ```

---
---

# D — Run the simulation

**No robot and no camera needed for this section.**

## Step 19 — Start the simulated robot

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

**You should see:** an RViz window with an orange robot arm.

**Leave this terminal running.** Everything else happens in new terminals.

> **If it goes wrong**
>
> `Package not found` and it only searched `['/opt/ros/humble']` → you missed
> `source install/local_setup.bash`. Both source lines are needed in **every** terminal.

## Step 20 — Check the controllers

Open a **new** terminal (`+` in the terminal panel):

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
ros2 control list_controllers
```

**You should see both lines saying `active`:**

```
fairino5_controller      joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster          active
```

You now have a working simulated FR5. **Sections A–D are the whole software install.**

Press `Ctrl+C` in the RViz terminal when you want to stop.

> You may see a wall of red text with `Segmentation fault` and `class_loader` when you
> press Ctrl+C. **That is a known MoveIt bug on exit and is harmless** — the lines above
> it will say the hardware shut down cleanly.

---
---

# E — Add the camera

You need an **Intel RealSense D455** plugged into the laptop by USB.

WSL cannot see USB devices by itself, so Steps 21–24 forward the camera into it.

## Step 21 — Install usbipd (once per laptop)

**In PowerShell as administrator:**

```powershell
winget install --exact dorssel.usbipd-win
```

**Then close PowerShell and open a new administrator window.** The installer changes the
PATH and the old window will not have it.

## Step 22 — Find the camera

Plug the camera in. **In the new administrator PowerShell:**

```powershell
usbipd list
```

**You should see** a line like:

```
2-4    8086:0b5c  Intel(R) RealSense(TM) Depth Camera 455 ...   Not shared
```

**Note its BUSID** (`2-4` above). It is different on every laptop and changes if you use
a different USB port. **Use your own number everywhere below.**

## Step 23 — Share the camera (once per laptop)

```powershell
usbipd bind --force --busid 2-4
```

## Step 24 — Attach the camera (EVERY SESSION)

```powershell
usbipd attach --wsl --busid 2-4
```

**You should see:** `usbipd list` now says **Attached** on that line.

> **This does not persist.** Repeat Step 24 after every reboot, every WSL restart, and
> every time you unplug the camera.

> **If it goes wrong**
>
> `Device busy (exported) ... used by Windows` → close the Windows Camera app, Teams,
> Zoom, and the RealSense Viewer, then run Step 23 again followed by Step 24.
>
> `There is no device with busid` → the camera is unplugged, or on a different port. Run
> `usbipd list` again and use the BUSID shown.

## Step 25 — Confirm Ubuntu sees the camera

Back in a VS Code terminal:

```bash
lsusb | grep -i intel
```

**You should see:** `Intel(R) RealSense(TM) Depth Camera 455`

Nothing → go back to Step 24.

## Step 26 — Check nothing else is using the camera

```bash
pgrep -af realsense
```

**Must print nothing.** If it prints something:

```bash
pkill -f realsense2_camera_node
```

> **Only ever run one camera program at a time.** Two at once gives
> `errno=16 Device or resource busy`, resets the camera, and breaks the attachment. You
> then have to redo Steps 24–26.

## Step 27 — Start the robot (if it is not already running)

Terminal 1:

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

## Step 28 — Start the camera and the controls

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/fairino_380_ws/teleop.launch.py
```

**You should see two new windows:**

- a video window showing you, with a green skeleton on your right arm
- a control panel titled *FR5 hand teleoperation*

> **If it goes wrong**
>
> Video window never appears, log full of `Timer expired` or `Device or resource busy` →
> the camera attachment went stale. Press `Ctrl+C`, then redo Steps 26, 24, 25, 28 in
> that order.
>
> `tkinter is not installed` → `sudo apt install -y python3-tk`
>
> `mediapipe is not installed` → go back to Step 14.

## Step 29 — Stand where the camera can see you

Stand about **2 metres back**, facing the camera, so your **whole upper body is in
frame**.

**You should see:** `TRACKED` in green on the video window, and `ready` in green on the
control panel.

> `cannot see your arm` → step further back. It needs both shoulders **and both hips**
> visible.

## Step 30 — Press the three buttons, in order

**1. Press `0. GO TO HOME POSE`.**

The arm in RViz moves to its home position. Wait until the panel says `at home`.
It moves slowly — that is deliberate.

**2. Press `1. CAPTURE NEUTRAL POSE`.**

Nothing moves. This records where your hand is and where the robot's tool is.

Stand comfortably with your arm out to the side at about chest height when you press it.

**3. Press `2. CLICK TO START MOVING`.**

The button turns red.

## Step 31 — Move the robot with your hand

- **Swing your arm left and right** → the robot's base turns the same way
- **Reach out and pull back** → the arm extends and retracts
- **Raise and lower your hand** → the tool goes up and down

The robot lags behind you. That is the speed limit working.

Press the button again, or press `Esc`, to stop.

> **If a direction feels backwards**, tick the matching **invert** box in the panel, then
> press `CAPTURE NEUTRAL POSE` again.
>
> **If it says `LIMIT:` constantly**, press `GO TO HOME POSE` and capture neutral again.
> Capturing while stretched out leaves no room to move.

**That is the whole system working.** Everything from here is the real robot.

---
---

# F — The real robot

> ## Read this before anything in Section F
>
> The FR5 weighs about 22 kg and reaches 922 mm. It can injure you and it can damage
> itself.
>
> - [ ] **A supervisor or instructor is in the room.** Not optional.
> - [ ] **The physical emergency stop is within reach of your hand**, and you know where
>       it is without looking.
> - [ ] **The whole swept area is clear** — including behind and below the arm.
> - [ ] You have already done Section E and the simulation works.
>
> **The physical e-stop is the only true stop.** Every software stop travels over the
> same network link that might be the problem.
>
> Everything in this section moves at **5 degrees per second per joint**, which is about
> 5 cm per second at the tool, roughly a thirty-sixth of the robot's maximum. Do not
> raise it.

## Step 32 — Power on, in this order

1. Turn on the **power supply**.
2. Turn on the **red switch on the black box** beside the robot.
3. **Wait up to one minute** for the controller to start.
4. Check the **indicator below the gripper is GREEN**.

Not green → **release the emergency stop switch**, then check again.

## Step 33 — Connect the network cable

Plug the Ethernet cable from the FAIRINO emergency-stop box into the laptop.

## Step 34 — Set the laptop's network address

**This is in Windows, not Ubuntu.**

Settings → Network & Internet → Ethernet → IP assignment → **Edit** → **Manual** → turn
**IPv4** on:

| Field | Value |
|---|---|
| IP address | `192.168.58.10` |
| Subnet mask | `255.255.255.0` |
| Gateway | leave blank |

`192.168.58.2` is the **robot's** address. Never set the laptop to that.

## Step 35 — Check the laptop can reach the robot

In a VS Code terminal:

```bash
python3 - <<'PY'
import socket
try:
    with socket.create_connection(("192.168.58.2", 8081), 3):
        print("port 8081 reachable")
except OSError as e:
    print("FAILED:", e)
PY
```

**You should see:** `port 8081 reachable`

> **FAILED → stop here.** Check the cable, that the controller is on and green, and that
> Step 34 is correct. Nothing below will work until this passes.
>
> (`ping` may fail even when this succeeds. WSL sometimes drops ping. **This test is the
> one that matters.**)

## Step 36 — Make sure nothing else holds the robot connection

```bash
pgrep -af ros2_cmd_server
```

**Must print nothing.** If it prints something, close that program first. Only one
program can talk to the controller at a time.

## Step 37 — Set a small safe working area

For a first run, keep the robot in a small envelope:

```bash
cd ~/fairino_380_ws
sed -i 's/"z_min": 180.0/"z_min": 300.0/; s/"r_max": 870.0/"r_max": 700.0/' hand_teleop.py
grep -A 6 "^WORKSPACE" hand_teleop.py
```

That keeps the tool at least 30 cm above the robot's base and within 70 cm of it.

**Look at the actual bench.** If there is a fixture, a laptop, a toolbox or cables under
the arm, raise `z_min` further by editing the number.

## Step 38 — Switch the software to the real robot

```bash
cd ~/fairino_380_ws
sed -i 's|mock_components/GenericSystem|fairino_hardware/FairinoHardwareInterface|' \
  src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
grep plugin src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

**You should see:** `<plugin>fairino_hardware/FairinoHardwareInterface</plugin>`

**This is the line that separates simulation from a real moving robot.** Always check it
with `grep` before launching, and switch it back when you are finished.

## Step 39 — Start the robot

Terminal 1:

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

**Read the startup text. You must see:**

```
机械臂SDK连接成功!            (SDK connection successful)
初始指令位置: <six numbers>
机械臂硬件启动成功!           (hardware started successfully)
```

> **STOP IF you see:**
>
> `机械臂SDK连接失败！请检查端口时候被占用` → something else holds the connection. Go
> back to Step 36.
>
> `读取初始关节角度错误，硬件无法启动！` → the software could not read the robot's
> position and **refused to start**. That is a safety feature working correctly. Fix the
> connection. **Never try to force past it.**

## Step 40 — Check the computer agrees with the robot

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

**You need all three:**

1. `plugin name: fairino_hardware/FairinoHardwareInterface` and `state: ... active`
2. both controllers `active`
3. joint values that **match the real robot**

For (3), multiply each number by **57.3** to get degrees and compare against the teach
pendant or the web interface at `http://192.168.58.2` (user `admin`, password `123`).

> **The joint names are NOT in order.** They are published as
> `j1, j2, j4, j5, j3, j6`. Match them by **name**, not by position in the list.

> **STOP IF the numbers disagree with the real robot.** You would be commanding a robot
> you cannot correctly see.

## Step 41 — Attach the camera

**In administrator PowerShell:**

```powershell
usbipd attach --wsl --busid 2-4
```

## Step 42 — Start the controls, in hold mode

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/fairino_380_ws/teleop.launch.py hold:=true scale:=0.5
```

**`hold:=true` means the robot only moves while you hold the mouse button down.** Letting
go stops it. Use this on real hardware.

Stand in frame and check the panel says `ready` in green.

## Step 43 — Check the path before homing

**`GO TO HOME POSE` moves every joint straight to the home position. It does not avoid
obstacles.**

Look at the arm. Look at where it would have to travel.

> **If the arm is in an awkward position, do not press it.** Ask your supervisor to move
> it near home first.

## Step 44 — First real movement

- Put your hand on the **physical e-stop**
- **Watch the arm, not the screen**
- Press `0. GO TO HOME POSE`

It takes 10–20 seconds.

> **Press the e-stop if** it moves toward anything, moves faster than a slow crawl, or
> moves a joint you did not expect.

Wait for the panel to say `at home`.

## Step 45 — Capture neutral

Stand comfortably, arm out to the side at chest height. Press
`1. CAPTURE NEUTRAL POSE`.

Nothing moves.

> If the message warns you are close to a limit, press `GO TO HOME POSE` again and
> recapture.

## Step 46 — First hand-controlled movement

**Decide what you are going to do before you start.** For example: *raise my hand five
centimetres.*

- Hand on the e-stop
- **Hold** the move button down
- Make that one small movement
- **Release the button**

Check the arm did what you intended.

## Step 47 — Practise stopping

Deliberately, while the arm is moving:

- let go of the button mid-movement — it should stop
- step out of the camera's view — it should say `FROZE: lost sight of your arm`

**Do this early, on purpose, while nothing is wrong.** Then you know it works before you
need it.

## Step 48 — Build up slowly

Make gradually larger movements. Stop and check after each one.

Do not raise the speed. Do not widen the working area until you have watched the robot
behave for a while.

## Step 49 — Shutting down

1. Release the button / press `Esc`
2. `Ctrl+C` in each terminal
3. Press the **physical e-stop** before putting hands near the robot
4. Turn off the red switch, then the power supply
5. Put the software back to simulation:

```bash
cd ~/fairino_380_ws
sed -i 's|fairino_hardware/FairinoHardwareInterface|mock_components/GenericSystem|' \
  src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

---
---

# Starting again next time

Everything above installs once. A normal session is:

**1.** Open VS Code → `Ctrl+Shift+P` → `WSL: Connect to WSL using Distro` → Ubuntu-22.04

**2.** Administrator PowerShell:
```powershell
usbipd attach --wsl --busid 2-4
```

**3.** Terminal 1:
```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

**4.** Terminal 2:
```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/fairino_380_ws/teleop.launch.py
```

**5.** `GO TO HOME POSE` → `CAPTURE NEUTRAL POSE` → `CLICK TO START MOVING`

---

# Quick fixes

| What you see | What to do |
|---|---|
| `wsl is not recognized` | Step 3's **If it goes wrong** box |
| `usbipd is not recognized` | Close PowerShell, open a new **administrator** one |
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `Package not found`, searched only `/opt/ros/humble` | `source ~/fairino_380_ws/install/local_setup.bash` in **this** terminal |
| `_ARRAY_API not found` | `pip3 install "numpy==1.26.4"` |
| `mediapipe is not installed` | Step 14 |
| `tkinter is not installed` | `sudo apt install -y python3-tk` |
| Camera missing from `lsusb` | Step 24 — attach does not persist |
| `Device or resource busy`, `errno=16` | Two camera programs running. Step 26, then 24 |
| `Device busy (exported)` | Close Windows camera apps, then Steps 23 and 24 |
| `cannot see your arm` | Stand further back; both shoulders and both hips must show |
| `LIMIT:` constantly | `GO TO HOME POSE`, then capture neutral again |
| A direction is backwards | Tick **invert**, then capture neutral again |
| `Segmentation fault` on Ctrl+C | Known MoveIt bug on exit. Harmless |
| Compiler warnings when building | Expected from the vendor driver. Not errors |

---

**Everything here is explained in full in `README.md`** — why each step exists, how the
system works, the robot's parameters, and the complete troubleshooting reference.
