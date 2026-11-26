# FAIRINO FR5 — Robot Arm Project

Control a 6-axis robot arm with ROS 2, MoveIt and RViz — with **RViz sliders alone**, or
**with your hand**, through a depth camera.

BITS Pilani Dubai Campus · Robotics Mechanisms

---

## 👉 Already set up? Go straight to running it

### **[▶ RUN SIMULATION](RUN_SIMULATION.md)** — no robot, nothing can break
### **[▶ RUN THE REAL ROBOT](RUN_REAL_ROBOT.md)** — read it all before you start

### **[📖 HOW IT WORKS](HOW_IT_WORKS.md)** — understand the whole thing from scratch

---

## What this does

| | |
|---|---|
| 🤖 **Move the arm** | Drag it in a 3D view, press Plan, press Execute |
| 🎚 **RViz sliders, no MoveIt** | Drag a slider, the real arm glides there |
| ✋ **Hand control** | A camera watches your hand. The arm follows it |
| 📐 **The maths** | Forward and inverse kinematics, tested on 2000 positions |
| 👁 **See it** | The robot, the workspace limits and the camera data, all in 3D |

---

## What you need

- Windows 10 or 11
- About **20 GB** free
- *(for hand control)* an Intel RealSense camera — D435 or D455
- *(for the real robot)* an FR5, an Ethernet cable, **and a supervisor**

---

# SETUP

Do this **once**. About an hour, most of it waiting for downloads.

> 💡 **Wondering why any of this?** → **[WHY.md](WHY.md)**

## 1. Turn on Windows features

Open **PowerShell as administrator** (right-click → Run as administrator):

```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

**Restart your computer.**

## 2. Install Linux

PowerShell as administrator again:

```powershell
wsl --install -d Ubuntu-22.04
```

It asks for a **username** and **password**. Pick something simple. **Write the password
down** — you need it for every `sudo` command, and nothing appears on screen as you type
it.

> ⚠️ Must be **Ubuntu-22.04**. Not 24.04. [Why?](WHY.md#why-ubuntu-2204)

## 3. Install ROS 2

In your Ubuntu window, paste this whole block:

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

Takes 10–20 minutes.

## 4. Install the extra packages

```bash
sudo apt install -y \
  ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-realsense2-camera ros-humble-realsense2-description \
  ros-humble-joint-state-publisher-gui \
  python3-pip python3-tk
```

## 5. Make every terminal ready

```bash
cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
EOF
source ~/.bashrc
echo "$ROS_DISTRO $RMW_IMPLEMENTATION $ROS_LOCALHOST_ONLY"
```

**Should print:** `humble rmw_fastrtps_cpp 1`

> 💡 [Why those last three lines?](WHY.md#why-the-dds-settings)

## 6. Get this project

```bash
git clone https://github.com/Aahylxo/FairinoF5---Project-.git ~/fairino_380_ws
cd ~/fairino_380_ws
ls
```

**Should show:** `hand_teleop.py`, `fr5_kinematics.py`, `src`, and the guides.

## 7. Install the Python packages

```bash
cd ~/fairino_380_ws
pip3 install -r requirements.txt
pip3 install "numpy==1.26.4"
```

> ⚠️ That second line is not a mistake. numpy must be installed **last**.
> [Why?](WHY.md#why-numpy-1264)

## 8. Build it

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

Takes 5–15 minutes. Warnings are fine. **Errors are not.**

**Should end with:** `Summary: N packages finished`

> 💡 [What is colcon doing?](WHY.md#why-colcon-build)

## 9. Check it worked

```bash
cd ~/fairino_380_ws
bash versions.sh | tee VERSIONS.txt
```

**Should show:** `Ubuntu 22.04`, `humble`, `numpy 1.26.4`, `mediapipe 0.10.14`.

Some lines are **supposed** to look wrong:

| Line | Fine? |
|---|---|
| `git tag  NO .git FOLDER` | ✅ normal |
| `CAMERA ... NOT FOUND` | ✅ normal, no camera plugged in |
| `192.168.58.2  not reachable` | ✅ normal, no robot connected |

## 10. Done — run it

```bash
cd ~/fairino_380_ws
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

**A window opens with an orange robot arm.** That is it working.

### 🎉 Now go to **[RUN SIMULATION](RUN_SIMULATION.md)**

---

## 🔧 Setup troubleshooting

Only read this if something above went wrong.

| Problem | Fix |
|---|---|
| **`wsl` is not recognized** | Step 1 did not run, or you did not restart. Do both again |
| **`WslRegisterDistribution failed`** | Virtualisation is off in your BIOS. Reboot into BIOS and enable it (often called VT-x, SVM, or Virtualisation) |
| **I forgot my Ubuntu password** | In PowerShell: `wsl -u root`, then `passwd yourusername` |
| **`404 Not Found` installing a package** | Your package list is stale. `sudo apt update` first |
| **`E: Unable to locate package ros-humble-...`** | Step 3 did not finish. Run it again and watch for errors |
| **`--break-system-packages: no such option`** | Leave that flag off. Ubuntu 22.04's pip is older and does not have it |
| **`colcon: command not found`** | `sudo apt install -y python3-colcon-common-extensions` |
| **Build fails with a compiler error** | You are out of disk or memory. Close other programs, free some space, try again |
| **`Package 'fairino5_v6_moveit2_config' not found`** | You missed `source install/local_setup.bash`. Needed in **every** terminal. [Why?](WHY.md#why-two-source-lines) |
| **`AttributeError: _ARRAY_API not found`** | numpy got upgraded. `pip3 install "numpy==1.26.4"` |
| **`module 'mediapipe' has no attribute 'solutions'`** | `pip3 install "mediapipe==0.10.14"` |
| **VS Code lost the WSL connection** | Normal. `Ctrl+Shift+P` → `WSL: Connect to WSL using Distro` |
| **`rmw handle is invalid`, or things restarting in a loop** | Step 5, in every terminal |
| **Nothing works, everything shows `---`** | Check the plugin — see below |

### The one that catches everybody

If the robot launches but **everything shows `---`** and nothing moves:

```bash
grep plugin ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

**For simulation it must say `mock_components/GenericSystem`.** If it says
`fairino_hardware/FairinoHardwareInterface`, it is trying to talk to a robot that is not
there, and then nothing works at all.

```bash
cd ~/fairino_380_ws
sed -i 's|fairino_hardware/FairinoHardwareInterface|mock_components/GenericSystem|' \
  src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

> 💡 [Why does one line do that?](WHY.md#why-the-plugin-switch)

---

# The files

| File | What it is |
|---|---|
| **[RUN_SIMULATION.md](RUN_SIMULATION.md)** | How to run it with no robot |
| **[RUN_REAL_ROBOT.md](RUN_REAL_ROBOT.md)** | How to run it on the real arm |
| **[HOW_IT_WORKS.md](HOW_IT_WORKS.md)** | **The whole project explained from nothing** — start here to understand it |
| **[WHY.md](WHY.md)** | Why any of this is the way it is |
| **[REFERENCE.md](REFERENCE.md)** | Full detail on everything |
| **[FOLLOW_ALONG.md](FOLLOW_ALONG.md)** | The long 49-step setup, with every failure mode |
| `hand_teleop.py` | The hand control |
| `teleop.launch.py` | Camera + hand control in one command (`depth:=false` for colour only) |
| `slider_bridge.py` | RViz slider window → real arm, smooth and guarded |
| `rviz_real.launch.py` | The robot + plain RViz, **no MoveIt** — used with the slider bridge |
| `slider_controllers.yaml` | Controller config for that launch (`open_loop_control` on) |
| `fr5_kinematics.py` | The maths — forward and inverse kinematics |
| `nudge_j1.py` | Move one joint a known amount, safely |
| `fairino_home.py` | Send the arm home through Method A |
| `arm_tracker.py` | Phase 1 vision: your arm's angles on `/human_arm_angles`, moves nothing |
| `patch_driver.sh` | Fixes two defects in the vendor's C++ driver |
| `versions.sh` | Records every version installed |

---

# Where the project is

## ✅ Working

- Setup, build, versions recorded
- **Typed commands move the real arm**
- RViz on its own, and with MoveIt
- Forward and inverse kinematics — **2000 / 2000, zero failures**
- Hand control in simulation
- **Hand control moved the real arm**
- **MoveIt plans and executes on the real arm** — confirmed 26 Nov, once the robot was
  switched out of manual mode
- **RViz sliders move the real arm with no MoveIt** — smooth, 26 Nov
  ([how](RUN_REAL_ROBOT.md#part-5b--rviz-sliders-on-the-real-arm-no-moveit))

## ⏳ Left to do

| | |
|---|---|
| **Camera data in RViz** | Needs a RealSense plugged in. ~10 minutes |
| **Screenshots** | Four that matter. Nothing blocking it |
| **`/joint_states` pauses** | Mostly explained and worked around ([why](WHY.md#why-joint_states-arrived-in-bursts)); still seen now and then on WSL |
| **Hand control: stamped goals** | Give `hand_teleop.py` the same stamped, open-loop goals as the slider bridge |

## ⭐ If the arm will not move and everything says it is fine

**The robot is probably in MANUAL mode. Switch it to AUTO.**

In manual the robot still reports its position, still accepts `MoveJ`, and still shows
every status as healthy — but **refuses the `ServoJ` stream MoveIt uses.** So RViz plans,
reports SUCCEEDED, and nothing happens.

```bash
# needs ros2_cmd_server running
timeout 10s ros2 topic echo /nonrt_state_data --once --field robot_mode
```

**`0` = auto (good). `1` = manual (nothing will move).**

Change it on the pendant, or at `http://192.168.58.2` — the hand icon in the top bar.

[The full story](WHY.md#why-the-robot-refuses-commands-error-14) — it cost three days.

## ⚠️ Other known problems

- **A latched fault also blocks motion** and looks identical from the computer. Check
  `http://192.168.58.2` (admin / 123) for a red alarm and clear it
- **`/joint_states` has failed twice** — once publishing nothing, once publishing all
  zeros while the arm was at home. [What happened](WHY.md#why-the-arm-shook)
- **Subscribers can see `/joint_states` pause for ~1 s** even when the robot publishes a
  clean 100 Hz. Best-effort subscriptions mostly fix it; a clean restart of everything
  helps. [Why](WHY.md#why-joint_states-arrived-in-bursts)
- **The camera is not in the robot model** — so MoveIt does not know it is there and
  would plan straight through it
- **The camera needs WSL kernel 6.x.** An older kernel has no `uvcvideo` module, so
  there is no `/dev/video*` and librealsense finds nothing even though `lsusb` sees it.
  `wsl --update`, or use a machine with a newer kernel

## 🔧 Fixed in the vendor's code

| What | Where |
|---|---|
| `ServoJ` streamed without the required `ServoMoveStart()` | `patch_driver.sh` |
| A refused `ServoJ` logged at INFO 250×/s while `write()` returned OK | `patch_driver.sh` |
| `joint_limits.yaml` allowed 183 °/s against the FR5's rated 180 °/s | lowered to 171.9 |
| `update_rate` changed to 125 — **a mistake, since reverted to 100** | [why](WHY.md#why-update_rate-went-to-125-and-back) |
| j3 limit in the URDF was ±162° (the FR16/20/30 figure); the FR5's is ±150° | corrected in `fairino5_v6.urdf` |
| The repo was committed with the plugin on real hardware | back on simulation, as it must be |

---

# ⛔ Before you touch the real robot

- **Someone qualified must be in the room**
- **Know where the e-stop is without looking**
- **If the arm shakes — hit the e-stop. Do not wait.**

It has shaken twice, and vibrated during slider testing. All were software faults, all are
fixed and guarded against: [shaking](WHY.md#why-the-arm-shook) ·
[vibration](WHY.md#why-the-sliders-vibrated).

**Read [RUN_REAL_ROBOT.md](RUN_REAL_ROBOT.md) completely before your first run.**

---

## The robot

**FAIRINO FR5** · 6 axes · 5 kg payload · 922 mm reach · controller at `192.168.58.2`

## Versions

Ubuntu 22.04 · ROS 2 Humble · Python 3.10 · numpy 1.26.4 · mediapipe 0.10.14 ·
driver tag `V3.0.0_RobotV3.8.0`

Run `bash versions.sh` for the full list from your own machine.
