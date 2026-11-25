# RUN — SIMULATION

**No robot. Nothing can get hurt.** Use this to practise, to demonstrate, and to check
things work before going near the real arm.

Already installed? Start at Step 1. Never installed? Do `FOLLOW_ALONG.md` Sections A–C
first, once.

---

## ⚠ Step 0 — Check you are in SIMULATION mode

**Do this every single time.** It is one line and it is the single most common reason
nothing works.

```bash
grep plugin ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

**Must print:** `<plugin>mock_components/GenericSystem</plugin>`

If it says `fairino_hardware/FairinoHardwareInterface`, you are pointed at the real
robot. With no robot attached the driver cannot connect, so `ros2_control_node` dies on
startup — and then **nothing** works, in a way that looks like a broken install. Fix it:

```bash
cd ~/fairino_380_ws
sed -i 's|fairino_hardware/FairinoHardwareInterface|mock_components/GenericSystem|' \
  src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

No rebuild needed. Check it again before moving on.

> 💡 [Why does one line do this?](WHY.md#why-the-plugin-switch)

---

# PART 1 — The robot on screen

## Step 1 — Start it

**Terminal 1:**

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

**You should see:** an RViz window with an orange robot arm.

**Leave this terminal running.** Everything else goes in new terminals.

## Step 2 — Check it is really alive

**Terminal 2:**

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
ros2 topic hz /joint_states
```

**You should see:** a rate around **125**. `Ctrl+C` to stop watching.

> **If it says `does not appear to be published yet`** — the controller manager is not
> running. Check:
> ```bash
> ros2 node list | grep controller_manager
> ```
> Nothing? Then `ros2_control_node` died. **Go back to Step 0** — it is nearly always the
> plugin. Then:
> ```bash
> pkill -f spawner
> pkill -f ros2_control_node
> ```
> `Ctrl+C` Terminal 1 and start again.

## Step 3 — Move it with MoveIt

In RViz, find the **MotionPlanning** panel, bottom left. Missing? **Panels →
MotionPlanning**.

1. **Planning** tab → set **Velocity Scaling** and **Acceleration Scaling** to **0.05**
   *(they reset to 1.0 every launch — harmless here, but build the habit now)*
2. **Start State → `<current>`**
3. Drag the orange marker on the arm's end, **or** use the **Joints** tab sliders
4. Press **Plan** — a ghost arm shows the path
5. Press **Execute** — the arm follows it

**That is MoveIt: plan a path, then run it.**

---

# PART 2 — RViz on its own, no MoveIt

Worth being able to show. Step 1 opened RViz, but **MoveIt** configured it. This builds
the same view by hand.

Stop Terminal 1 first (`Ctrl+C`) so nothing clashes.

```bash
# terminal 1
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
xacro src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.urdf.xacro \
  > /tmp/fr5.urdf
ros2 run robot_state_publisher robot_state_publisher /tmp/fr5.urdf
```

```bash
# terminal 2
source /opt/ros/humble/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

```bash
# terminal 3   <-- BOTH source lines matter here
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
rviz2
```

**In RViz, set it up yourself:**

1. **Global Options → Fixed Frame** → `base_link`
2. **Add → RobotModel** → **Description Topic** → `/robot_description`
3. **Add → TF**

Drag a slider. The arm moves. **No MoveIt anywhere.**

> **Coloured axes but no arm, `RobotModel  Status: Error`** → you missed
> `source install/local_setup.bash` in terminal 3. The 3D model files are found through
> the workspace. Close RViz, source both lines, try again.

**The sliders move the picture, not a robot.** RViz only displays.

> 💡 [RViz vs MoveIt](WHY.md#why-rviz-and-moveit-are-different)

**Stop all three before Part 3** — `joint_state_publisher_gui` publishes `/joint_states`
and will fight anything else that does.

---

# PART 3 — Hand control

## Step 4 — Attach the camera

**In PowerShell as administrator:**

```powershell
usbipd list
usbipd attach --wsl --busid 1-2
```

Use **your own BUSID** from the `usbipd list` output. **This does not persist** — redo it
every session and after every unplug.

**In Ubuntu:**

```bash
lsusb | grep -i intel
pgrep -af realsense
```

First must show the camera. **Second must print nothing.**

```bash
pkill -f realsense2_camera_node     # only if the second printed something
```

> **Only ever one camera program at a time.** Two gives `Device or resource busy` and
> breaks the attachment — you then redo Step 4. [Why?](WHY.md#why-only-one-camera-program)

## Step 5 — Start the robot (if it is not already up)

**Terminal 1:**

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

## Step 6 — Start the hand control

**Terminal 2:**

```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/fairino_380_ws/teleop.launch.py deadband:=20.0 height:=480
```

`height:=480` is for the **D435** — it does not support 640×360. On a D455 you can leave
it out.

**You should see two windows:** a video window with your hand outlined, and a panel
titled *FR5 hand teleoperation*.

## Step 7 — Show the workspace in RViz (once)

**Add → By topic → `/fr5_workspace` → MarkerArray → OK**

Then **File → Save Config** so you never do it again.

**You should see:** a blue cylinder (reach limit), a red disc (floor), and a **green
sphere** — where your hand is sending the tool.

## Step 8 — Drive it

Hold your hand about **45 cm** from the lens. Wait for **TRACKED** in green.

**1. `0. GO TO HOME POSE`** — wait until it says `at home`

**2. `1. CAPTURE NEUTRAL POSE`** — hand in a comfortable **middle** position, not
stretched out

**3. `2. CLICK TO START MOVING`** — button turns red

> 💡 The simulated arm starts **stretched straight out** — that is the all-zeros pose,
> and it is correct. `GO TO HOME POSE` folds it into the proper working position.

**4. Pinch your thumb and index finger together.** The panel goes from `PINCH TO MOVE`
to `MOVING`. Open your fingers and it stops instantly.

| Your hand, while pinching | Robot |
|---|---|
| left / right | base rotates |
| toward / away from camera | arm extends / retracts |
| up / down | tool up / down |

Releasing works like lifting a mouse off the mat — the arm stays, and re-pinching carries
on from there. [Why pinch?](WHY.md#why-pinch-to-move)

**Stop:** the button, `Esc`, or `Space`.

---

## When something is wrong

| Panel says | What it means |
|---|---|
| Everything `---` | No `/joint_states`. Terminal 1 is not running, or `ros2_control_node` died — **Step 0** |
| `cannot see your hand` | Hand out of frame, fingers together, or poor light. Spread your fingers |
| `no depth here` | Hand closer than ~20 cm. Move it back. Still works, just less precisely |
| `LIMIT: reach` | At the edge of the workspace. Bring your hand toward the middle |
| `CAPTURE NEUTRAL` greyed out | Arm is not at home. Press GO TO HOME first |
| `capture a neutral pose` | You skipped button 1 |
| `BAD FEEDBACK - DO NOT ENGAGE` | The simulated robot starts at all-zeros, which is correct. If you see this in simulation, Step 0 is wrong — the plugin says real hardware. Fix it and relaunch, or add `--assume-sim` |
| Drifts while you hold still | Relaunch with `deadband:=25.0 hand_smooth_hz:=1.5` |

## Quick checks

```bash
ros2 topic hz /joint_states                                          # ~125
ros2 topic info /joint_states --verbose | grep -i "publisher count"  # must be 1
ros2 node list | grep controller_manager                             # must exist
pgrep -af hand_teleop                                                # only one
pgrep -af realsense                                                  # only one
```

## Shutting down

`Ctrl+C` each terminal. A wall of red with `Segmentation fault` and `class_loader` on
exit is a **known MoveIt bug and is harmless** — the lines above it say it shut down
cleanly.

```powershell
usbipd detach --busid 1-2
```

---

**Full explanations:** `README.md` — Part 3 for MoveIt, Part 3.7 for standalone RViz,
Parts 5.7 and 5.8 for the hand control.

**To run on the real robot:** `RUN_REAL_ROBOT.md`. **Read all of it first.**
