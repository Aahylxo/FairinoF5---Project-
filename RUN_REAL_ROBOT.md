# RUN — THE REAL ROBOT

> # ⛔ READ ALL OF THIS BEFORE YOU START
>
> The FR5 weighs about **22 kg** and reaches **922 mm**. It can injure you and it can
> damage itself.
>
> - [ ] **A supervisor or instructor is in the room.** Not optional.
> - [ ] **Your hand is on the physical e-stop**, and you know where it is without looking.
> - [ ] **The whole swept area is clear** — including behind and below the arm.
> - [ ] **You have already done the same thing in simulation** (`RUN_SIMULATION.md`).
>
> **The physical e-stop is the only true stop.** Every software stop travels over the
> same network link that might be the problem.
>
> ### This arm has shaken violently twice. Both times it was a software fault.
>
> If it starts shaking or vibrating in place — **hit the e-stop immediately.** Do not wait
> to see whether it settles. Both causes are understood and guarded against, and the
> guards are in the current code, but the habit is what keeps you safe.
>
> Check you are on the current code:
> ```bash
> grep -c FROZEN_SECONDS ~/fairino_380_ws/hand_teleop.py
> ```
> **If that prints `0`, stop. Do not run on hardware.** Update first.

---

# PART 1 — Power up and connect

## Step 1 — Power on, in order

1. Mains to the controller box
2. **Red switch on the black box** → on
3. Wait for the **green indicator**
4. Teach pendant boots

## Step 2 — Check the pendant

Three things, and all three have caused the problem before:

- [ ] **E-stop released** — on the pendant *and* any external one
- [ ] **Mode is AUTO, not MANUAL** — in manual the pendant owns the robot and ignores the network
- [ ] **No application running on the pendant** — it holds the controller connection

## Step 3 — Network

Ethernet from the controller to the laptop. Laptop must be **192.168.58.1**, mask
**255.255.255.0**.

```bash
ip addr show | grep 192.168.58
ping -c4 192.168.58.2
```

**Four replies, under a millisecond.** No replies → cable, power, or laptop IP.

## Step 4 — Nothing else is holding the connection

```bash
pgrep -af ros2_cmd_server
pgrep -af ros2_control_node
pgrep -af hand_teleop
```

**All three must print nothing.** If any does:

```bash
pkill -f ros2_cmd_server
pkill -f ros2_control_node
pkill -f hand_teleop.py
pkill -f joint_state_publisher_gui
```

> `joint_state_publisher_gui` is on that list for a reason: it publishes `/joint_states`
> from sliders. Left running next to the real robot, two sources fight over the same
> topic and everything downstream reads a position the arm is not in.

---

# PART 2 — Start the software

## Step 5 — Switch to the real robot

```bash
cd ~/fairino_380_ws
sed -i 's|mock_components/GenericSystem|fairino_hardware/FairinoHardwareInterface|' \
  src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
grep plugin src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
```

**Must print:** `<plugin>fairino_hardware/FairinoHardwareInterface</plugin>`

No rebuild needed.

## Step 6 — Launch, and save the log

**Terminal 1:**

```bash
cd ~/fairino_380_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py 2>&1 | tee ~/launch.log
```

**Nothing should move.** Hand stays near the e-stop anyway.

### ⏳ WAIT for this line

```
Successful 'activate' of hardware 'FakeSystem'
```

(`FakeSystem` is the vendor's name for the component. It is the real driver — Step 5
proved that.)

**Between `Starting ...please wait...` and that line, the driver publishes all zeros.**
Engaging in that window is what made the arm shake. **Wait for it.**

> 💡 [The full story of both times it shook](WHY.md#why-the-arm-shook)

> **If `ros2_control_node` dies instead**, there will be a Chinese error about the SDK
> connection. That means the driver could not reach the controller — go back to Steps 2–4.
> Keep `~/launch.log`.

---

# PART 3 — Verify before you touch anything

**Terminal 2:**

```bash
source /opt/ros/humble/setup.bash
source ~/fairino_380_ws/install/local_setup.bash
```

## Step 7 — Is the feedback real?

```bash
ros2 topic echo /joint_states --once
```

At home, `position` should be roughly:

```
[0.00, -1.57, 1.57, -1.57, -1.57, 0.00]      (radians)
```

### 🔴 Six zeros means DO NOT PROCEED

All-zeros is the driver saying "I cannot read the arm" as if it were a position. It looks
valid and it is not. Wait longer, or relaunch.

> **The `name:` field comes out as `j1, j2, j4, j5, j3, j6` — not in order.** That is
> normal for this driver. Index 2 is j4 and index 4 is j3. The scripts match by name, so
> it is handled; anything you write yourself must do the same, or j3, j4 and j5 end up
> swapped.

## Step 8 — Is everything active?

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic info /joint_states --verbose | grep -i "publisher count"
```

- Both controllers **`active`**
- `hz` around **125**
- **`Publisher count: 1`** — more than one means something else is publishing, which is a
  hazard, not a nuisance

---

# PART 4 — First movement

Start with **one joint, 10 degrees**, from a script, so the move is exact and nothing
else can move.

```bash
cd ~/fairino_380_ws
python3 nudge_j1.py --dry-run     # prints what it would do, sends nothing
python3 nudge_j1.py               # asks you to type YES
```

It reads the arm, refuses if the feedback is bad or the move would break a joint limit,
then moves **2°/s** — a 10° move takes 5 seconds, so you have time to stop it.

```bash
python3 nudge_j1.py -- -10        # back again (note the --)
python3 nudge_j1.py --joint j6 --degrees 15
```

**If it reports it moved far less than asked**, the arm is not following. Stop and go back
to Step 2.

---

# PART 5 — MoveIt on the real arm

## Step 9 — Slow it down FIRST

RViz → **MotionPlanning** panel → **Planning** tab:

- **Velocity Scaling → 0.05**
- **Acceleration Scaling → 0.05**

**These reset to 1.0 on every launch.** At 1.0 this arm moves fast enough to hurt
someone. Set them before anything else, every time.

## Step 10 — Plan, look, then execute

1. **Start State → `<current>`**
2. Small goal — **Joints** tab, one joint, ~10°
3. **Plan**
4. **Look at the ghost trajectory.** Swinging wide, dipping toward the table, flipping the
   elbow? **Do not execute.** Plan again or choose a smaller goal.
5. **Execute**

> **The planning scene is empty.** MoveIt does not know about your table, the camera, or
> anything else in the cell, and will plan straight through them. **You are the collision
> checker.**

---

# PART 6 — Hand control on the real arm

Only after Parts 4 and 5 both worked, and after you have done the same thing in
simulation.

## Step 11 — Camera

**Admin PowerShell:**
```powershell
usbipd attach --wsl --busid 1-2
```
```bash
lsusb | grep -i intel
pgrep -af realsense          # must print nothing
```

## Step 12 — Start it in HOLD mode

```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/fairino_380_ws/teleop.launch.py \
    deadband:=20.0 height:=480 hold:=true scale:=0.5
```

| | Why |
|---|---|
| `hold:=true` | **Moves only while you hold the mouse button.** Let go and it stops. Not a toggle |
| `scale:=0.5` | Half motion, so mistakes are half the size |
| `height:=480` | D435 does not do 640×360 |

> 💡 [Why a deadman switch at all?](WHY.md#why-pinch-to-move)

## Step 13 — Check the panel agrees with the arm

**Before pressing anything**, the `actual` column must match the physical arm. At home:

```
j1 ~0    j2 ~-90    j3 ~+90    j4 ~-90    j5 ~-90    j6 ~0
```

### 🔴 If it reads `0.00` everywhere, or `BAD FEEDBACK — DO NOT ENGAGE`

**Stop.** The driver is lying about the position. `Ctrl+C`, go back to Step 7. The code
will refuse to engage — that is the guard working, not a bug to get around.

## Step 14 — Move it

**1. `0. GO TO HOME POSE`** — *look at the arm and the path first.* This is a joint-space
move and it does **not** avoid obstacles. Wait for `at home`.

**2. `1. CAPTURE NEUTRAL POSE`** — hand in a comfortable middle position

**3. `2. HOLD TO MOVE`** — hold the button down

**4. Pinch** thumb and index. Panel goes `MOVING`.

**Both gates must be satisfied** — button held *and* pinching. Release either and it
stops.

Start with small movements. Watch the arm, not the screen.

## Step 15 — Practise stopping

Before doing anything interesting, deliberately: release the button, release the pinch,
press `Esc`. Know what each feels like **before** you need it.

---

# If the arm shakes

1. **E-STOP.** Immediately.
2. `Ctrl+C` every terminal.
3. Check `/joint_states` — is it zeros, or real?
4. Check `Publisher count` is 1.
5. **Tell your supervisor.** They need to know, regardless of the cause.

Do not re-engage until the `actual` column matches the physical arm and the `error`
column is near zero.

---

# Shutting down

1. Stop the teleop (button, `Esc`)
2. `Ctrl+C` Terminal 2
3. `Ctrl+C` Terminal 1 — wait for `'deactivate'` and `'shutdown' hardware`
4. Pendant e-stop pressed
5. Power off at the black box

```powershell
usbipd detach --busid 1-2
```

### Set the repo back to simulation

```bash
cd ~/fairino_380_ws
sed -i 's|fairino_hardware/FairinoHardwareInterface|mock_components/GenericSystem|' \
  src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
git status
```

**Always commit with the plugin on simulation**, so anyone cloning the repo gets
something that runs without hardware.

---

# Known open fault

`/joint_states` has twice failed on this setup — once publishing nothing, once publishing
all zeros while the arm was at home. Both times the hardware component reported `active`
with all six interfaces `[claimed]`, so the status is not a reliable indicator.

**If it happens again, keep `~/launch.log`** and send the first 40 lines. README Part 7
has the full history.

---

**Full explanations:** `README.md`. **Simulation:** `RUN_SIMULATION.md`.
**Installing from scratch:** `FOLLOW_ALONG.md`.
