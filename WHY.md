# Why

Every "why are we doing this?" answer, in one place. The other files link here so they can
stay short.

**Jump to:**
[Windows](#why-wsl) ·
[Ubuntu 22.04](#why-ubuntu-2204) ·
[ROS 2](#why-ros-2) ·
[Humble](#why-humble) ·
[two source lines](#why-two-source-lines) ·
[colcon build](#why-colcon-build) ·
[pinned versions](#why-pinned-versions) ·
[numpy 1.x](#why-numpy-1264) ·
[mediapipe 0.10.14](#why-mediapipe-01014) ·
[the plugin switch](#why-the-plugin-switch) ·
[RViz vs MoveIt](#why-rviz-and-moveit-are-different) ·
[DH parameters](#why-dh-parameters) ·
[inverse kinematics](#why-inverse-kinematics) ·
[the camera](#why-a-depth-camera) ·
[usbipd](#why-usbipd-every-time) ·
[pinch to move](#why-pinch-to-move) ·
[the speed limit](#why-the-robot-lags-behind-your-hand) ·
[the filters](#why-four-different-filters) ·
[the shaking](#why-the-arm-shook) ·
[one camera](#why-only-one-camera-program) ·
[DDS settings](#why-the-dds-settings) ·
**[error 14 — THE ANSWER](#why-the-robot-refuses-commands-error-14)** ·
[the update_rate mistake](#why-update_rate-went-to-125-and-back) ·
[sliders need a bridge](#why-the-sliders-need-a-bridge) ·
[slider vibration](#why-the-sliders-vibrated) ·
[/joint_states in bursts](#why-joint_states-arrived-in-bursts)

---

## Why WSL

The FAIRINO driver is only built for Linux. It will not run on Windows.

WSL is a real Ubuntu Linux running inside Windows. You get Linux without wiping your
laptop or dual booting.

**WSL2, not WSL1.** WSL1 cannot forward USB devices, so the camera would never work.

---

## Why Ubuntu 22.04

Because ROS 2 Humble only exists for 22.04.

This is not a preference. There is no Humble build for 24.04. If you install 24.04 you
will get a long way in and then hit a wall.

---

## Why ROS 2

ROS is a way for separate programs to talk to each other. Each program publishes messages
on a named channel, and any other program can listen.

That is why this project can have the camera, the hand tracker, the planner and the robot
driver as four separate things that do not know about each other. The robot driver does
not care that a hand is controlling it. It just receives joint positions.

Without ROS you would write one huge program instead.

---

## Why Humble

It is the last ROS 2 version the FAIRINO driver supports. Newer versions exist; the driver
does not work with them.

---

## Why two source lines

Every new terminal needs both:

```bash
source /opt/ros/humble/setup.bash                    # ROS itself
source ~/fairino_380_ws/install/local_setup.bash     # this project
```

The first tells the terminal where ROS is. The second tells it where **your** packages
are.

Miss the second and you get `Package 'fairino5_v6_moveit2_config' not found`. The error
message lists where it searched — if it only searched `/opt/ros/humble`, that is the
missing line.

It also breaks RViz in a confusing way: the robot's 3D model files are found through the
workspace, so you get the joint axes but **no arm**, with `RobotModel  Status: Error`.

---

## Why colcon build

The driver is C++ source code. It has to be compiled before it can run.

`colcon` is the tool that compiles everything in `src/` and puts the results in
`install/`.

**`--symlink-install`** makes `install/` point at the original files instead of copying
them. That is why you can edit a config file and just relaunch, with no rebuild. The
plugin switch in `RUN_REAL_ROBOT.md` depends on this.

You only build once, unless you change the C++ code.

---

## Why pinned versions

A "pin" is an exact version number instead of "whatever is newest".

Newest is how a project that worked last month stops working today. Someone publishes a
new release, your laptop picks it up, and something breaks for a reason that has nothing
to do with your code.

Everything is pinned in `requirements.txt`. `versions.sh` records what is actually
installed so you can compare two machines when one works and one does not.

---

## Why numpy 1.26.4

ROS Humble's packages were all compiled against numpy 1.x.

numpy 2 changed how compiled code talks to it. Install numpy 2 and those packages break
with:

```
AttributeError: _ARRAY_API not found
```

Install numpy **last**, after everything else, because other packages will quietly pull in
numpy 2 while installing.

---

## Why mediapipe 0.10.14

MediaPipe is Google's hand and body tracking library.

Version 1.0 deleted the `mp.solutions` API this project uses. On 1.x you get:

```
module 'mediapipe' has no attribute 'solutions'
```

0.10.14 is the last version that has it.

---

## Why the plugin switch

One line in one file decides whether ROS talks to a real robot or a pretend one.

```
mock_components/GenericSystem              <- simulation
fairino_hardware/FairinoHardwareInterface  <- real robot
```

**Simulation:** a fake robot that instantly moves wherever you tell it. Nothing physical.

**Real:** the driver opens an Ethernet connection to the controller.

**If it is set to real and no robot is attached**, the driver cannot connect, the
`ros2_control_node` process dies on startup, and then nothing works at all — no joint
positions, no MoveIt, every number on the teleop panel shows `---`. It looks like a
broken install. It is one wrong line.

**Always check this first when nothing works.**

---

## Why RViz and MoveIt are different

**RViz is a viewer.** An empty 3D window. You add things to it — a robot, camera data,
markers. It draws them. **It cannot command anything.**

**MoveIt is a planner.** It works out how to get the arm from A to B without tying itself
in knots. It has no window of its own, so it borrows RViz.

When you run `demo.launch.py`, you get RViz **with MoveIt's panel in it**. Everything you
see was added by MoveIt, not by you.

That is why there is a separate section for running RViz on its own: it shows you can use
the tool, not just the one window MoveIt sets up.

| | Draws the robot | Moves the robot |
|---|---|---|
| RViz alone | yes | **no** |
| RViz + MoveIt | yes | yes |
| Hand teleoperation | yes | yes |

---

## Why DH parameters

Four numbers per joint that describe the robot's shape — how long each piece is and how
each joint is angled relative to the last.

Six joints, four numbers each, and you have described the whole arm. It is a standard
notation, so any robotics person can read your table and know exactly what the arm looks
like.

Everything else is built on it. Without the table there is no way to calculate where the
hand ends up.

---

## Why inverse kinematics

Two directions, and only one is easy.

**Forward kinematics** — you know the six joint angles, you want the tool position. Just
multiply the matrices together. One answer, always.

**Inverse kinematics** — you know where you want the tool, you need the six angles. Much
harder. There can be **eight** different sets of angles that all put the tool in the same
place, and some of them are useless (elbow through the table).

The hand control needs the inverse direction. Your hand says "put the tool here" and
something has to turn that into six numbers. That is what `fr5_kinematics.py` does, and
it is tested on 2000 random positions with zero failures.

**This is also why the hand control works the way it does.** An earlier version mapped
your elbow angle onto the robot's elbow joint directly — simpler, no IK needed. But
mapping joints means **you have no idea where the tool ends up**. During testing it drove
the arm at the floor while every joint was within its limits, because joint limits do not
know where the floor is. Solving for tool position instead makes a floor one line of code.

---

## Why a depth camera

A normal webcam gives a flat picture. It cannot tell whether your hand is near or far.

A depth camera measures distance to every pixel. So your hand becomes a real 3D point,
and moving your hand 10 cm forward moves the robot a real 100 mm forward.

**The difference measured:**

| Guessing distance from how big your hand looks | 18.5 mm of jitter |
|---|---|
| **Measuring it with depth** | **~2 mm** |

About nine times better.

A webcam will not work at all here, by the way — laptop built-in cameras cannot be
forwarded into WSL reliably.

---

## Why usbipd every time

Windows owns the USB ports. WSL does not see them by default.

`usbipd` hands a specific device over to Linux. Two steps:

- **`bind`** — mark it as shareable. Done once, survives reboots.
- **`attach`** — actually hand it over. **Lost every reboot and every unplug.**

So `attach` is the one you redo every session. If the camera "stopped working", that is
almost always it.

---

## Why pinch to move

Because something has to be a deadman switch — a thing that stops the robot the moment
you stop actively asking for it.

Without one, the robot follows your hand the whole time, including when you reach for your
coffee.

Pinching is better than a button because your hand is already in front of the camera. Open
your fingers and it stops instantly.

**Releasing works like lifting a mouse off the mat.** The arm stays put. When you pinch
again it carries on from there, so you can reposition your arm comfortably without the
robot moving.

---

## Why the robot lags behind your hand

That is the speed limit, and it is deliberate.

Your hand can move 50 cm in a quarter second. A 22 kg robot arm cannot, and should not
try. The limit is **5°/s per joint** by default.

In RViz the green ball shows where your hand is sending the tool. The gap between the ball
and the arm **is** the lag. Ball ahead, arm catching up. That is correct.

---

## Why four different filters

They catch different problems, at different points. One alone is not enough.

| | Where it sits | What it catches |
|---|---|---|
| `hand_smooth_hz` | on the raw measurement | camera noise |
| `deadband` | on the target | the last bit of drift a filter cannot remove |
| `filter_hz` | after the IK | jitter the maths introduces |
| `max_speed` | at the output | everything else |

**The deadband is the interesting one.** A filter only *reduces* noise — it never gets to
zero. So the target creeps forever and the robot faithfully follows. A deadband says
"below 12 mm, do nothing at all".

Measured, holding still:

```
no filtering                  3.07 mm of wander
smoothing only                1.69 mm
smoothing + 12 mm deadband    0.00 mm
```

That is what makes holding still actually still.

---

## Why the arm shook

It happened twice. **Both were software faults, and both are now guarded against.**

### The first time — the command ran away from the arm

The code limited how fast the command could change, but measured that against **its own
previous command**, not against where the arm actually was.

In simulation the fake arm moves instantly, so the two never drift apart. On a real arm
there is lag. The command marched on anyway, and each new goal asked the robot to close
the whole accumulated gap at once.

Simulated, with a 5°/s limit: it demanded **80°/s**. The arm accelerates hard, overshoots,
gets corrected, overshoots back. Shaking in place.

**Fixed:** the command can never get more than a fraction of a degree ahead of the real
arm.

### The second time — the robot lied about where it was

`/joint_states` reported **all six joints at exactly 0.00°** while the arm was physically
at home. Not missing data. Wrong data, that looks completely valid.

This happens in the gap between `Starting ...please wait...` and
`Successful 'activate'` — the driver reports "I cannot read the arm" as a position of
zero.

And then the fix from the first time made it worse. That fix pins the command close to the
reported position — which was zero. **All-zeros means the arm stretched straight out
horizontally.** So it commanded the arm to leave home and straighten:

| | home | commanded |
|---|---|---|
| j2 | −90° | ~0° |
| j3 | +90° | ~0° |
| j4 | −90° | ~0° |

Those are exactly the three joints that shook.

**Fixed, two ways:** the code refuses to engage when the feedback reads all zeros, and it
stops by itself if it is commanding motion and the arm is not moving.

**This is why `RUN_REAL_ROBOT.md` makes you wait for `Successful 'activate'` and then
check the numbers before touching anything.**

---

## Why only one camera program

The camera can only be opened by one program at a time.

Start a second and you get `errno=16 Device or resource busy`, and it often breaks the USB
forwarding too — so you have to redo the `usbipd attach`.

`teleop.launch.py` starts its own camera. So if you already have one running, stop it
first.

```bash
pgrep -af realsense        # should print nothing before you start
```

---

## Why the DDS settings

```bash
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
```

ROS programs find each other through a messaging system called DDS. There is more than one
implementation.

A broken or leftover configuration file makes programs fail to start, or start and never
see each other. That happened on the lab workstation — `robot_state_publisher` restarting
in a loop with `rcl node's rmw handle is invalid`.

Setting it explicitly avoids the whole class of problem.

**Every terminal must match.** Two programs on different settings simply never see each
other, and **nothing reports an error** — which is what makes it horrible to diagnose.

```bash
echo $RMW_IMPLEMENTATION $ROS_LOCALHOST_ONLY   # same in every terminal
```

---

## Why the robot refuses commands (error 14)

# ⭐ THE ANSWER: the robot was in MANUAL mode.

**Switch it to AUTO. That is the whole fix.**

```bash
# check it -- needs ros2_cmd_server running
timeout 10s ros2 topic echo /nonrt_state_data --once --field robot_mode
```

| | |
|---|---|
| `0` | **AUTO** — ServoJ works, MoveIt works |
| `1` | **MANUAL** — ServoJ refused, MoveIt silently does nothing |

Change it on the pendant, or on the web interface at `http://192.168.58.2` — the hand
icon in the top bar, right of the speed box.

### Why it was so hard to see

Manual mode does not block everything. It blocks exactly one thing:

| In manual mode | |
|---|---|
| Reading `/joint_states` | ✅ works |
| `MoveJ` — one discrete command | ✅ works |
| **`ServoJ` — continuous streaming** | ❌ **refused, error 14** |

So the arm reports its position perfectly, Method A moves it fine, every status says
healthy — and **only MoveIt fails.** Which looks like a MoveIt problem, and isn't.

It is a sensible safety design: nobody should be able to stream motion into a robot that
someone may be stood next to, hand-guiding. It is just invisible from the computer.

### How it was proved

The run before the fix already had every other change in place — `update_rate` back at
100, joint limits lowered, the `ServoMoveStart` patch applied, the alarm cleared. It
failed. The next run differed in exactly one way, `robot_mode: 1 → 0`, and the arm moved.

**One variable.**

---

Below is everything that was investigated before the mode was found. The defects are
real and worth keeping — but none of them was the blocker.

### What it looks like

```
ServoJ指令下发错误,错误码:14
```

"ServoJ dispatch error, code 14." In one log it appeared **6,137 times** — about 250 a
second.

Meanwhile `/joint_states` reads perfectly, the hardware shows `active`, both controllers
are running, and **MoveIt reports every move as SUCCEEDED.** The arm does nothing, or
stops partway.

### What it means

Code 14 is FAIRINO's *"interface execution failed."* In practice: **the robot has
latched a fault and is refusing all motion** until it is cleared. Reads keep working,
which is what makes it so confusing.

### Where the actual message is

**Not in any terminal.** Open the robot's web interface:

```
http://192.168.58.2        username: admin     password: 123
```

The real message was sitting there the whole time:

```
[Error] The command speed in the joint space of Axis 4
        exceeds the limit and can be reset
```

**Click to clear it.** The arm moves again immediately.

> **This is the lesson.** Everything on the computer said the system was healthy. The one
> piece of information that mattered was on the robot's own screen. When a robot will not
> move and the software insists it is fine, **go and look at the robot.**

### Why it kept coming back

Clearing the alarm lets exactly one move through, then it faults again. Three causes
were found, each real:

1. **`joint_limits.yaml` allowed more speed than the robot accepts** — 180.5 and
   183.3 °/s against a rated **180 °/s**. j4, j5 and j6 were the worst, which is why the
   alarm always named Axis 4. Lowered to 171.9 °/s.
2. **No `ServoMoveStart()`** — the SDK requires it before streaming ServoJ and the vendor
   driver never calls it. Without the bracket the controller buffers each command
   separately and the commanded position runs ahead of the arm. Patched.
3. **`update_rate` at 125 Hz** — see below.

### Why nobody noticed for three sessions

**The driver returns OK when ServoJ is refused.** `write()` ignores the return code, so
ros2_control is told the command landed, MoveIt reports success, and the only evidence is
an INFO line among 250 identical ones per second.

The patch now logs **one** ERROR naming the fix, and says when ServoJ starts working
again.

---

## Why update_rate went to 125, and back

**It is at 100. If yours says 125, change it back.**

```bash
sed -i 's/update_rate: 125/update_rate: 100/' \
  ~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/ros2_controllers.yaml
```

### What happened

The driver calls `ServoJ(..., 0.008, ...)` — "reach this in 8 ms." The controller ran
every 10 ms. Two numbers describing one cycle and disagreeing, so the controller was
changed to 125 Hz to match.

### Why that was wrong

The robot computes speed as **delta ÷ cmdT**, with cmdT fixed at 8 ms. At 100 Hz a
command arrives every 10 ms but claims to need 8, so the arm arrives early and waits.
**The 2 ms is slack.**

WSL has no real-time kernel. That slack is what absorbs a late cycle. Without it, a
delayed cycle means a bigger position delta still labelled 8 ms — speed over the limit,
robot faults.

### The bigger mistake

**Nothing was failing when it was changed.** No symptom, no error. A live setting on a
22 kg arm was changed because two numbers looked untidy on the page, it was never tested,
and it went into the documentation as a "fix."

A working system is not an invitation to tidy it.

---

## Why the sliders need a bridge

The brief was "move the robot using only RViz" — the slider window, not MoveIt. Out of
the box that cannot work: `joint_state_publisher_gui` **publishes joint angles on
`/joint_states`**, and RViz draws them. Nothing reaches the controller, so only the
picture moves.

Pointing the slider window at the real robot directly would be dangerous, three ways:

1. **It would fight the real feedback.** `/joint_states` is also where the robot reports
   where it is. Two publishers on one topic and everything downstream reads positions the
   arm is not in. So `slider_bridge.py` starts the window **remapped** onto
   `/slider_joint_states`.
2. **Every slider starts at 0** — the arm stretched straight out, the pose behind the
   second shaking incident. The bridge starts the window with its sliders set to the
   arm's **real** pose, and sends nothing until every slider is within 3° of the arm.
3. **A slider can be flicked 180° in a tenth of a second.** The bridge never asks for
   more than 5 °/s per joint, and never more than 1° ahead of where the controller is.

`rviz_real.launch.py` starts the robot without `move_group`, so it is genuinely "RViz and
sliders, no MoveIt".

---

## Why the sliders vibrated

Getting the slider bridge smooth took several versions on the real arm. Each finding is
in the code now:

| Version | What the arm did | Cause |
|---|---|---|
| Stream a tiny step every 40 ms, hold when feedback pauses | Moved in jerks | Feedback paused ~1 s at a time (next section); every pause froze it |
| One whole goal per slider change, "like MoveIt" | Robot alarm: *command speed of axis 6 exceeds the limit* | A long goal plus any stall: on waking, the controller jumps to where the goal says it should be by now — 5° in one 8 ms cycle |
| Tiny steps again, re-sent every 40 ms even when still | **Vibrated with the sliders untouched** | Re-sending the same point restarts the controller's interpolation from the *measured* position every 40 ms — a back-and-forth pull |
| Tiny steps, sent only on change, open loop | Smooth path, still buzzing | **Every goal arriving unstamped makes the controller hold for one 10 ms cycle** before moving — a 10–25 Hz stutter |
| **Current:** 1° segments, 10 Hz, stamped, open loop, timed from a model of the controller | Smooth | — |

The two settings that matter, both standard ROS:

- **`open_loop_control: true`** (`slider_controllers.yaml`) — the JointTrajectoryController
  starts each new goal from its **last command**, not the noisy measured position. Only
  `rviz_real.launch.py` uses it; MoveIt keeps the vendor's file.
- **`header.stamp` = send time** on every goal — the controller picks the motion up
  mid-way instead of restarting it.

Because open loop makes the controller predictable, the bridge models it exactly
(`class Segment`) and times each segment from where the reference is *now*, so the speed
stays constant.

---

## Why /joint_states arrived in bursts

With only the robot running, `ros2 topic hz /joint_states` showed a clean **100 Hz,
max gap 12 ms**. Yet a script subscribing to it received it in **bursts about once a
second**.

The difference was the subscription's QoS:

- `ros2 topic hz` subscribes **best-effort** — take whatever arrives, never wait.
- The scripts subscribed **reliable** (the default when you pass `10`). If DDS drops one
  sample, a reliable subscription holds back everything after it until the lost one is
  re-sent — on WSL, roughly a second.

For a 100 Hz state stream only the newest sample matters, so `slider_bridge.py`,
`nudge_j1.py` and `hand_teleop.py` now subscribe with `qos_profile_sensor_data`
(best-effort). The pauses did not vanish completely in every session — a clean restart of
everything helped most — which is why the slider bridge still treats any feedback pause
as a possible control-loop stall and limits itself to 1° until feedback returns.

This is also the most likely explanation for the old "teleop reports `FROZE`" fault.

---

*Something not explained here? `REFERENCE.md` has the full detail on everything.*
