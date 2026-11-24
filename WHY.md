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
[numpy 1.x](#why-numpy-126) ·
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
[DDS settings](#why-the-dds-settings)

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

*Something not explained here? `REFERENCE.md` has the full detail on everything.*
