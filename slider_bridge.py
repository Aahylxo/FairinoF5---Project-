#!/usr/bin/env python3
"""
slider_bridge.py  —  drive the FR5 from the joint_state_publisher_gui sliders.
No MoveIt anywhere.

    sliders (joint_state_publisher_gui)  ->  /slider_joint_states
        -> this script: soft limits, small speed-limited steps, safety checks
        -> /fairino5_controller/joint_trajectory
        -> ros2_control -> FairinoHardwareInterface -> ServoJ -> arm

THIS SCRIPT MOVES THE ROBOT.

How it moves the arm
--------------------
Ten times a second (only while the sliders are moving) it sends the
controller a short straight segment towards the sliders: at most SEGMENT_DEG
(1 deg) from where the controller's reference is right now, timed so every
joint moves at a constant max_speed.

- Constant speed: the controller runs with open_loop_control
  (slider_controllers.yaml), so each new segment starts from its last
  COMMANDED position. That is predictable, so the bridge models it exactly
  (class Segment) and times the next segment from there. An earlier version
  sent tiny steps every 40 ms with a fixed timing margin; the speed pulsed
  25 times a second and the arm vibrated.
- Each goal is stamped with its send time. Unstamped, the controller holds
  still for one 10 ms cycle whenever a goal arrives -- a 10-25 Hz stutter
  that was the vibration in every earlier version.
- Nothing is sent while the sliders are still. Re-sending the same point
  made the arm vibrate in place.
- A segment is at most 1 deg, so if the control loop ever stalls, catching
  up can jump at most 1 deg in one cycle (~125 deg/s, under the FR5's 180).
  A big goal plus a stall is what tripped "command speed in the joint space
  of axis 6 exceeds the limit" in testing.
- /joint_states pauses for ~1 s at a time on this setup. During a pause it
  goes at most GAP_LEAD_DEG (1 deg) past the last reported position, then
  waits: a pause may be the control loop stalling, and a bigger lead would be
  executed as one jump when it wakes. So when feedback pauses, motion
  pauses -- the cure for that is fixing the pauses, not a bigger lead. No feedback for STALE_FAULT seconds = stop for good.
- /joint_states is subscribed BEST-EFFORT. A reliable subscription received
  it in ~1 s bursts on WSL while `ros2 topic hz` showed a clean 100 Hz.
- ROS callbacks run on the main thread, between ticks (spin_once).

Why it is not just "point the slider GUI at the controller"
-----------------------------------------------------------
1. The GUI publishes on /joint_states -- the same topic as the robot's REAL
   feedback. Two publishers on one topic and everything downstream reads a
   position the arm is not in. So the GUI is started here, remapped onto
   /slider_joint_states.
2. The GUI starts every slider at 0 -- the arm stretched straight out. So it
   is started with its sliders preset to where the arm ACTUALLY is, and
   nothing is sent until every slider is within PICKUP_DEG of the real arm.
3. A slider can be flicked 180 degrees in a tenth of a second. The goal's
   duration is computed from the distance, so the arm still moves at
   max_speed, never faster.

Safety checks: soft limits (j3 +/-150, not the URDF's +/-162), all-zeros and
out-of-range feedback refused, no feedback for STALE_FAULT seconds = stop,
arm not following = stop. Faults LATCH: nothing more is sent until restart.

Usage (robot already running: ros2 launch ~/fairino_380_ws/rviz_real.launch.py)
-----
    python3 slider_bridge.py --dry-run         # compute and print, send nothing
    python3 slider_bridge.py                   # asks you to type YES
    python3 slider_bridge.py --max-speed 3     # slower (deg/s per joint)

Ctrl+C or closing the slider window stops sending; the arm finishes its
last tiny step (under a degree) and holds. The physical e-stop is the real
stop.
"""

import argparse
import math
import os
import subprocess
import sys
import threading
import time

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
CONTROLLER_TOPIC = "/fairino5_controller/joint_trajectory"
SLIDER_TOPIC = "/slider_joint_states"

# FR5 soft limits, degrees -- the robot's own, NOT the URDF's (the URDF gives
# j3 as +/-162; the FR5 is +/-150). Same table as fr5_kinematics.py.
SOFT_LIMITS_DEG = [
    (-175.0, 175.0),
    (-265.0, 85.0),
    (-150.0, 150.0),
    (-265.0, 85.0),
    (-175.0, 175.0),
    (-175.0, 175.0),
]
LIMIT_MARGIN_DEG = 2.0          # never command closer than this to a limit

LOOP_HZ = 25.0
SEND_PERIOD = 0.1               # at most 10 goals a second
SEGMENT_DEG = 1.0               # a goal is never more than this from where the
                                # controller's reference is right now -- so a
                                # stalled loop catching up can jump at most 1 deg
SPEED_FACTOR = 0.9              # time each segment for 90 % of max_speed
FRESH_LEAD_DEG = 1.5            # goal never this far past the arm (fresh feedback)
GAP_LEAD_DEG = 1.0              # ... or past its LAST KNOWN position while
                                # feedback is paused. Kept small on purpose: a
                                # feedback pause may be the control loop
                                # itself stalling (it publishes /joint_states),
                                # and whatever is queued then gets executed in
                                # one jump when it wakes up. 1 deg in one 8 ms
                                # cycle is ~125 deg/s, under the FR5's 180.
SEND_EPS_DEG = 0.01             # send only when the goal changes by more

MAX_SPEED_DEG_S = 5.0           # default, per joint
MAX_SPEED_CAP = 20.0            # refuse anything faster on the command line
PICKUP_DEG = 3.0                # sliders must be this close to the arm to arm
STATE_TIMEOUT = 0.25            # older than this: hold, send nothing new
STALE_FAULT = 10.0              # no feedback for this long = stop for good

# Arm-not-following detector. With ~1 s feedback gaps on this robot, it looks
# over a longer window: asking for more than FROZEN_DEMAND_DEG of motion, and
# the reported position has not changed by FROZEN_MOVE_DEG in FROZEN_SECONDS.
FROZEN_DEMAND_DEG = 2.0
FROZEN_MOVE_DEG = 0.1
FROZEN_SECONDS = 3.0


# ----------------------------------------------------------- pure functions
# Kept free of ROS so they can be tested on their own.

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def clamp_to_limits(q_deg):
    out = []
    for d, (lo, hi) in zip(q_deg, SOFT_LIMITS_DEG):
        out.append(clamp(d, lo + LIMIT_MARGIN_DEG, hi - LIMIT_MARGIN_DEG))
    return out


class Segment:
    """Where the controller's reference is, modelled exactly.

    With open_loop_control the JointTrajectoryController starts every new
    goal from its own last COMMANDED position and moves in a straight line
    to the goal over time_from_start. That is deterministic, so the bridge
    can know where the reference is at any instant without asking -- and
    time the next goal from there, giving a constant speed.
    """

    def __init__(self, q, now):
        self.start = list(q)
        self.goal = list(q)
        self.t0 = now
        self.dur = 1e-6

    def at(self, now):
        f = clamp((now - self.t0) / self.dur, 0.0, 1.0)
        return [s + (g - s) * f for s, g in zip(self.start, self.goal)]

    def restart(self, goal, now, max_speed):
        """New goal from the current reference. Returns its duration."""
        ref = self.at(now)
        dist = max(abs(g - r) for g, r in zip(goal, ref))
        dur = max(0.05, dist / (max_speed * SPEED_FACTOR))
        self.start, self.goal, self.t0, self.dur = ref, list(goal), now, dur
        return dur


def plan_goal(target, ref, anchor, anchor_lead):
    """The next goal: toward the sliders, but

    - at most SEGMENT_DEG from the controller's current reference
    - at most anchor_lead from where the arm last reported being
    - inside the soft limits
    """
    goal = []
    for t, r, a in zip(target, ref, anchor):
        g = clamp(t, r - SEGMENT_DEG, r + SEGMENT_DEG)
        g = clamp(g, a - anchor_lead, a + anchor_lead)
        goal.append(g)
    return clamp_to_limits(goal)


def feedback_problem(actual_deg, simulation):
    """Reason string if /joint_states is obviously not the real arm, else None."""
    if all(abs(d) < 1e-6 for d in actual_deg) and not simulation:
        return ("/joint_states reads all zeros - the driver is not reading "
                "the arm (wait for \"Successful 'activate'\")")
    for i, d in enumerate(actual_deg):
        lo, hi = SOFT_LIMITS_DEG[i]
        if d < lo - 1.0 or d > hi + 1.0:
            return f"/joint_states says j{i + 1} = {d:.1f} deg, outside its limits"
    return None


def pickup_gaps(target, actual):
    """Per-joint |slider - arm| in degrees."""
    return [abs(t - a) for t, a in zip(target, actual)]


def detect_simulation():
    """True if the ros2_control xacro is set to the mock robot.

    Unreadable file -> assume real hardware (keeps the all-zeros guard on).
    """
    path = os.path.expanduser(
        "~/fairino_380_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/"
        "config/fairino5_v6_robot.ros2_control.xacro")
    try:
        with open(path) as fh:
            return "mock_components/GenericSystem" in fh.read()
    except OSError:
        return False


# ------------------------------------------------------------------ ROS part
def run(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    def by_name(msg):
        """JointState -> [j1..j6] in degrees, matched by NAME.

        The FR5 driver publishes j1, j2, j4, j5, j3, j6 -- by index would
        swap three joints.
        """
        found = {n: p for n, p in zip(msg.name, msg.position) if n in JOINTS}
        if len(found) != 6:
            return None
        return [math.degrees(found[n]) for n in JOINTS]

    class Bridge(Node):
        def __init__(self):
            super().__init__("slider_bridge")
            self.lock = threading.Lock()
            self.actual = None
            self.actual_stamp = 0.0
            self.max_gap = 0.0           # longest /joint_states gap, seconds
            self.slider = None
            self.slider_stamp = 0.0
            # BEST-EFFORT, like `ros2 topic hz`. A RELIABLE subscriber here
            # received ~1 message a second: when DDS drops a sample, reliable
            # delivery holds back everything after it until the repair, and
            # under WSL that stalled for ~1 s at a time. For a 100 Hz state
            # stream the newest sample is all that matters, so never wait.
            self.create_subscription(JointState, "/joint_states", self.on_state,
                                     qos_profile_sensor_data)
            self.create_subscription(JointState, SLIDER_TOPIC, self.on_slider,
                                     qos_profile_sensor_data)
            self.pub = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 10)

        def on_state(self, msg):
            q = by_name(msg)
            if q is not None:
                now = time.time()
                with self.lock:
                    if self.actual_stamp:
                        self.max_gap = max(self.max_gap, now - self.actual_stamp)
                    self.actual, self.actual_stamp = q, now

        def on_slider(self, msg):
            q = by_name(msg)
            if q is not None:
                with self.lock:
                    self.slider, self.slider_stamp = q, time.time()

        def take_gap(self):
            with self.lock:
                g, self.max_gap = self.max_gap, 0.0
                return g

        def grab(self):
            with self.lock:
                return (self.actual, self.actual_stamp,
                        self.slider, self.slider_stamp)

        def send(self, q_deg, duration):
            msg = JointTrajectory()
            # Stamp = the moment we sent it. With a zero stamp the controller
            # starts the trajectory at its first sample, so it holds still for
            # one cycle every time a goal arrives -- at 10-25 goals a second
            # that is a 10-25 Hz stutter, felt as vibration. With the send
            # time, it picks up mid-motion and carries straight on.
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = list(JOINTS)
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(d) for d in q_deg]
            pt.time_from_start = Duration(sec=int(duration),
                                          nanosec=int((duration % 1.0) * 1e9))
            msg.points = [pt]
            self.pub.publish(msg)

    rclpy.init()
    node = Bridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def pump(seconds):
        """Run ROS callbacks on this thread for `seconds`."""
        end = time.time() + seconds
        while True:
            left = end - time.time()
            if left <= 0:
                return
            executor.spin_once(timeout_sec=left)

    gui = None
    try:
        simulation = detect_simulation()
        print(f"\n  slider bridge  -  {'SIMULATION (mock robot)' if simulation else 'REAL ROBOT'}"
              f"{'  -  DRY RUN, sending nothing' if args.dry_run else ''}")
        print(f"  max speed {args.max_speed:.1f} deg/s per joint\n")

        # ---- 1. wait for real, sane feedback
        print("  waiting for /joint_states ...")
        t0 = time.time()
        while True:
            actual, a_stamp, _s, _ss = node.grab()
            if actual is not None and time.time() - a_stamp < STATE_TIMEOUT:
                break
            if time.time() - t0 > 10.0:
                print("  no /joint_states in 10 s - is rviz_real.launch.py running?")
                return
            pump(0.05)

        problem = feedback_problem(actual, simulation)
        if problem:
            print(f"  REFUSING: {problem}")
            return
        print("  arm is at  " + "  ".join(f"{n}={d:7.1f}" for n, d in zip(JOINTS, actual)))

        if not simulation and not args.dry_run and not args.yes:
            print("\n  This will move the REAL FR5. Supervisor present, hand on the"
                  " e-stop, area clear, pendant in AUTO?")
            if input("  Type YES to continue: ").strip() != "YES":
                print("  not confirmed - nothing sent")
                return

        # ---- 2. start the slider GUI, remapped, sliders preset to the real pose
        if not args.no_gui:
            cmd = ["ros2", "run", "joint_state_publisher_gui",
                   "joint_state_publisher_gui", "--ros-args",
                   "-r", f"joint_states:={SLIDER_TOPIC}",
                   "-r", "__node:=slider_gui"]
            for n, d in zip(JOINTS, actual):
                cmd += ["-p", f"zeros.{n}:={math.radians(d):.6f}"]
            gui = subprocess.Popen(cmd)
            print(f"  slider window starting (publishing on {SLIDER_TOPIC}) ...")

        pump(0.3)                       # fresh feedback after the YES prompt
        actual = node.grab()[0]
        control_loop(args, node, gui, simulation, actual, pump)

    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n  stopped")
    finally:
        if gui is not None and gui.poll() is None:
            gui.terminate()
            try:
                gui.wait(timeout=3)
            except subprocess.TimeoutExpired:
                gui.kill()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def control_loop(args, node, gui, simulation, actual, pump):
    def trip(reason):
        print(f"\n  FAULT - nothing more will be sent: {reason}"
              "\n  Ctrl+C, fix it (check http://192.168.58.2 for an alarm),"
              " and restart.")
        return reason

    armed = False
    fault = None
    seg = Segment(actual, time.time())
    last_send = 0.0
    last_print = 0.0
    last_a_stamp = 0.0
    frozen_ref = None              # (time, position) the detector compares to
    period = 1.0 / LOOP_HZ
    last = time.time()

    while True:
        pump(max(0.0, last + period - time.time()))
        now = time.time()
        dt = max(1e-3, now - last)
        last = now

        if gui is not None and gui.poll() is not None:
            print("\n  slider window closed - stopping")
            return
        if not node.context.ok():
            return

        actual, a_stamp, slider, s_stamp = node.grab()
        if fault or actual is None:
            continue

        age = now - a_stamp
        new_sample = a_stamp != last_a_stamp
        last_a_stamp = a_stamp

        # ---- feedback checks
        if new_sample:
            problem = feedback_problem(actual, simulation)
            if problem:
                fault = trip(problem)
                continue
        paused = age > STATE_TIMEOUT
        if paused:
            if armed and age > STALE_FAULT:
                fault = trip(f"no /joint_states for {age:.1f} s")
                continue
            if not armed:
                continue                           # only arm on fresh feedback

        if slider is None or now - s_stamp > 1.0:
            continue                               # no slider input
        target = clamp_to_limits(slider)

        # ---- pick-up: sliders must match the arm before anything moves
        if not armed:
            gaps = pickup_gaps(target, actual)
            if max(gaps) <= PICKUP_DEG:
                armed = True
                seg = Segment(actual, now)
                frozen_ref = (now, list(actual))
                print("\n  ARMED - the arm now follows the sliders")
            elif now - last_print > 1.0:
                last_print = now
                off = "  ".join(f"{n}:{g:5.1f}" for n, g in zip(JOINTS, gaps)
                                if g > PICKUP_DEG)
                print(f"  not armed - move these sliders to the arm (deg off): {off}")
            continue

        # ---- arm-not-following detector (skipped in a dry run)
        if new_sample and not paused and not args.dry_run:
            demand = max(abs(t - a) for t, a in zip(target, actual))
            ref_t, ref_q = frozen_ref
            moved = max(abs(a - r) for a, r in zip(actual, ref_q))
            if demand < FROZEN_DEMAND_DEG or moved > FROZEN_MOVE_DEG:
                frozen_ref = (now, list(actual))
            elif now - ref_t > FROZEN_SECONDS:
                fault = trip("arm is not following (alarm? manual mode? "
                             "speed box at 0?)")
                continue

        # ---- next goal: a short straight segment towards the sliders,
        # timed from where the controller's reference is RIGHT NOW so the
        # speed stays constant. (Earlier versions sent tiny steps every 40 ms
        # with a fixed timing margin; the speed pulsed 25 times a second and
        # the arm vibrated.)
        if now - last_send >= SEND_PERIOD:
            ref = seg.at(now)
            goal = plan_goal(target, ref, actual,
                             GAP_LEAD_DEG if paused else FRESH_LEAD_DEG)
            if max(abs(g - o) for g, o in zip(goal, seg.goal)) > SEND_EPS_DEG:
                dur = seg.restart(goal, now, args.max_speed)
                if not args.dry_run:
                    node.send(goal, dur)
                last_send = now

        if now - last_print > 1.0:
            last_print = now
            print("  cmd " + " ".join(f"{d:7.1f}" for d in seg.at(now))
                  + "   arm " + " ".join(f"{d:7.1f}" for d in actual)
                  + f"   gap {node.take_gap() * 1000:4.0f} ms")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true",
                    help="compute and print, publish nothing")
    ap.add_argument("--max-speed", type=float, default=MAX_SPEED_DEG_S,
                    help=f"deg/s per joint (default {MAX_SPEED_DEG_S}, max {MAX_SPEED_CAP})")
    ap.add_argument("--yes", action="store_true",
                    help="skip the YES prompt on real hardware")
    ap.add_argument("--no-gui", action="store_true",
                    help="do not start the slider window (start it yourself, "
                         f"remapped to {SLIDER_TOPIC})")
    args = ap.parse_args()
    if not 0.0 < args.max_speed <= MAX_SPEED_CAP:
        sys.exit(f"--max-speed must be between 0 and {MAX_SPEED_CAP}")
    run(args)


if __name__ == "__main__":
    main()
