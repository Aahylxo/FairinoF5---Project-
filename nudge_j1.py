#!/usr/bin/env python3
"""
nudge_j1.py  —  move ONE joint of the FR5 by a known amount, slowly. The first
movement on the real arm (RUN_REAL_ROBOT.md Part 4).

THIS SCRIPT MOVES THE ROBOT.

Needs the robot running through ros2_control (Method B):
    ros2 launch fairino5_v6_moveit2_config demo.launch.py
or  ros2 launch ~/fairino_380_ws/rviz_real.launch.py

    python3 nudge_j1.py --dry-run             # prints what it would do, sends nothing
    python3 nudge_j1.py                       # j1 +10 deg; asks you to type YES
    python3 nudge_j1.py -- -10                # j1 back again (note the --)
    python3 nudge_j1.py --joint j6 --degrees 15
    python3 nudge_j1.py --speed 1             # slower (deg/s, default 2)

What it does
------------
1. Reads the arm from /joint_states (by joint NAME -- the driver publishes
   j1, j2, j4, j5, j3, j6).
2. Refuses if the feedback is bad (all zeros on real hardware, or a joint
   outside its limits) or if the move would end within 2 deg of a soft limit.
3. Sends ONE trajectory to fairino5_controller: that joint only, everything
   else held where it is, at --speed deg/s. 10 deg at 2 deg/s = 5 seconds,
   so there is time to hit the e-stop.
4. Waits, then reports how far the arm actually moved. If it moved far less
   than asked, the arm is not following (manual mode, an alarm, speed box 0).

Ctrl+C during the move sends "stop where you are now".
"""

import argparse
import math
import os
import sys
import time

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
CONTROLLER_TOPIC = "/fairino5_controller/joint_trajectory"

# FR5 soft limits, degrees (same table as fr5_kinematics.py / REFERENCE 6.3)
SOFT_LIMITS_DEG = {
    "j1": (-175.0, 175.0),
    "j2": (-265.0, 85.0),
    "j3": (-150.0, 150.0),
    "j4": (-265.0, 85.0),
    "j5": (-175.0, 175.0),
    "j6": (-175.0, 175.0),
}
LIMIT_MARGIN_DEG = 2.0
MAX_NUDGE_DEG = 45.0             # one nudge is a small, deliberate move
MAX_SPEED = 10.0                 # deg/s; refuse anything faster
STATE_TIMEOUT = 0.5


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


def feedback_problem(q, simulation):
    if all(abs(d) < 1e-6 for d in q.values()) and not simulation:
        return ("/joint_states reads all zeros - the driver is not reading the "
                "arm. Wait for \"Successful 'activate'\" and try again")
    for n, d in q.items():
        lo, hi = SOFT_LIMITS_DEG[n]
        if d < lo - 1.0 or d > hi + 1.0:
            return f"/joint_states says {n} = {d:.1f} deg, outside its limits"
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("amount", nargs="?", type=float, default=None,
                    help="degrees to move (default 10; negative = other way, after --)")
    ap.add_argument("--degrees", type=float, default=None,
                    help="same as the positional amount")
    ap.add_argument("--joint", default="j1", choices=JOINTS)
    ap.add_argument("--speed", type=float, default=2.0, help="deg/s (default 2)")
    ap.add_argument("--dry-run", action="store_true", help="print the plan, send nothing")
    ap.add_argument("--yes", action="store_true", help="skip the YES prompt")
    args = ap.parse_args()

    deg = args.degrees if args.degrees is not None else (
        args.amount if args.amount is not None else 10.0)
    if deg == 0.0 or abs(deg) > MAX_NUDGE_DEG:
        sys.exit(f"amount must be non-zero and at most {MAX_NUDGE_DEG:.0f} deg")
    if not 0.0 < args.speed <= MAX_SPEED:
        sys.exit(f"--speed must be between 0 and {MAX_SPEED:.0f} deg/s")

    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    class Nudge(Node):
        def __init__(self):
            super().__init__("nudge_joint")
            self.q = None
            self.stamp = 0.0
            # best-effort: see WHY.md#why-joint_states-arrived-in-bursts
            self.create_subscription(JointState, "/joint_states", self.on_state,
                                     qos_profile_sensor_data)
            self.pub = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 10)

        def on_state(self, msg):
            found = {n: math.degrees(p) for n, p in zip(msg.name, msg.position)
                     if n in JOINTS}
            if len(found) == 6:
                self.q, self.stamp = found, time.time()

        def send(self, q_deg, seconds):
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = list(JOINTS)
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(q_deg[n]) for n in JOINTS]
            pt.time_from_start = Duration(sec=int(seconds),
                                          nanosec=int((seconds % 1.0) * 1e9))
            msg.points = [pt]
            self.pub.publish(msg)

    rclpy.init()
    node = Nudge()
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    def pump(seconds):
        end = time.time() + seconds
        while time.time() < end:
            ex.spin_once(timeout_sec=max(0.0, end - time.time()))

    moving = False
    try:
        simulation = detect_simulation()
        print(f"\n  nudge  -  {'SIMULATION' if simulation else 'REAL ROBOT'}"
              f"{'  -  DRY RUN, sending nothing' if args.dry_run else ''}")

        t0 = time.time()
        while node.q is None or time.time() - node.stamp > STATE_TIMEOUT:
            if time.time() - t0 > 10.0:
                print("  no /joint_states in 10 s - is the robot launched?")
                return 1
            pump(0.05)

        start = dict(node.q)
        problem = feedback_problem(start, simulation)
        if problem:
            print(f"  REFUSING: {problem}")
            return 1

        goal = dict(start)
        goal[args.joint] = start[args.joint] + deg
        lo, hi = SOFT_LIMITS_DEG[args.joint]
        if not lo + LIMIT_MARGIN_DEG <= goal[args.joint] <= hi - LIMIT_MARGIN_DEG:
            print(f"  REFUSING: {args.joint} would end at {goal[args.joint]:.1f} deg, "
                  f"within {LIMIT_MARGIN_DEG:.0f} deg of its limit ({lo:.0f} .. {hi:.0f})")
            return 1

        seconds = max(1.0, abs(deg) / args.speed)
        print("  arm now   " + "  ".join(f"{n}={start[n]:7.1f}" for n in JOINTS))
        print(f"  plan      {args.joint} {start[args.joint]:.1f} -> {goal[args.joint]:.1f} deg"
              f"  ({deg:+.1f} deg at {args.speed:.1f} deg/s = {seconds:.1f} s),"
              " all other joints held")

        if args.dry_run:
            print("  dry run - nothing sent")
            return 0

        if not simulation and not args.yes:
            print("\n  This will move the REAL FR5. Supervisor present, hand on the"
                  " e-stop, area clear, pendant in AUTO?")
            if input("  Type YES to continue: ").strip() != "YES":
                print("  not confirmed - nothing sent")
                return 1
            pump(0.3)
            if feedback_problem(node.q, simulation) or any(
                    abs(node.q[n] - start[n]) > 1.0 for n in JOINTS):
                print("  REFUSING: the arm moved or the feedback changed while "
                      "waiting - run it again")
                return 1

        node.send(goal, seconds)
        moving = True
        end = time.time() + seconds + 2.0
        last = 0.0
        while time.time() < end:
            pump(0.1)
            if time.time() - last > 1.0:
                last = time.time()
                print(f"  {args.joint} = {node.q[args.joint]:7.1f} deg")
            problem = feedback_problem(node.q, simulation)
            if problem:
                print(f"\n  STOPPED WATCHING: {problem}. Hand on the e-stop.")
                return 1
        moving = False

        moved = node.q[args.joint] - start[args.joint]
        print(f"\n  moved {moved:+.1f} deg of {deg:+.1f} asked")
        if abs(moved) < 0.8 * abs(deg):
            print("  ** The arm moved far less than asked - it is not following. **\n"
                  "  Check: pendant in AUTO, no alarm at http://192.168.58.2,"
                  " web speed box not 0. Then RUN_REAL_ROBOT.md Step 2.")
            return 1
        print("  OK")
        return 0

    except (KeyboardInterrupt, ExternalShutdownException):
        if moving and node.q is not None and rclpy.ok():
            node.send(dict(node.q), 0.5)          # stop where it is now
            pump(0.2)
            print("\n  stopped - holding the current position")
        else:
            print("\n  stopped")
        return 1
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
