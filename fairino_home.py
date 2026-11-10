#!/usr/bin/env python3
"""
fairino_home.py  —  move the FR5 to its home position.

Home pose:  j1=0  j2=-90  j3=90  j4=-90  j5=-90  j6=0   (degrees)

THIS SCRIPT MOVES THE ROBOT.

Requires Method A to be running (README Part 2.1):

    ros2 run fairino_hardware ros2_cmd_server

It will NOT work while Method B (RViz/MoveIt) holds the controller
connection — see README Part 0.5.

Usage
-----
    python3 fairino_home.py              # asks for confirmation first
    python3 fairino_home.py --yes        # skip the prompt (scripted use)
    python3 fairino_home.py --speed 3    # slower than the default 5 %
    python3 fairino_home.py --dry-run    # register the target, do not move
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

try:
    from fairino_msgs.srv import RemoteCmdInterface
except ImportError:
    sys.exit("fairino_msgs not found.  Did you source the workspace?\n"
             "  source ~/fairino_380_ws/install/local_setup.bash")

SERVICE = "/fairino_remote_command_service"

# Home joint angles in degrees, in joint order j1..j6
HOME = (0, -90, 90, -90, -90, 0)

# Target slot to register the home pose into
TARGET_ID = 1

# FR5 soft limits, degrees: (lower, upper) per joint.
# Note axes 2 and 4 are asymmetric, and axis 3 is +/-150 for the FR5
# (the shipped URDF wrongly says +/-162 -- see README Part 6.3).
SOFT_LIMITS = (
    (-175, 175),
    (-265, 85),
    (-150, 150),
    (-265, 85),
    (-175, 175),
    (-175, 175),
)

MAX_SPEED = 10  # refuse anything above this; commissioning speed is 5


class HomeMover(Node):

    def __init__(self):
        super().__init__("fairino_home")
        self.cli = self.create_client(RemoteCmdInterface, SERVICE)

    def wait_for_server(self, timeout=10.0):
        self.get_logger().info(f"Waiting for {SERVICE} ...")
        if not self.cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f"{SERVICE} did not appear within {timeout:.0f}s.\n"
                "  Is ros2_cmd_server running?   ros2 run fairino_hardware ros2_cmd_server\n"
                "  Is RViz/MoveIt holding the connection?  See README Part 0.5.")
            return False
        return True

    def send(self, cmd_str):
        """Send one command string, return the response or None."""
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd_str
        self.get_logger().info(f"-> {cmd_str}")

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done():
            self.get_logger().error("No response within 30s.")
            return None

        resp = future.result()
        res = getattr(resp, "cmd_res", None)
        self.get_logger().info(f"<- cmd_res = {res}")
        return res


def check_limits():
    """Refuse to run if the hard-coded home pose is outside the soft limits."""
    for i, (angle, (lo, hi)) in enumerate(zip(HOME, SOFT_LIMITS), start=1):
        if not lo <= angle <= hi:
            sys.exit(f"Home angle for j{i} ({angle} deg) is outside the FR5 "
                     f"soft limit [{lo}, {hi}]. Refusing to run.")


def confirm(speed, dry_run):
    print()
    print("  " + "-" * 62)
    print("   THIS WILL MOVE THE ROBOT")
    print("  " + "-" * 62)
    print(f"   Target : j1={HOME[0]}  j2={HOME[1]}  j3={HOME[2]}  "
          f"j4={HOME[3]}  j5={HOME[4]}  j6={HOME[5]}   (degrees)")
    print(f"   Speed  : {speed} %")
    if dry_run:
        print("   DRY RUN: the target will be registered but no motion sent.")
    print()
    print("   Before continuing, confirm:")
    print("     - a supervisor is present")
    print("     - the emergency stop is within reach of your hand")
    print("     - the swept volume is clear for the whole arm and tool")
    print("     - nobody is inside the work envelope")
    print()
    try:
        answer = input("   Type YES to proceed, anything else to abort: ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return answer == "YES"


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yes", action="store_true",
                    help="skip the confirmation prompt")
    ap.add_argument("--speed", type=int, default=5,
                    help="speed percentage (default 5, refuses above "
                         f"{MAX_SPEED})")
    ap.add_argument("--dry-run", action="store_true",
                    help="register the target but do not move")
    args = ap.parse_args()

    if not 1 <= args.speed <= MAX_SPEED:
        sys.exit(f"--speed must be between 1 and {MAX_SPEED}. "
                 "Commissioning speed is 5.")

    check_limits()

    if not args.yes and not confirm(args.speed, args.dry_run):
        print("   Aborted.\n")
        return 1

    rclpy.init()
    node = HomeMover()
    rc = 1
    try:
        if not node.wait_for_server():
            return 1

        # Step 1 -- register the target. Safe: this only stores numbers.
        jnt = ",".join(str(a) for a in HOME)
        res = node.send(f"JNTPoint({TARGET_ID},{jnt})")
        if res is None:
            return 1
        if str(res) != "0":
            node.get_logger().error(
                f"Registering the target returned cmd_res={res}, expected 0. "
                "Not moving.")
            return 1

        if args.dry_run:
            node.get_logger().info("Dry run: target registered, no motion sent.")
            return 0

        # Step 2 -- move. This is the part that moves the arm.
        res = node.send(f"MoveJ(JNT{TARGET_ID},{args.speed},0,0)")
        if res is None:
            return 1
        if str(res) != "0":
            node.get_logger().error(f"MoveJ returned cmd_res={res}, expected 0.")
            return 1

        node.get_logger().info(
            "Command accepted. NOTE: cmd_res=0 means the API accepted it, "
            "not that the arm reached the pose.")
        node.get_logger().info(
            "Verify with:  ros2 topic echo /nonrt_state_data --once")
        rc = 0

    except KeyboardInterrupt:
        node.get_logger().warn(
            "Interrupted. This does NOT stop the robot. "
            "Use the physical e-stop or ImmStopJOG() -- README Part 2.5.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return rc


if __name__ == "__main__":
    sys.exit(main())
