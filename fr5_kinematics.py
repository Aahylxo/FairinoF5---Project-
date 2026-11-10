#!/usr/bin/env python3
"""
fr5_kinematics.py  —  forward and inverse kinematics for the FAIRINO FR5.

Standard (classic) Denavit-Hartenberg convention:

    T_i = Rz(theta_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)

DH table (README Part 6.1), lengths in mm:

    i |  theta  |   a    |  d   | alpha
    --+---------+--------+------+-------
    1 | theta_1 |    0   | 152  |  +90
    2 | theta_2 |  -425  |   0  |    0
    3 | theta_3 |  -395  |   0  |    0
    4 | theta_4 |    0   | 102  |  +90
    5 | theta_5 |    0   | 102  |  -90
    6 | theta_6 |    0   | 100  |    0

The FR5 has the same kinematic structure as a UR-style arm: a shoulder
offset, two parallel elbow links, and a non-spherical offset wrist. That
structure has a closed-form inverse solution with up to 8 branches, which is
what ik() computes -- no numerical iteration, no initial guess.

NOTHING HERE TOUCHES THE ROBOT. It is pure maths.

Usage
-----
    # forward: six joint angles in degrees -> tool pose
    python3 fr5_kinematics.py fk 0 -90 90 -90 -90 0

    # inverse: tool pose -> every valid joint solution
    python3 fr5_kinematics.py ik --xyz 100 -400 500 --rpy 0 90 0

    # prove FK and IK agree, over many random poses
    python3 fr5_kinematics.py selftest

    # compare against the live robot (needs ROS 2 + /joint_states)
    python3 fr5_kinematics.py live
"""

import argparse
import math
import sys

import numpy as np

# --------------------------------------------------------------- DH table
#            a (mm)    d (mm)    alpha (rad)
DH = [
    (   0.0,   152.0,  math.pi / 2),   # joint 1
    (-425.0,     0.0,  0.0),           # joint 2
    (-395.0,     0.0,  0.0),           # joint 3
    (   0.0,   102.0,  math.pi / 2),   # joint 4
    (   0.0,   102.0, -math.pi / 2),   # joint 5
    (   0.0,   100.0,  0.0),           # joint 6
]

A = [row[0] for row in DH]
D = [row[1] for row in DH]
ALPHA = [row[2] for row in DH]

# FR5 soft limits in degrees (README Part 6.3).
# These are the robot's own limits, NOT the shipped URDF values -- the URDF
# wrongly gives joint 3 as +/-162, which is the FR16/20/30 figure.
SOFT_LIMITS_DEG = [
    (-175.0, 175.0),
    (-265.0,  85.0),
    (-150.0, 150.0),
    (-265.0,  85.0),
    (-175.0, 175.0),
    (-175.0, 175.0),
]


# ------------------------------------------------------------------ basics
def dh_matrix(theta, a, d, alpha):
    """One standard-DH link transform."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def fk(q, upto=6):
    """
    Forward kinematics.

    q    : six joint angles in RADIANS
    upto : stop after this many joints (6 = full chain, base to tool flange)

    Returns a 4x4 homogeneous transform, translation in mm.
    """
    T = np.eye(4)
    for i in range(upto):
        T = T @ dh_matrix(q[i], A[i], D[i], ALPHA[i])
    return T


def fk_all_frames(q):
    """Return [T01, T02, ... T06] -- useful for drawing or debugging."""
    frames = []
    T = np.eye(4)
    for i in range(6):
        T = T @ dh_matrix(q[i], A[i], D[i], ALPHA[i])
        frames.append(T.copy())
    return frames


# ----------------------------------------------------- pose <-> transform
def rpy_to_matrix(roll, pitch, yaw):
    """XYZ fixed-axis roll-pitch-yaw (radians) -> 3x3 rotation."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [    -sp,                cp * sr,                cp * cr],
    ])


def matrix_to_rpy(R):
    """3x3 rotation -> (roll, pitch, yaw) in radians, XYZ fixed-axis."""
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    if abs(sy) > 1.0 - 1e-9:                    # gimbal lock
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def make_pose(x, y, z, roll, pitch, yaw):
    """Build a 4x4 from mm + radians."""
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(roll, pitch, yaw)
    T[:3, 3] = (x, y, z)
    return T


# ------------------------------------------------------------------- IK
def _wrap(angle):
    """Wrap to (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def ik(T06, tol=1e-8):
    """
    Closed-form inverse kinematics.

    T06 : desired 4x4 pose of the tool flange, translation in mm

    Returns a list of joint solutions, each a list of six angles in radians,
    wrapped to (-pi, pi]. Up to 8 branches: shoulder left/right,
    elbow up/down, wrist flip. Unreachable poses give fewer, or none.

    Joint limits are NOT applied here -- use filter_limits() for that, so you
    can see the raw mathematics separately from the robot's restrictions.
    """
    T06 = np.asarray(T06, dtype=float)
    d1, d4, d5, d6 = D[0], D[3], D[4], D[5]
    a2, a3 = A[1], A[2]

    solutions = []

    # ---- theta 1 -------------------------------------------------------
    # Position of frame 5's origin, found by backing off d6 along the tool z.
    p05 = T06 @ np.array([0.0, 0.0, -d6, 1.0])
    psi = math.atan2(p05[1], p05[0])
    r = math.hypot(p05[0], p05[1])
    if r < abs(d4) - tol:
        return []                       # wrist centre inside the shoulder offset
    ratio = max(-1.0, min(1.0, d4 / r))
    phi = math.acos(ratio)

    for t1 in (_wrap(psi + phi + math.pi / 2),
               _wrap(psi - phi + math.pi / 2)):

        s1, c1 = math.sin(t1), math.cos(t1)

        # ---- theta 5 ---------------------------------------------------
        num = T06[0, 3] * s1 - T06[1, 3] * c1 - d4
        ratio5 = max(-1.0, min(1.0, num / d6))
        for t5 in (math.acos(ratio5), -math.acos(ratio5)):

            s5 = math.sin(t5)

            # ---- theta 6 -----------------------------------------------
            if abs(s5) < 1e-7:
                # Wrist singularity: theta4 and theta6 are coupled, infinitely
                # many pairs give the same pose. Pin theta6 = 0 and let
                # theta4 absorb the rotation.
                t6 = 0.0
            else:
                yx, yy = T06[0, 1], T06[1, 1]
                xx, xy = T06[0, 0], T06[1, 0]
                t6 = math.atan2((-yx * s1 + yy * c1) / s5,
                                ( xx * s1 - xy * c1) / s5)

            # ---- theta 3, 2, 4 -------------------------------------------
            T01 = dh_matrix(t1, A[0], D[0], ALPHA[0])
            T45 = dh_matrix(t5, A[4], D[4], ALPHA[4])
            T56 = dh_matrix(t6, A[5], D[5], ALPHA[5])
            T14 = np.linalg.inv(T01) @ T06 @ np.linalg.inv(T56) @ np.linalg.inv(T45)

            p13 = T14 @ np.array([0.0, -d4, 0.0, 1.0])
            px, py = p13[0], p13[1]
            L2 = px * px + py * py

            cos3 = (L2 - a2 * a2 - a3 * a3) / (2 * a2 * a3)
            if abs(cos3) > 1.0 + 1e-6:
                continue                       # out of reach on this branch
            cos3 = max(-1.0, min(1.0, cos3))

            for t3 in (math.acos(cos3), -math.acos(cos3)):

                denom = math.sqrt(L2)
                if denom < tol:
                    continue
                t2 = math.atan2(-py, -px) + math.asin(
                    max(-1.0, min(1.0, a3 * math.sin(t3) / denom)))

                T12 = dh_matrix(t2, A[1], D[1], ALPHA[1])
                T23 = dh_matrix(t3, A[2], D[2], ALPHA[2])
                T34 = np.linalg.inv(T12 @ T23) @ T14
                t4 = math.atan2(T34[1, 0], T34[0, 0])

                sol = [_wrap(t1), _wrap(t2), _wrap(t3),
                       _wrap(t4), _wrap(t5), _wrap(t6)]

                # Only keep it if it actually reproduces the requested pose.
                # This filters the spurious branches the algebra can produce.
                if pose_error(fk(sol), T06)[0] < 1e-4:
                    if not any(np.allclose(sol, s, atol=1e-6) for s in solutions):
                        solutions.append(sol)

    return solutions


def pose_error(Ta, Tb):
    """(translation error in mm, rotation error in degrees) between two poses."""
    dp = float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]))
    R = Ta[:3, :3].T @ Tb[:3, :3]
    cos_angle = (np.trace(R) - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return dp, math.degrees(math.acos(cos_angle))


# --------------------------------------------------------------- limits
def within_limits(q_rad, limits_deg=None):
    """True if every joint of this solution is inside the FR5 soft limits."""
    limits = limits_deg or SOFT_LIMITS_DEG
    for angle, (lo, hi) in zip(q_rad, limits):
        if not lo <= math.degrees(angle) <= hi:
            return False
    return True


def filter_limits(solutions, limits_deg=None):
    """Keep only the solutions the FR5 can actually reach."""
    return [s for s in solutions if within_limits(s, limits_deg)]


def closest_solution(solutions, q_ref):
    """
    Pick the solution requiring the least total joint movement from q_ref.

    This is what you want when commanding a real arm: of several valid ways
    to reach the same pose, take the one that moves least, rather than the
    one that flips the elbow through the table.
    """
    if not solutions:
        return None
    return min(solutions,
               key=lambda s: sum(abs(_wrap(a - b)) for a, b in zip(s, q_ref)))


# ------------------------------------------------------------- printing
def print_pose(T, label="pose"):
    x, y, z = T[:3, 3]
    roll, pitch, yaw = matrix_to_rpy(T[:3, :3])
    print(f"\n  {label}")
    print(f"    position     x = {x:10.3f} mm")
    print(f"                 y = {y:10.3f} mm")
    print(f"                 z = {z:10.3f} mm")
    print(f"    orientation  roll  = {math.degrees(roll):8.3f} deg")
    print(f"                 pitch = {math.degrees(pitch):8.3f} deg")
    print(f"                 yaw   = {math.degrees(yaw):8.3f} deg")
    print("\n    transform")
    for row in T:
        print("      [" + "  ".join(f"{v:9.4f}" for v in row) + "]")


def print_solutions(solutions, q_ref=None):
    if not solutions:
        print("\n  No solutions. The pose is out of reach, or outside the "
              "joint limits.")
        return

    print(f"\n  {len(solutions)} solution(s), degrees:\n")
    header = "      #   " + "".join(f"{'j'+str(i+1):>10s}" for i in range(6))
    print(header + "    within limits")
    print("      " + "-" * (len(header) + 10))

    best = closest_solution(filter_limits(solutions), q_ref) if q_ref else None

    for i, s in enumerate(solutions):
        degs = "".join(f"{math.degrees(a):10.2f}" for a in s)
        ok = "yes" if within_limits(s) else "NO"
        mark = "  <- closest" if best is not None and np.allclose(s, best) else ""
        print(f"      {i:<3d} {degs}       {ok}{mark}")


# ------------------------------------------------------------ selftest
def selftest(n=2000, seed=0, verbose=False):
    """
    Generate random reachable joint sets, run FK, feed the pose back into IK,
    and check that at least one returned solution reproduces the original pose.

    This is the test that matters: it verifies FK and IK against each other
    without needing the robot, and it would catch a sign error in either.
    """
    rng = np.random.default_rng(seed)
    worst_pos = 0.0
    worst_rot = 0.0
    failures = 0
    recovered_exactly = 0
    counts = {}

    print(f"\n  Round-trip test: {n} random poses")
    print("  joint angles -> FK -> pose -> IK -> solutions -> FK -> pose\n")

    for _ in range(n):
        q = [math.radians(rng.uniform(lo, hi)) for lo, hi in SOFT_LIMITS_DEG]
        T = fk(q)

        sols = ik(T)
        counts[len(sols)] = counts.get(len(sols), 0) + 1

        if not sols:
            failures += 1
            if verbose:
                print("    no solution for q =",
                      [round(math.degrees(a), 2) for a in q])
            continue

        # best reproduction of the requested pose among the branches
        errs = [pose_error(fk(s), T) for s in sols]
        dp, dr = min(errs, key=lambda e: e[0] + e[1])
        worst_pos = max(worst_pos, dp)
        worst_rot = max(worst_rot, dr)
        if dp > 1e-3 or dr > 1e-3:
            failures += 1

        # did any branch recover the exact joint angles we started from?
        if any(all(abs(_wrap(a - b)) < 1e-6 for a, b in zip(s, q)) for s in sols):
            recovered_exactly += 1

    print(f"    worst position error      {worst_pos:.3e} mm")
    print(f"    worst orientation error   {worst_rot:.3e} deg")
    print(f"    original pose recovered   {recovered_exactly}/{n}")
    print(f"    failures                  {failures}/{n}")
    print("\n    branches found per pose:")
    for k in sorted(counts):
        print(f"      {k} solution(s): {counts[k]}")

    ok = failures == 0
    print("\n  RESULT: " + ("PASS — FK and IK agree.\n" if ok else
                            "FAIL — see above.\n"))
    return ok


# ---------------------------------------------------------------- live
def live_compare(timeout=10.0):
    """
    Read the real joint angles from /joint_states, run our FK on them, and
    print the resulting tool pose.

    Useful as the final check: compare this against what MoveIt reports for
    the end-effector frame. Agreement means the DH table and this
    implementation are both right.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError:
        sys.exit("ROS 2 Python packages not found. Source the workspace:\n"
                 "  source /opt/ros/humble/setup.bash")

    names_wanted = ["j1", "j2", "j3", "j4", "j5", "j6"]
    got = {}

    class Listener(Node):
        def __init__(self):
            super().__init__("fr5_kinematics_live")
            self.create_subscription(JointState, "/joint_states", self.cb, 10)

        def cb(self, msg):
            # IMPORTANT: /joint_states does NOT publish j1..j6 in order.
            # The observed order is j1, j2, j4, j5, j3, j6. Always match by
            # name, never by index.
            for name, pos in zip(msg.name, msg.position):
                if name in names_wanted:
                    got[name] = pos

    rclpy.init()
    node = Listener()
    print("\n  Waiting for /joint_states ...")
    start = node.get_clock().now()
    try:
        while rclpy.ok() and len(got) < 6:
            rclpy.spin_once(node, timeout_sec=0.2)
            elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(got) < 6:
        print(f"    Only got {sorted(got)} within {timeout:.0f}s.")
        print("    Is the robot launched?  See README Part 3.4.")
        return 1

    q = [got[n] for n in names_wanted]
    print("\n    joint angles read from the robot:")
    for i, a in enumerate(q, start=1):
        print(f"      j{i} = {a:10.6f} rad = {math.degrees(a):9.3f} deg")

    print_pose(fk(q), "tool flange pose computed by our FK")

    print("\n  Compare this against MoveIt's end-effector pose in RViz.")
    print("  A mismatch means either the DH table or a joint-angle offset "
          "is wrong,")
    print("  not necessarily that the maths is broken — see the note in "
          "--help.\n")
    return 0


# ----------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_fk = sub.add_parser("fk", help="joint angles -> tool pose")
    p_fk.add_argument("angles", nargs=6, type=float,
                      metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                      help="six joint angles in degrees")
    p_fk.add_argument("--frames", action="store_true",
                      help="also print every intermediate frame origin")

    p_ik = sub.add_parser("ik", help="tool pose -> joint angles")
    p_ik.add_argument("--xyz", nargs=3, type=float, required=True,
                      metavar=("X", "Y", "Z"), help="position in mm")
    p_ik.add_argument("--rpy", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                      metavar=("R", "P", "Y"),
                      help="orientation in degrees (default 0 0 0)")
    p_ik.add_argument("--near", nargs=6, type=float, default=None,
                      metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                      help="current joint angles in degrees; marks the "
                           "solution needing the least movement")
    p_ik.add_argument("--all", action="store_true",
                      help="show solutions outside the joint limits too")

    p_st = sub.add_parser("selftest", help="verify FK and IK against each other")
    p_st.add_argument("-n", type=int, default=2000, help="number of poses")
    p_st.add_argument("--seed", type=int, default=0)
    p_st.add_argument("-v", "--verbose", action="store_true")

    sub.add_parser("live", help="read /joint_states and run FK on it")

    args = ap.parse_args()

    if args.cmd == "fk":
        q = [math.radians(a) for a in args.angles]
        if not within_limits(q):
            print("\n  WARNING: those angles are outside the FR5 soft limits.")
        T = fk(q)
        print_pose(T, "tool flange pose")
        if args.frames:
            print("\n  frame origins (mm)")
            for i, Ti in enumerate(fk_all_frames(q), start=1):
                x, y, z = Ti[:3, 3]
                print(f"    frame {i}:  {x:10.3f} {y:10.3f} {z:10.3f}")
        print()
        return 0

    if args.cmd == "ik":
        x, y, z = args.xyz
        roll, pitch, yaw = (math.radians(a) for a in args.rpy)
        T = make_pose(x, y, z, roll, pitch, yaw)
        print_pose(T, "requested tool pose")

        sols = ik(T)
        if not args.all:
            sols = filter_limits(sols)

        q_ref = [math.radians(a) for a in args.near] if args.near else None
        print_solutions(sols, q_ref)
        print()
        return 0 if sols else 1

    if args.cmd == "selftest":
        return 0 if selftest(args.n, args.seed, args.verbose) else 1

    if args.cmd == "live":
        return live_compare()

    return 1


if __name__ == "__main__":
    sys.exit(main())
