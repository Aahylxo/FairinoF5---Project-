#!/usr/bin/env python3
"""
hand_teleop.py  —  Cartesian hand teleoperation of the FR5.

Your hand's position in space drives the robot's TOOL position in space,
through the inverse kinematics in fr5_kinematics.py.

    camera -> MediaPipe -> your wrist position (metres, torso frame)
           -> offset from the pose you captured
           -> scaled, axis-mapped into the robot's base frame
           -> CLAMPED to a safe Cartesian workspace box
           -> inverse kinematics -> six joint angles
           -> smoothing, rate limiting, joint clamping
           -> /fairino5_controller/joint_trajectory

This replaces arm_tracker.py + joint_mapper.py + teleop_node.py with a single
process and a single window.

WHY THIS IS BETTER THAN MAPPING JOINT TO JOINT
----------------------------------------------
Your arm's joints and the FR5's joints are not the same shape, so any
joint-to-joint mapping is arbitrary and needs hand-tuned offsets. Worse, it has
no idea where the tool actually ends up -- which is how you drive the arm into
the floor while every joint is still inside its limits.

Working in Cartesian space fixes both. "The tool may not go below z = 200 mm"
is one line here and effectively impossible in joint space. And your 3-DOF hand
position plus a fixed tool orientation fully determines all six joint angles,
so the 3-to-6 problem goes away.

ORIENTATION
-----------
The tool keeps whatever orientation the robot had when you pressed
"Capture neutral". Only position follows your hand. That guarantees the
starting pose is reachable and avoids needing to specify an orientation by
hand.

Usage
-----
    python3 hand_teleop.py --dry-run     # compute everything, publish nothing
    python3 hand_teleop.py               # drive the robot
    python3 hand_teleop.py --hold        # hold-to-move instead of click-toggle

Needs tkinter:  sudo apt install -y python3-tk
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import cv2

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    sys.exit("tkinter is not installed.  Run:\n"
             "  sudo apt install -y python3-tk")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Our own modules, which live beside this file.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import fr5_kinematics as K
except ImportError:
    sys.exit("fr5_kinematics.py not found. It must be in the same folder as "
             "this script.")
try:
    from arm_tracker import (torso_frame, mp_pose, mp_draw,
                             R_SHOULDER, R_ELBOW, R_WRIST,
                             L_SHOULDER, R_HIP, L_HIP)
except ImportError as exc:
    sys.exit(f"could not import from arm_tracker.py: {exc}\n"
             "It must be in the same folder as this script.")

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
CONTROLLER_TOPIC = "/fairino5_controller/joint_trajectory"
IMAGE_TOPIC = "/camera/camera/color/image_raw"

# --- Cartesian workspace box, millimetres in the robot's base frame --------
# This is what stops the arm reaching into the floor or folding into itself.
# Tighten it for your bench; these are deliberately conservative.
WORKSPACE = {
    "z_min": 300.0,      # never below this height -- this is the floor guard
    "z_max": 900.0,
    "r_min": 250.0,      # stay out of the column above the base
    "r_max": 700.0,      # inside the 922 mm reach, with margin
}

# How far the tool may stray from the captured neutral position, per axis, in
# millimetres. This keeps motion local and predictable, and stops a big hand
# movement dragging the target onto the edge of the workspace where the IK
# starts flipping between arm configurations.
#
# It matters more than it looks: the home pose already sits 696 mm from the
# base, so without this the tool spends its whole time pinned against r_max.
MAX_OFFSET_MM = 300.0

# --- safety defaults ------------------------------------------------------
MAX_SPEED_DEG_S = 10.0
FILTER_HZ = 2.0
LOOP_HZ = 25.0
STATE_TIMEOUT = 1.0
IMAGE_TIMEOUT = 1.0

# How far the hand's motion is amplified. A human's comfortable reach is maybe
# 0.6 m; the FR5's is 0.92 m.
DEFAULT_SCALE = 1.0

# Refuse an IK solution that needs more than this on any one joint: that is
# a different arm configuration, not a small correction.
MAX_JUMP_DEG = 30.0

# Max base rotation away from the captured neutral, degrees.
MAX_AZIMUTH_DEG = 90.0

# The FR5 home pose, degrees. Tool ends up around (-497, -102, 477), which is
# 696 mm from the base -- comfortably central, with room to move in every
# direction. Capturing a neutral pose from anywhere near a workspace limit is
# what makes teleoperation feel like it is constantly jammed.
HOME_DEG = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# --------------------------------------------------------------------- ROS
class TeleopNode(Node):

    def __init__(self):
        super().__init__("fr5_hand_teleop")
        self.lock = threading.Lock()
        self.frame = None
        self.frame_stamp = 0.0
        self.actual = None               # joint angles, radians, j1..j6
        self.actual_stamp = 0.0

        try:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
        except ImportError:
            sys.exit("cv_bridge is not installed.  Run:\n"
                     "  sudo apt install -y ros-humble-cv-bridge")

        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, 1)
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.pub = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 10)

    def on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:                                     # noqa: BLE001
            return
        with self.lock:
            self.frame = img
            self.frame_stamp = time.time()

    def on_state(self, msg):
        # /joint_states is NOT published in j1..j6 order -- match by name.
        found = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                found[name] = pos
        if len(found) == 6:
            with self.lock:
                self.actual = [found[n] for n in JOINTS]
                self.actual_stamp = time.time()

    def grab(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            return (frame, self.frame_stamp,
                    None if self.actual is None else list(self.actual),
                    self.actual_stamp)

    def send(self, q_rad, dt):
        msg = JointTrajectory()
        msg.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(a) for a in q_rad]
        horizon = max(0.05, dt * 2.0)
        pt.time_from_start = Duration(sec=int(horizon),
                                      nanosec=int((horizon % 1.0) * 1e9))
        msg.points = [pt]
        self.pub.publish(msg)


# -------------------------------------------------------------------- maths
def hand_in_torso_frame(world_landmarks):
    """
    Wrist position in metres, expressed in a frame attached to the operator's
    torso: x out to their right, y up the spine, z forward from the chest.

    Returns None if the torso landmarks are too degenerate to build a frame.
    """
    lm = np.array([[p.x, p.y, p.z] for p in world_landmarks.landmark])
    frame = torso_frame(lm)
    if frame is None:
        return None
    origin, R = frame
    return R.T @ (lm[R_WRIST] - origin)


def to_cylindrical(p):
    """
    (x, y, z) -> (radius in the horizontal plane, angle about the vertical,
    height). Angle in radians, measured the same way as the robot's j1.
    """
    x, y, z = p
    return math.hypot(x, y), math.atan2(y, x), z


def from_cylindrical(r, theta, z):
    return np.array([r * math.cos(theta), r * math.sin(theta), z])


def hand_cylindrical(p):
    """
    The operator's hand in cylindrical coordinates about their own vertical
    axis, from a torso-frame position (x right, y up, z forward).

    Returns (horizontal reach, azimuth, height). Azimuth is 0 straight out to
    the right and increases towards the front.
    """
    rho = math.hypot(p[0], p[2])
    phi = math.atan2(p[2], p[0])
    return rho, phi, p[1]


def clamp_offset(delta, limit):
    """Clamp the per-axis offset from the neutral position."""
    hits = []
    out = []
    for value, axis in zip(delta, "XYZ"):
        if abs(value) > limit:
            value = math.copysign(limit, value)
            hits.append(f"offset {axis}")
        out.append(value)
    return np.array(out), hits


def clamp_workspace(p):
    """
    Clamp a target position (mm, robot base frame) into the safe box.

    Returns (clamped_point, list_of_limits_hit).
    """
    hits = []
    x, y, z = p

    if z < WORKSPACE["z_min"]:
        z = WORKSPACE["z_min"]
        hits.append("floor")
    elif z > WORKSPACE["z_max"]:
        z = WORKSPACE["z_max"]
        hits.append("ceiling")

    r = math.sqrt(x * x + y * y + z * z)
    if r > WORKSPACE["r_max"]:
        s = WORKSPACE["r_max"] / r
        x, y, z = x * s, y * s, z * s
        hits.append("reach")
    elif r < WORKSPACE["r_min"] and r > 1e-6:
        s = WORKSPACE["r_min"] / r
        x, y, z = x * s, y * s, z * s
        hits.append("too close")

    # Re-check height after the radial clamp
    if z < WORKSPACE["z_min"]:
        z = WORKSPACE["z_min"]
        if "floor" not in hits:
            hits.append("floor")

    return np.array([x, y, z]), hits


# ---------------------------------------------------------------------- app
class App:

    AXES = ["right", "up", "forward"]

    def __init__(self, root, node, args):
        self.root = root
        self.node = node
        self.args = args

        self.pose = mp_pose.Pose(model_complexity=1,
                                 min_detection_confidence=0.6,
                                 min_tracking_confidence=0.6,
                                 smooth_landmarks=True)

        self.enabled = False
        self.neutral_hand = None       # hand position at capture, metres
        self.neutral_pos = None        # tool position at capture, mm
        self.neutral_rot = None        # tool orientation at capture, fixed
        self.neutral_cyl = None        # tool (radius, angle, height) at capture
        self.neutral_hand_cyl = None   # hand (reach, azimuth, height) at capture
        self.last_phi = None           # last good hand azimuth
        self.command = None            # the joint command we are sending
        self.filtered = None
        self.last_desired = None       # previous IK result, for flip detection
        self.homing = False            # driving back to the home pose
        self.last = time.time()
        self.reason = "idle"
        self.hits = []
        self.target_pos = None
        self.cyl_readout = None
        self.tracking = False

        root.title("FR5 hand teleoperation  -  Cartesian")
        root.protocol("WM_DELETE_WINDOW", self.quit)

        mode = ("DRY RUN  -  nothing is published" if args.dry_run
                else f"LIVE  -  publishing to {CONTROLLER_TOPIC}")
        tk.Label(root, text=f"  {mode}  ",
                 bg="#1f5fa9" if args.dry_run else "#a93226", fg="white",
                 font=("TkDefaultFont", 11, "bold"), pady=7).pack(fill="x")

        # ---- status, large and impossible to miss
        self.status = tk.Label(root, text="starting...",
                               font=("TkDefaultFont", 15, "bold"),
                               fg="#777", pady=8)
        self.status.pack(fill="x")
        self.detail = tk.Label(root, text="", font=("TkDefaultFont", 11),
                               fg="#a93226", pady=2)
        self.detail.pack(fill="x")

        # ---- mode selector
        mframe = ttk.LabelFrame(root, text=" mapping mode ")
        mframe.pack(fill="x", padx=10, pady=(6, 2))
        self.mode = tk.StringVar(value=args.mode)
        for label, value, hint in (
                ("Cylindrical  (arm swing -> base rotation)", "cylindrical",
                 "your arm's sweep drives j1 one-for-one"),
                ("Cartesian  (hand XYZ -> tool XYZ)", "cartesian",
                 "straight-line offsets; base rotation is limited")):
            row = ttk.Frame(mframe)
            row.pack(fill="x", padx=10, pady=2)
            ttk.Radiobutton(row, text=label, value=value,
                            variable=self.mode,
                            command=self.on_mode).pack(side="left")
            tk.Label(row, text="  " + hint, fg="#777",
                     font=("TkDefaultFont", 8)).pack(side="left")

        # ---- cylindrical controls
        self.cyl_frame = ttk.LabelFrame(root, text=" cylindrical ")
        self.cyl_frame.pack(fill="x", padx=10, pady=4)

        cbox = ttk.Frame(self.cyl_frame)
        cbox.pack(padx=10, pady=6)
        tk.Label(cbox, text="base rotation gain").grid(row=0, column=0, padx=6)
        self.azim_gain = tk.DoubleVar(value=1.0)
        ttk.Scale(cbox, from_=0.3, to=2.5, variable=self.azim_gain,
                  orient="horizontal", length=130).grid(row=0, column=1)
        self.azim_lab = tk.Label(cbox, text="1.00 x", width=7)
        self.azim_lab.grid(row=0, column=2, padx=(4, 14))

        self.azim_inv = tk.BooleanVar(value=False)
        ttk.Checkbutton(cbox, text="invert rotation",
                        variable=self.azim_inv).grid(row=0, column=3, padx=6)
        self.radial_inv = tk.BooleanVar(value=False)
        ttk.Checkbutton(cbox, text="invert reach",
                        variable=self.radial_inv).grid(row=0, column=4, padx=6)

        self.cyl_lab = tk.Label(self.cyl_frame, text="", fg="#555",
                                font=("TkFixedFont", 10))
        self.cyl_lab.pack(pady=(0, 6))

        # ---- axis mapping: three rows, not six
        aframe = ttk.LabelFrame(root, text=" hand axis  ->  robot axis "
                                           " (Cartesian mode only) ")
        self.cart_frame = aframe
        aframe.pack(fill="x", padx=10, pady=6)

        self.axis_src = {}
        self.axis_inv = {}
        # The arm reaches out along NEGATIVE x from the base: the home tool
        # position is x = -497. So "hand moves toward the camera" must map to
        # -x for the arm to extend rather than fold back toward its base.
        defaults = {"X": "forward", "Y": "right", "Z": "up"}
        inverted = {"X": True, "Y": True, "Z": False}

        for col, axis in enumerate(("X", "Y", "Z")):
            box = ttk.Frame(aframe)
            box.grid(row=0, column=col, padx=12, pady=8)
            tk.Label(box, text=f"robot {axis}",
                     font=("TkDefaultFont", 10, "bold")).pack()
            var = tk.StringVar(value=defaults[axis])
            ttk.OptionMenu(box, var, defaults[axis], *self.AXES).pack()
            self.axis_src[axis] = var
            inv = tk.BooleanVar(value=inverted[axis])
            ttk.Checkbutton(box, text="invert", variable=inv).pack()
            self.axis_inv[axis] = inv

        sbox = ttk.Frame(aframe)
        sbox.grid(row=0, column=3, padx=16)
        tk.Label(sbox, text="scale").pack()
        self.scale = tk.DoubleVar(value=args.scale)
        ttk.Scale(sbox, from_=0.5, to=3.0, variable=self.scale,
                  orient="horizontal", length=140).pack()
        self.scale_lab = tk.Label(sbox, text=f"{args.scale:.2f} x")
        self.scale_lab.pack()

        # ---- tool position readout
        pframe = ttk.LabelFrame(root, text=" tool position (mm, robot base frame) ")
        pframe.pack(fill="x", padx=10, pady=4)
        self.pos_frame = pframe
        self.pos_labels = {}
        for i, axis in enumerate(("X", "Y", "Z")):
            tk.Label(pframe, text=axis, width=3,
                     font=("TkDefaultFont", 10, "bold")).grid(row=0, column=i * 2,
                                                              padx=(12, 2), pady=6)
            lab = tk.Label(pframe, text="---", width=10, anchor="e",
                           font=("TkFixedFont", 12))
            lab.grid(row=0, column=i * 2 + 1, padx=(0, 10))
            self.pos_labels[axis] = lab
        self.limit_lab = tk.Label(pframe, text="", fg="#b26a00",
                                  font=("TkDefaultFont", 10, "bold"))
        self.limit_lab.grid(row=0, column=6, padx=10)

        # ---- joints
        jframe = ttk.LabelFrame(root, text=" joints (degrees) ")
        jframe.pack(fill="both", expand=True, padx=10, pady=4)
        for c, head in enumerate(["", "commanded", "actual", "error"]):
            tk.Label(jframe, text=head,
                     font=("TkDefaultFont", 9, "bold")).grid(
                         row=0, column=c, padx=10, pady=(6, 2))
        self.cells = []
        for i, name in enumerate(JOINTS):
            tk.Label(jframe, text=name, width=4,
                     font=("TkDefaultFont", 11, "bold")).grid(
                         row=i + 1, column=0, padx=10, pady=1)
            row = []
            for c in range(1, 4):
                lab = tk.Label(jframe, text="---", width=10, anchor="e",
                               font=("TkFixedFont", 11))
                lab.grid(row=i + 1, column=c, padx=10)
                row.append(lab)
            self.cells.append(row)

        # ---- controls
        cframe = ttk.Frame(root)
        cframe.pack(fill="x", padx=10, pady=8)

        self.home_btn = tk.Button(
            cframe, text="0.  GO TO HOME POSE",
            font=("TkDefaultFont", 12, "bold"),
            bg="#4a6fa5", fg="white", activebackground="#35527a",
            activeforeground="white", command=self.go_home, height=2)
        self.home_btn.pack(fill="x")

        self.cap_btn = tk.Button(cframe, text="1.  CAPTURE NEUTRAL POSE",
                                 font=("TkDefaultFont", 12, "bold"),
                                 command=self.capture, height=2)
        self.cap_btn.pack(fill="x", pady=(6, 0))

        self.idle_text = ("2.  HOLD TO MOVE" if args.hold
                          else "2.  CLICK TO START MOVING")
        self.live_text = ("MOVING  -  release to stop" if args.hold
                          else "MOVING  -  click to stop")

        self.go = tk.Button(cframe, text=self.idle_text,
                            font=("TkDefaultFont", 14, "bold"),
                            bg="#2d7a3e", fg="white",
                            activebackground="#1f5a2c", activeforeground="white",
                            bd=4, height=2, state="disabled")
        self.go.pack(fill="x", pady=(6, 0))

        if args.hold:
            self.go.bind("<ButtonPress-1>", self.engage)
            self.go.bind("<ButtonRelease-1>", self.disengage)
            root.bind("<FocusOut>", self.disengage)
        else:
            self.go.config(command=self.toggle)

        tk.Button(cframe, text="STOP   (Esc)",
                  font=("TkDefaultFont", 12, "bold"),
                  bg="#a93226", fg="white", activebackground="#7b241c",
                  activeforeground="white", bd=3,
                  command=self.disengage).pack(fill="x", pady=(6, 0))

        root.bind("<Escape>", self.disengage)
        root.bind("<space>", self.disengage)

        tk.Label(root,
                 text=f"max {args.max_speed:.0f} deg/s per joint   |   "
                      f"workspace z>{WORKSPACE['z_min']:.0f}mm, "
                      f"r<{WORKSPACE['r_max']:.0f}mm   |   "
                      f"the physical e-stop is still the real stop",
                 fg="#666", font=("TkDefaultFont", 9)).pack(pady=(0, 8))

        self.on_mode()
        self.tick()

    # --- controls ---------------------------------------------------------
    def capture(self):
        frame, f_stamp, actual, a_stamp = self.node.grab()
        now = time.time()

        if actual is None or now - a_stamp > STATE_TIMEOUT:
            self.reason = "no /joint_states - is the robot launched?"
            return
        if self.hand is None:
            self.reason = "cannot see your arm - step back into frame"
            return

        T = K.fk(actual)
        self.neutral_hand = np.array(self.hand, dtype=float)
        self.neutral_pos = T[:3, 3].copy()
        self.neutral_rot = T[:3, :3].copy()
        self.neutral_cyl = to_cylindrical(self.neutral_pos)
        self.neutral_hand_cyl = hand_cylindrical(self.neutral_hand)
        self.last_phi = self.neutral_hand_cyl[1]
        r = float(np.linalg.norm(self.neutral_pos))
        z = float(self.neutral_pos[2])

        # A neutral pose near a workspace limit leaves nowhere to move and
        # makes every direction hit something straight away.
        headroom = min(WORKSPACE["r_max"] - r, z - WORKSPACE["z_min"])
        if headroom < 120.0:
            self.reason = (f"captured, BUT only {headroom:.0f} mm from a limit "
                           f"(r={r:.0f}, z={z:.0f}) - press GO TO HOME first")
        else:
            self.reason = (f"neutral captured - tool at "
                           f"{self.neutral_pos[0]:.0f}, "
                           f"{self.neutral_pos[1]:.0f}, "
                           f"{self.neutral_pos[2]:.0f} mm   (r={r:.0f})")
        self.go.config(state="normal")

    def on_mode(self):
        """Show only the controls that apply to the selected mode."""
        self.cyl_frame.pack_forget()
        self.cart_frame.pack_forget()
        if self.mode.get() == "cylindrical":
            self.cyl_frame.pack(fill="x", padx=10, pady=4,
                                before=self.pos_frame)
        else:
            self.cart_frame.pack(fill="x", padx=10, pady=6,
                                 before=self.pos_frame)
        self.reason = f"{self.mode.get()} mode - capture a neutral pose"

    def go_home(self):
        """
        Drive the arm back to the home pose, through the same rate limiter.

        Teleoperation should always start from here. The home pose sits in the
        middle of the workspace; capturing a neutral pose from a stretched-out
        position leaves no room to move and makes everything hit a limit.
        """
        _f, _fs, actual, a_stamp = self.node.grab()
        if actual is None or time.time() - a_stamp > STATE_TIMEOUT:
            self.reason = "no /joint_states - is the robot launched?"
            return
        if self.enabled:
            self.reason = "stop teleoperation before homing"
            return

        self.command = [math.degrees(a) for a in actual]
        self.filtered = list(self.command)
        self.homing = True
        self.reason = "moving to home pose..."
        self.home_btn.config(text="0.  HOMING...  (Esc to stop)")

    def toggle(self):
        if self.enabled:
            self.disengage()
        else:
            self.engage()

    def engage(self, _event=None):
        _frame, _fs, actual, a_stamp = self.node.grab()
        now = time.time()

        if self.neutral_pos is None:
            self.reason = "capture a neutral pose first"
            return
        if actual is None or now - a_stamp > STATE_TIMEOUT:
            self.reason = "no /joint_states - is the robot launched?"
            return
        if not self.tracking:
            self.reason = "cannot see your arm - step back into frame"
            return

        # Start from where the arm actually is, in degrees.
        self.command = [math.degrees(a) for a in actual]
        self.filtered = list(self.command)
        self.last_desired = list(self.command)
        self.enabled = True
        self.reason = "engaged"
        self.go.config(bg="#a93226", text=self.live_text)

    def disengage(self, _event=None):
        if self.enabled or self.homing:
            self.reason = "stopped"
        self.enabled = False
        self.homing = False
        self.home_btn.config(text="0.  GO TO HOME POSE")
        self.go.config(bg="#2d7a3e", text=self.idle_text)

    def quit(self):
        self.disengage()
        try:
            self.pose.close()
        except Exception:                                     # noqa: BLE001
            pass
        cv2.destroyAllWindows()
        self.root.destroy()

    # --- main loop --------------------------------------------------------
    hand = None

    def tick(self):
        now = time.time()
        dt = max(1e-3, now - self.last)
        self.last = now

        frame, f_stamp, actual, a_stamp = self.node.grab()
        image_fresh = frame is not None and (now - f_stamp) < IMAGE_TIMEOUT
        state_fresh = actual is not None and (now - a_stamp) < STATE_TIMEOUT

        # ---------------- vision
        self.hand = None
        self.tracking = False
        if image_fresh:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            res = self.pose.process(rgb)

            if res.pose_landmarks and res.pose_world_landmarks:
                vis = [p.visibility for p in res.pose_landmarks.landmark]
                needed = (R_SHOULDER, R_ELBOW, R_WRIST,
                          L_SHOULDER, R_HIP, L_HIP)
                if all(vis[i] > 0.6 for i in needed):
                    p = hand_in_torso_frame(res.pose_world_landmarks)
                    if p is not None:
                        self.hand = p
                        self.tracking = True

            # Build the view FIRST, then draw on it.
            #
            # Detection runs on the unmirrored frame -- mirroring it would make
            # MediaPipe label the operator's left arm "right". But drawing
            # before the flip mirrors the text too, so the overlay goes on
            # afterwards, with the landmark x coordinates mirrored to match.
            view = cv2.flip(frame, 1)
            view = cv2.resize(view,
                              (960, int(960 * view.shape[0] / view.shape[1])))
            view = np.ascontiguousarray(view)

            if res.pose_landmarks:
                self.draw(view, res.pose_landmarks)
            else:
                cv2.putText(view, "NO PERSON DETECTED", (20, 44),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (60, 60, 230), 2,
                            cv2.LINE_AA)

            cv2.imshow("hand teleop  -  right arm", view)
            cv2.waitKey(1)

        # ---------------- drop out of enabled if anything is wrong
        if self.enabled:
            if not self.tracking:
                self.enabled = False
                self.reason = "FROZE: lost sight of your arm"
                self.go.config(bg="#2d7a3e", text=self.idle_text)
            elif not state_fresh:
                self.enabled = False
                self.reason = "FROZE: lost /joint_states"
                self.go.config(bg="#2d7a3e", text=self.idle_text)

        # ---------------- homing: drive to the home pose, same rate limiter
        if self.homing:
            if not state_fresh:
                self.homing = False
                self.reason = "homing stopped: lost /joint_states"
                self.home_btn.config(text="0.  GO TO HOME POSE")
            else:
                alpha = 1.0 - math.exp(-2.0 * math.pi * self.args.filter_hz * dt)
                max_step = self.args.max_speed * dt
                for i in range(6):
                    self.filtered[i] += alpha * (HOME_DEG[i] - self.filtered[i])
                    step = clamp(self.filtered[i] - self.command[i],
                                 -max_step, max_step)
                    self.command[i] += step

                if not self.args.dry_run:
                    self.node.send([math.radians(a) for a in self.command], dt)

                err = max(abs(c - h) for c, h in zip(self.command, HOME_DEG))
                if err < 0.5:
                    self.homing = False
                    self.reason = "at home - now capture a neutral pose"
                    self.home_btn.config(text="0.  GO TO HOME POSE")

        # ---------------- hand position -> tool target -> IK
        self.hits = []
        self.target_pos = None
        ik_ok = True

        if self.hand is not None and self.neutral_pos is not None:
            scale = float(self.scale.get())
            self.scale_lab.config(text=f"{scale:.2f} x")
            off_hits = []

            if self.mode.get() == "cylindrical":
                # ---- CYLINDRICAL
                #
                # Your arm sweeps in an arc about your shoulder; the robot
                # sweeps in an arc about its base. Matching those directly
                # makes base rotation as easy as swinging your arm.
                #
                # In Cartesian mode the base is driven by a LINEAR sideways
                # offset, capped at +/-300 mm. At a 696 mm radius that is only
                # about +/-25 degrees of j1. Here the full sweep of your arm
                # maps straight onto the full sweep of the base.
                rho, phi, hy = hand_cylindrical(self.hand)
                rho0, phi0, hy0 = self.neutral_hand_cyl

                # Azimuth is undefined when the hand is near your vertical
                # axis, so hold the last good value rather than letting it
                # spin on noise.
                if rho < 0.15:
                    phi = self.last_phi
                    off_hits.append("azimuth held")
                else:
                    self.last_phi = phi

                r0, th0, z0 = self.neutral_cyl

                d_theta = (phi - phi0) * float(self.azim_gain.get())
                if self.azim_inv.get():
                    d_theta = -d_theta
                d_theta = clamp(d_theta,
                                -math.radians(self.args.max_azimuth),
                                math.radians(self.args.max_azimuth))
                if abs(d_theta) >= math.radians(self.args.max_azimuth) - 1e-6:
                    off_hits.append("azimuth")

                d_r = (rho - rho0) * 1000.0 * scale
                d_z = (hy - hy0) * 1000.0 * scale
                if self.radial_inv.get():
                    d_r = -d_r

                d_r = clamp(d_r, -self.args.max_offset, self.args.max_offset)
                d_z = clamp(d_z, -self.args.max_offset, self.args.max_offset)

                raw = from_cylindrical(r0 + d_r, th0 + d_theta, z0 + d_z)
                self.cyl_readout = (r0 + d_r, math.degrees(th0 + d_theta),
                                    z0 + d_z)
            else:
                # ---- CARTESIAN
                delta = self.hand - self.neutral_hand    # metres, torso frame
                comp = {"right": delta[0], "up": delta[1], "forward": delta[2]}

                d = []
                for axis in ("X", "Y", "Z"):
                    v = comp[self.axis_src[axis].get()]
                    if self.axis_inv[axis].get():
                        v = -v
                    d.append(v * 1000.0 * scale)          # metres -> mm

                delta_mm, off_hits = clamp_offset(np.array(d),
                                                  self.args.max_offset)
                raw = self.neutral_pos + delta_mm
                self.cyl_readout = None

            self.target_pos, box_hits = clamp_workspace(raw)
            self.hits = off_hits + box_hits

            if self.enabled and state_fresh:
                T = np.eye(4)
                T[:3, :3] = self.neutral_rot
                T[:3, 3] = self.target_pos

                sols = K.filter_limits(K.ik(T))
                seed = [math.radians(a) for a in self.command]
                best = K.closest_solution(sols, seed)
                if best is None:
                    ik_ok = False
                    self.reason = "no IK solution for that position"
                else:
                    desired = [math.degrees(a) for a in best]

                    # Reject configuration flips.
                    #
                    # Several joint solutions reach the same tool pose, and the
                    # nearest one can still be a different arm configuration --
                    # elbow up instead of down, say. Executing that swings the
                    # whole arm through a wide arc to reach the same point.
                    #
                    # The comparison is against the PREVIOUS IK result, not
                    # against the current command. The command deliberately
                    # lags behind the target because of the rate limiter, so
                    # comparing to it would make any sustained movement look
                    # like a flip and freeze the arm a second or two in.
                    # Between consecutive frames the target barely moves, so a
                    # large change here really is a branch change.
                    jump = max(abs(d - c)
                               for d, c in zip(desired, self.last_desired))
                    if jump > self.args.max_jump:
                        ik_ok = False
                        self.reason = (f"held: that would flip the arm "
                                       f"configuration ({jump:.0f} deg jump)")
                        desired = None

                if best is not None and desired is not None:
                    self.last_desired = list(desired)
                    alpha = 1.0 - math.exp(-2.0 * math.pi *
                                           self.args.filter_hz * dt)
                    max_step = self.args.max_speed * dt
                    for i in range(6):
                        lo, hi = K.SOFT_LIMITS_DEG[i]
                        self.filtered[i] += alpha * (desired[i] - self.filtered[i])
                        step = clamp(self.filtered[i] - self.command[i],
                                     -max_step, max_step)
                        self.command[i] = clamp(self.command[i] + step, lo, hi)

                    if not self.args.dry_run:
                        self.node.send([math.radians(a) for a in self.command],
                                       dt)

        # ---------------- display
        for axis, i in (("X", 0), ("Y", 1), ("Z", 2)):
            self.pos_labels[axis].config(
                text="---" if self.target_pos is None
                else f"{self.target_pos[i]:8.1f}")
        self.limit_lab.config(
            text="" if not self.hits else "LIMIT: " + ", ".join(self.hits))

        self.azim_lab.config(text=f"{float(self.azim_gain.get()):.2f} x")
        if getattr(self, "cyl_readout", None) is None:
            self.cyl_lab.config(text="")
        else:
            r, th, z = self.cyl_readout
            self.cyl_lab.config(
                text=f"radius {r:7.1f} mm     base angle {th:7.1f} deg"
                     f"     height {z:7.1f} mm")

        for i in range(6):
            c = self.command[i] if self.command else None
            a = math.degrees(actual[i]) if actual else None
            self.cells[i][0].config(text="---" if c is None else f"{c:8.2f}")
            self.cells[i][1].config(text="---" if a is None else f"{a:8.2f}")
            if c is not None and a is not None:
                e = c - a
                self.cells[i][2].config(text=f"{e:8.2f}",
                                        fg="#c33" if abs(e) > 5 else "#555")
            else:
                self.cells[i][2].config(text="---", fg="#555")

        if self.homing:
            self.status.config(text="HOMING", fg="#1f5fa9")
        elif self.enabled and ik_ok:
            self.status.config(text="MOVING", fg="#a93226")
        elif not image_fresh:
            self.status.config(text="no camera images", fg="#b26a00")
        elif not self.tracking:
            self.status.config(text="cannot see your arm", fg="#b26a00")
        elif not state_fresh:
            self.status.config(text="no /joint_states", fg="#b26a00")
        elif self.neutral_pos is None:
            self.status.config(text="capture a neutral pose", fg="#1f5fa9")
        else:
            self.status.config(text="ready", fg="#1a7f37")

        self.detail.config(text=self.reason)
        self.root.after(int(1000.0 / LOOP_HZ), self.tick)

    # --- drawing ----------------------------------------------------------
    def draw(self, img, landmarks):
        """Draw the right-arm stick figure onto the already-mirrored view."""
        h, w = img.shape[:2]
        # Landmarks come from the unmirrored frame, so mirror x to match.
        px = [(int((1.0 - p.x) * w), int(p.y * h)) for p in landmarks.landmark]

        ok = self.tracking
        col = (80, 220, 120) if ok else (60, 60, 230)

        for a, b in ((R_SHOULDER, R_ELBOW), (R_ELBOW, R_WRIST)):
            cv2.line(img, px[a], px[b], col, 6, cv2.LINE_AA)
        for idx, name in ((R_SHOULDER, "shoulder"), (R_ELBOW, "elbow"),
                          (R_WRIST, "HAND")):
            cv2.circle(img, px[idx], 10, col, -1, cv2.LINE_AA)
            cv2.circle(img, px[idx], 10, (20, 20, 20), 2, cv2.LINE_AA)
            cv2.putText(img, name, (px[idx][0] + 16, px[idx][1] + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 2,
                        cv2.LINE_AA)

        cv2.putText(img, "TRACKED" if ok else "NOT TRACKED", (20, 44),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2, cv2.LINE_AA)


# --------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true",
                    help="compute everything, publish nothing")
    ap.add_argument("--hold", action="store_true",
                    help="hold-to-move instead of click-to-toggle")
    ap.add_argument("--scale", type=float, default=DEFAULT_SCALE,
                    help=f"hand motion amplification (default {DEFAULT_SCALE})")
    ap.add_argument("--max-speed", type=float, default=MAX_SPEED_DEG_S,
                    help=f"max degrees per second per joint "
                         f"(default {MAX_SPEED_DEG_S:.0f})")
    ap.add_argument("--filter-hz", type=float, default=FILTER_HZ,
                    help=f"low-pass cutoff (default {FILTER_HZ:.1f})")
    ap.add_argument("--mode", choices=("cylindrical", "cartesian"),
                    default="cylindrical",
                    help="cylindrical maps your arm's sweep onto base "
                         "rotation; cartesian maps hand XYZ to tool XYZ "
                         "(default cylindrical)")
    ap.add_argument("--max-azimuth", type=float, default=MAX_AZIMUTH_DEG,
                    help=f"max base rotation from neutral, degrees "
                         f"(default {MAX_AZIMUTH_DEG:.0f})")
    ap.add_argument("--max-offset", type=float, default=MAX_OFFSET_MM,
                    help=f"max millimetres the tool may stray from the "
                         f"neutral position, per axis "
                         f"(default {MAX_OFFSET_MM:.0f})")
    ap.add_argument("--max-jump", type=float, default=MAX_JUMP_DEG,
                    help=f"refuse IK solutions needing more than this many "
                         f"degrees on any joint, which would be a "
                         f"configuration flip (default {MAX_JUMP_DEG:.0f})")
    args = ap.parse_args()

    if args.max_speed <= 0 or args.max_speed > 45:
        sys.exit("--max-speed must be between 0 and 45 deg/s.")

    rclpy.init()
    node = TeleopNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    root = tk.Tk()
    App(root, node, args)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
