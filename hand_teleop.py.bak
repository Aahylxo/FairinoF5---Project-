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
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from builtin_interfaces.msg import Duration

# Our own modules, which live beside this file.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import fr5_kinematics as K
except ImportError:
    sys.exit("fr5_kinematics.py not found. It must be in the same folder as "
             "this script.")
try:
    import mediapipe as _mp
    mp_hands = _mp.solutions.hands
    mp_draw_styles = _mp.solutions.drawing_styles
except Exception as exc:                                  # noqa: BLE001
    sys.exit(f"mediapipe hands unavailable: {exc}")

try:
    from arm_tracker import (torso_frame, mp_pose, mp_draw,
                             R_SHOULDER, R_ELBOW, R_WRIST,
                             L_SHOULDER, R_HIP, L_HIP)
except ImportError as exc:
    sys.exit(f"could not import from arm_tracker.py: {exc}\n"
             "It must be in the same folder as this script.")

# --- MediaPipe Hands landmark indices
H_WRIST = 0
H_THUMB_TIP = 4
H_INDEX_MCP = 5
H_INDEX_TIP = 8
H_MIDDLE_MCP = 9
H_PINKY_MCP = 17

# How hand movement in the image maps to robot motion.
#
# Image coordinates are normalised 0..1 across the frame, so these say what a
# full sweep of the frame is worth. Apparent hand SIZE stands in for distance:
# the hand gets bigger as it comes towards the camera, and the ratio to its
# size at capture is a scale-free measure of how much closer it is.
DEG_PER_FRAME_WIDTH = 140.0     # degrees of base rotation, edge to edge
MM_PER_FRAME_HEIGHT = 700.0     # millimetres of tool height, top to bottom
MM_PER_SIZE_DOUBLING = 700.0    # millimetres of reach per doubling in size

# Pinch detection. The thumb-to-index distance is divided by the hand's own
# size, so it works at any distance from the camera.
PINCH_ON = 0.45                 # below this, the pinch is closed
PINCH_OFF = 0.70                # above this, it is open (hysteresis)

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
CONTROLLER_TOPIC = "/fairino5_controller/joint_trajectory"
IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/camera/color/camera_info"

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
MAX_SPEED_DEG_S = 5.0
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

# Smoothing applied to the measured hand position, in Hz. Lower is smoother.
#
# The torso reference frame is built from the shoulder and hip landmarks, and
# those wobble by a few pixels every frame. That makes the frame itself rotate
# slightly, so the computed hand angle drifts even when the operator is
# perfectly still. Smoothing here kills it at the source, before it reaches
# the mapping or the IK.
HAND_SMOOTH_HZ = 2.5

# Movement below this, in millimetres, is treated as noise and ignored.
#
# Without a deadband the target never settles: it jitters by a few millimetres
# forever, and the robot faithfully follows every bit of it. With one, holding
# still means the robot holds still.
DEADBAND_MM = 12.0

# How far the command is allowed to get ahead of where the arm actually is.
#
# 0 means "work it out from the speed limit", which is what you want. The
# controller is told to reach each point within one horizon, so a lead of
# (max_speed x horizon) is exactly the speed limit, and anything larger asks
# the arm to move faster than it is allowed to.
#
# Without this the controller runs open-loop: it rate-limits from its own
# previous command, so if the real arm lags the command marches on regardless.
# Each new goal then asks the robot to close the whole gap within one cycle,
# demanding a velocity far above the rate limit -- the arm overshoots,
# corrects, overshoots, and oscillates in place. A simulated arm never shows
# this because it reaches every commanded position instantly.
MAX_LEAD_DEG = 0.0      # 0 = derive it from max_speed

# If the arm falls further behind than this, it is not following and something
# is wrong. Stop rather than keep commanding.
AUTO_STOP_ERROR_DEG = 8.0

# The FR5 home pose, degrees. Tool ends up around (-497, -102, 477), which is
# 696 mm from the base -- comfortably central, with room to move in every
# direction. Capturing a neutral pose from anywhere near a workspace limit is
# what makes teleoperation feel like it is constantly jammed.
HOME_DEG = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# --------------------------------------------------------------------- ROS
class TeleopNode(Node):

    def __init__(self, args):
        super().__init__("fr5_hand_teleop")
        self.args = args
        self.lock = threading.Lock()
        self.frame = None
        self.frame_stamp = 0.0
        self.actual = None               # joint angles, radians, j1..j6
        self.actual_stamp = 0.0
        self.cap = None
        self.depth = None                # aligned depth frame, millimetres
        self.depth_stamp = 0.0
        self.intrinsics = None           # (fx, fy, cx, cy)

        if args.source == "ros":
            # Images arrive from the RealSense node over a ROS topic.
            try:
                from cv_bridge import CvBridge
                self.bridge = CvBridge()
            except ImportError:
                sys.exit("cv_bridge is not installed.  Run:\n"
                         "  sudo apt install -y ros-humble-cv-bridge")
            self.create_subscription(Image, args.topic, self.on_image, 1)
            self.get_logger().info(f"reading images from {args.topic}")

            if args.use_depth:
                from sensor_msgs.msg import CameraInfo
                self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, 1)
                self.create_subscription(CameraInfo, INFO_TOPIC,
                                         self.on_info, 1)
                self.get_logger().info(
                    f"using measured depth from {DEPTH_TOPIC}")
        else:
            # Read a plain webcam directly. Useful when the RealSense is not
            # available -- the laptop's built-in camera works once it has been
            # forwarded into WSL with usbipd.
            self.cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(args.device)
            if not self.cap.isOpened():
                sys.exit(f"could not open webcam {args.device}.\n"
                         "  Check 'ls /dev/video*' and try --device 1.\n"
                         "  The camera must be attached to WSL with usbipd.")

            # Ask for MJPEG. Forwarded over USB/IP, raw YUYV frames often
            # arrive only partly filled, which shows up as a mostly-green
            # image. MJPEG is compressed, so a frame either arrives whole or
            # not at all.
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # Let the camera settle: the first few frames are often junk.
            for _ in range(5):
                self.cap.read()

            ok, probe = self.cap.read()
            fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            fourcc_s = "".join(chr((fourcc >> (8 * i)) & 0xFF) for i in range(4))
            if ok and probe is not None:
                h, w = probe.shape[:2]
                self.get_logger().info(
                    f"webcam {args.device}: {w}x{h}, format {fourcc_s}")
            else:
                self.get_logger().warn(
                    f"webcam {args.device} opened but returned no frame. "
                    f"Try --device 1.")

        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.pub = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/fr5_workspace", 1)

    def on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:                                     # noqa: BLE001
            return
        with self.lock:
            self.frame = img
            self.frame_stamp = time.time()

    def on_depth(self, msg):
        """Aligned depth frame. 16-bit, in millimetres, same size as colour."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:                                     # noqa: BLE001
            return
        with self.lock:
            self.depth = img
            self.depth_stamp = time.time()

    def on_info(self, msg):
        """Camera intrinsics, needed to turn a pixel plus a depth into metres."""
        k = msg.k
        if k[0] > 0:
            self.intrinsics = (k[0], k[4], k[2], k[5])        # fx, fy, cx, cy

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

    def publish_markers(self, target=None):
        """
        Draw the safe workspace, and the point the tool is being sent to, in
        RViz.

        The limits were previously invisible: you only found one by hitting it
        and reading a message. Shown as shapes, people simply stay inside them.
        """
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def base(idx, kind):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = now
            m.ns = "fr5_workspace"
            m.id = idx
            m.type = kind
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        z_min, z_max = WORKSPACE["z_min"], WORKSPACE["z_max"]
        r_max, r_min = WORKSPACE["r_max"], WORKSPACE["r_min"]
        mid = (z_min + z_max) / 2000.0
        height = (z_max - z_min) / 1000.0

        # Outer reach limit
        outer = base(0, Marker.CYLINDER)
        outer.pose.position.z = mid
        outer.scale = Vector3(x=2 * r_max / 1000.0, y=2 * r_max / 1000.0,
                              z=height)
        outer.color = ColorRGBA(r=0.25, g=0.55, b=0.95, a=0.07)
        arr.markers.append(outer)

        # The floor guard -- the limit that matters most
        floor = base(1, Marker.CYLINDER)
        floor.pose.position.z = z_min / 1000.0
        floor.scale = Vector3(x=2 * r_max / 1000.0, y=2 * r_max / 1000.0,
                              z=0.004)
        floor.color = ColorRGBA(r=0.95, g=0.35, b=0.2, a=0.35)
        arr.markers.append(floor)

        # The column above the base, which is kept clear
        inner = base(2, Marker.CYLINDER)
        inner.pose.position.z = mid
        inner.scale = Vector3(x=2 * r_min / 1000.0, y=2 * r_min / 1000.0,
                              z=height)
        inner.color = ColorRGBA(r=0.95, g=0.6, b=0.2, a=0.10)
        arr.markers.append(inner)

        # Where the tool is being commanded to go
        dot = base(3, Marker.SPHERE)
        if target is None:
            dot.action = Marker.DELETE
        else:
            dot.pose.position = Point(x=target[0] / 1000.0,
                                      y=target[1] / 1000.0,
                                      z=target[2] / 1000.0)
            dot.scale = Vector3(x=0.05, y=0.05, z=0.05)
            dot.color = ColorRGBA(r=0.1, g=0.9, b=0.4, a=0.9)
        arr.markers.append(dot)

        self.marker_pub.publish(arr)

    def hand_point(self, u_norm, v_norm, patch=7):
        """
        Turn a normalised image position into a metric 3D point, using the
        measured depth and the camera intrinsics.

        Returns (x, y, z) in metres in the camera frame -- x right, y down,
        z away from the camera -- or None if there is no usable depth there.

        A median over a small patch is used rather than the single pixel:
        depth images have holes at edges and on dark or shiny surfaces, and a
        hole reads as zero, which would otherwise look like "right at the
        lens".
        """
        with self.lock:
            depth = None if self.depth is None else self.depth
            intr = self.intrinsics
            fresh = (time.time() - self.depth_stamp) < 1.0
        if depth is None or intr is None or not fresh:
            return None

        h, w = depth.shape[:2]
        u = int(round(u_norm * w))
        v = int(round(v_norm * h))
        if not (0 <= u < w and 0 <= v < h):
            return None

        half = patch // 2
        window = depth[max(0, v - half):v + half + 1,
                       max(0, u - half):u + half + 1]
        valid = window[window > 0]
        if valid.size < 4:
            return None

        z = float(np.median(valid)) / 1000.0                  # mm -> m
        if not 0.1 < z < 4.0:
            return None

        fx, fy, cx, cy = intr
        return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])

    def grab(self):
        if self.cap is not None:
            ok, img = self.cap.read()
            # A partly-transferred frame comes through as a flat block of
            # colour. Drop it rather than feeding MediaPipe nonsense.
            if ok and img is not None and float(img.std()) > 3.0:
                self.frame = img
                self.frame_stamp = time.time()
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            return (frame, self.frame_stamp,
                    None if self.actual is None else list(self.actual),
                    self.actual_stamp)

    def close(self):
        if self.cap is not None:
            self.cap.release()

    def send(self, q_rad, dt):
        msg = JointTrajectory()
        msg.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(a) for a in q_rad]
        # A slightly longer horizon than the publish period, so
        # consecutive goals blend rather than fight each other.
        horizon = max(0.1, dt * 2.0)
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


def hand_measures(landmarks):
    """
    Measure the hand from MediaPipe Hands landmarks.

    Returns (x, y, size, pinch):
        x, y   wrist position in the frame, 0..1
        size   palm width in frame units -- bigger means closer to the camera
        pinch  thumb-to-index distance divided by hand size, so it means the
               same whatever distance the hand is at

    Palm width is used for size rather than, say, wrist-to-fingertip, because
    it barely changes as the fingers move. Curl your fingers and the palm is
    still the palm.
    """
    lm = landmarks.landmark

    def pt(i):
        return np.array([lm[i].x, lm[i].y])

    wrist = pt(H_WRIST)
    index_mcp = pt(H_INDEX_MCP)
    pinky_mcp = pt(H_PINKY_MCP)
    middle_mcp = pt(H_MIDDLE_MCP)

    # Palm size: across the knuckles, and wrist to middle knuckle. Averaging
    # the two keeps it steady as the hand rotates.
    span = float(np.linalg.norm(index_mcp - pinky_mcp))
    length = float(np.linalg.norm(middle_mcp - wrist))
    size = 0.5 * (span + length)
    if size < 1e-4:
        return None

    pinch = float(np.linalg.norm(pt(H_THUMB_TIP) - pt(H_INDEX_TIP))) / size

    # Report the hand's centre rather than the wrist: it sits in the middle of
    # what you see, so it feels like the thing you are moving.
    cx, cy = (wrist + middle_mcp) / 2.0
    return float(cx), float(cy), size, pinch


def torso_yaw(world_landmarks):
    """
    Which way the operator's shoulders are facing, in radians.

    Taken from the line between the two shoulders, projected onto the
    horizontal plane. Turning the body changes it; moving an arm does not.

    This deliberately uses WORLD coordinates, not the torso frame -- the torso
    frame is built from these same landmarks, so body yaw is zero in it by
    construction.
    """
    lm = np.array([[p.x, p.y, p.z] for p in world_landmarks.landmark])
    across = lm[R_SHOULDER] - lm[L_SHOULDER]
    # MediaPipe world coordinates have y vertical, so x-z is the ground plane.
    if abs(across[0]) < 1e-6 and abs(across[2]) < 1e-6:
        return None
    return math.atan2(across[2], across[0])


def arm_metrics(world_landmarks):
    """
    How far the hand is from the shoulder, and how high it is.

    reach   straight-line shoulder-to-hand distance, in metres. Grows as the
            elbow straightens and the arm pushes out, which is what the
            operator feels as "extending".

            Note that elbow-to-hand is the forearm and never changes length,
            so it cannot drive anything. Shoulder-to-hand can.

    height  how far the hand is above the shoulder, along the spine.
    """
    lm = np.array([[p.x, p.y, p.z] for p in world_landmarks.landmark])
    frame = torso_frame(lm)
    if frame is None:
        return None
    _, R = frame

    arm = lm[R_WRIST] - lm[R_SHOULDER]
    reach = float(np.linalg.norm(arm))
    height = float(np.dot(arm, R[:, 1]))        # along the body's up axis
    return reach, height


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

        self.use_hands = (args.tracker == "hands")
        if self.use_hands:
            self.pose = None
            self.hands = mp_hands.Hands(max_num_hands=1,
                                        model_complexity=1,
                                        min_detection_confidence=0.6,
                                        min_tracking_confidence=0.6)
        else:
            self.hands = None
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
        self.hand_smooth = None        # low-passed hand position
        self.held_target = None        # target held through the deadband
        self.yaw = None                # smoothed torso yaw
        self.reach = None              # smoothed shoulder-to-hand distance
        self.height = None             # smoothed hand height above shoulder
        self.neutral_torso = None      # (yaw, reach, height) at capture
        self.hand_m = None             # smoothed (x, y, size) from Hands
        self.pinch_closed = False      # pinch state, with hysteresis
        self.neutral_hand_m = None     # (x, y, size) at capture
        self.hand_xyz = None           # smoothed metric hand point, metres
        self.neutral_xyz = None        # metric hand point at capture
        self.depth_ok = False          # is measured depth available
        self._home_warned = False      # has the away-from-home warning fired
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
                ("Torso  (turn your body -> base rotation)", "torso",
                 "arm does reach and height; body does rotation"),
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
        self.cyl_frame = ttk.LabelFrame(root, text=" rotation and reach ")
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

        if self.use_hands:
            mframe.pack_forget()
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
            self.reason = ("cannot see your hand - hold it up to the camera"
                           if self.use_hands
                           else "cannot see your arm - step back into frame")
            return

        T = K.fk(actual)
        self.neutral_hand = np.array(self.hand, dtype=float)
        self.neutral_pos = T[:3, 3].copy()
        self.neutral_rot = T[:3, :3].copy()
        self.held_target = None
        self.neutral_hand_m = (None if self.hand_m is None
                               else tuple(self.hand_m))
        self.neutral_xyz = (None if self.hand_xyz is None
                            else self.hand_xyz.copy())
        self.neutral_torso = (None if self.yaw is None
                              else (self.yaw, self.reach, self.height))
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
        if self.use_hands or self.mode.get() in ("torso", "cylindrical"):
            self.cyl_frame.pack(fill="x", padx=10, pady=4,
                                before=self.pos_frame)
        else:
            self.cart_frame.pack(fill="x", padx=10, pady=6,
                                 before=self.pos_frame)
        self.reason = f"{self.mode.get()} mode - capture a neutral pose"

    def check_home(self, actual, state_fresh):
        """
        Warn when the arm is parked somewhere awkward.

        Capturing a neutral pose from a stretched-out position is the single
        most common way to make this thing feel broken: every direction hits a
        limit immediately. So until the arm is near home, the capture button
        is disabled and the home button is highlighted.

        It does NOT home by itself. Moving a 22 kg arm because a program
        started is not a decision software should take -- GO TO HOME is a
        joint-space move that does not avoid obstacles, so a person has to
        look at the robot and press it.
        """
        if not state_fresh or self.enabled or self.homing:
            return

        off = max(abs(math.degrees(a) - h) for a, h in zip(actual, HOME_DEG))
        near_home = off < self.args.home_tolerance

        if near_home:
            if self._home_warned:
                self._home_warned = False
                self.home_btn.config(bg="#4a6fa5", text="0.  GO TO HOME POSE")
                self.cap_btn.config(state="normal")
                self.reason = "at home - capture a neutral pose"
        elif not self._home_warned:
            self._home_warned = True
            self.home_btn.config(bg="#c0392b",
                                 text=f"0.  ARM IS {off:.0f} deg FROM HOME "
                                      f"-  PRESS TO HOME IT")
            if not self.args.allow_any_start:
                self.cap_btn.config(state="disabled")
            self.reason = ("the arm is away from home - check the path is "
                           "clear, then press the red button")

    def lead_for(self, dt):
        """
        How far ahead of the real arm the command may sit.

        The controller is asked to reach each point within `horizon` seconds,
        so a gap of (max_speed x horizon) demands exactly max_speed. A little
        margin is allowed so small tracking errors do not stall the motion.
        """
        if self.args.max_lead > 0:
            return self.args.max_lead
        horizon = max(0.1, dt * 2.0)
        return max(0.3, self.args.max_speed * horizon * 1.2)

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
            self.reason = ("cannot see your hand" if self.use_hands
                           else "cannot see your arm - step back into frame")
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
        for solution in (self.pose, self.hands):
            try:
                if solution is not None:
                    solution.close()
            except Exception:                                 # noqa: BLE001
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
        if not image_fresh:
            self.hand_smooth = None
            self.yaw = self.reach = self.height = None
        if image_fresh and self.use_hands:
            self.tick_hands(frame, dt)
        elif image_fresh:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            res = self.pose.process(rgb)

            if res.pose_landmarks and res.pose_world_landmarks:
                vis = [p.visibility for p in res.pose_landmarks.landmark]
                needed = (R_SHOULDER, R_ELBOW, R_WRIST,
                          L_SHOULDER, R_HIP, L_HIP)
                if all(vis[i] > 0.6 for i in needed):
                    # --- torso-mode quantities, smoothed the same way.
                    # Body yaw comes from the two shoulder landmarks, which
                    # are far apart and well tracked, so it is much steadier
                    # than anything derived from the hand alone.
                    yaw = torso_yaw(res.pose_world_landmarks)
                    met = arm_metrics(res.pose_world_landmarks)
                    if yaw is not None and met is not None:
                        a = 1.0 - math.exp(-2.0 * math.pi *
                                           self.args.hand_smooth_hz * dt)
                        reach, height = met
                        if self.yaw is None:
                            self.yaw, self.reach, self.height = yaw, reach, height
                        else:
                            # Wrap the yaw difference so it never jumps 360.
                            self.yaw += a * ((yaw - self.yaw + math.pi)
                                             % (2 * math.pi) - math.pi)
                            self.reach += a * (reach - self.reach)
                            self.height += a * (height - self.height)

                    p = hand_in_torso_frame(res.pose_world_landmarks)
                    if p is not None:
                        # Low-pass the measured position before anything uses
                        # it. Everything downstream -- mapping, IK, the
                        # deadband -- then works from a steady number.
                        if self.hand_smooth is None:
                            self.hand_smooth = np.array(p, dtype=float)
                        else:
                            a = 1.0 - math.exp(-2.0 * math.pi *
                                               self.args.hand_smooth_hz * dt)
                            self.hand_smooth += a * (p - self.hand_smooth)
                        self.hand = self.hand_smooth.copy()
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
                self.reason = ("FROZE: lost sight of your hand"
                               if self.use_hands
                               else "FROZE: lost sight of your arm")
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
                actual_deg = [math.degrees(a) for a in actual]
                for i in range(6):
                    self.filtered[i] += alpha * (HOME_DEG[i] - self.filtered[i])
                    step = clamp(self.filtered[i] - self.command[i],
                                 -max_step, max_step)
                    self.command[i] += step
                    lead = self.lead_for(dt)
                    self.command[i] = clamp(self.command[i],
                                            actual_deg[i] - lead,
                                            actual_deg[i] + lead)

                worst = max(abs(c - a)
                            for c, a in zip(self.command, actual_deg))
                if worst > self.args.auto_stop:
                    self.homing = False
                    self.reason = (f"HOMING STOPPED: the arm is not following "
                                   f"({worst:.1f} deg behind)")
                    self.home_btn.config(text="0.  GO TO HOME POSE")
                elif not self.args.dry_run:
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

            if self.use_hands:
                # ---- HANDS
                #
                # Everything comes from the hand itself, close to the camera,
                # where the landmarks are precise:
                #
                #   across the frame  -> base rotation
                #   up the frame      -> tool height
                #   apparent size     -> reach  (bigger = nearer = further out)
                #
                # Size is a ratio against the size at capture, so it is
                # independent of hand size and camera.
                if self.neutral_hand_m is None:
                    raw = self.neutral_pos
                    self.cyl_readout = None
                    off_hits.append("recapture neutral")
                elif (self.args.use_depth and self.depth_ok
                      and self.neutral_xyz is not None):
                    # ---- measured 3D hand position
                    #
                    # The depth camera reports the real distance to the hand,
                    # so all three axes are measurements in metres rather than
                    # inferences from how big the hand looks. Reach in
                    # particular stops being a guess.
                    r0, th0, z0 = self.neutral_cyl
                    d = (self.hand_xyz - self.neutral_xyz) * 1000.0 * scale

                    # camera frame: x right, y DOWN, z away from the lens
                    lateral, drop, away = d[0], d[1], d[2]

                    d_theta = (math.atan2(-lateral, max(r0, 1.0))
                               * float(self.azim_gain.get()))
                    if self.azim_inv.get():
                        d_theta = -d_theta
                    d_theta = clamp(d_theta,
                                    -math.radians(self.args.max_azimuth),
                                    math.radians(self.args.max_azimuth))
                    if abs(d_theta) >= math.radians(self.args.max_azimuth) - 1e-6:
                        off_hits.append("rotation")

                    # coming towards the lens means extending outwards
                    d_r = -away
                    if self.radial_inv.get():
                        d_r = -d_r
                    d_z = -drop

                    d_r = clamp(d_r, -self.args.max_offset, self.args.max_offset)
                    d_z = clamp(d_z, -self.args.max_offset, self.args.max_offset)

                    raw = from_cylindrical(r0 + d_r, th0 + d_theta, z0 + d_z)
                    self.cyl_readout = (r0 + d_r, math.degrees(th0 + d_theta),
                                        z0 + d_z)
                else:
                    x0, y0, size0 = self.neutral_hand_m
                    x, y, size = self.hand_m
                    r0, th0, z0 = self.neutral_cyl

                    d_theta = (math.radians(DEG_PER_FRAME_WIDTH) * (x - x0)
                               * float(self.azim_gain.get()))
                    if not self.azim_inv.get():
                        # Mirrored view: moving your hand right on screen
                        # means x decreases, so flip for an intuitive match.
                        d_theta = -d_theta
                    d_theta = clamp(d_theta,
                                    -math.radians(self.args.max_azimuth),
                                    math.radians(self.args.max_azimuth))
                    if abs(d_theta) >= math.radians(self.args.max_azimuth) - 1e-6:
                        off_hits.append("rotation")

                    # log2 so that doubling and halving move the same amount
                    ratio = max(size / max(size0, 1e-6), 1e-3)
                    d_r = (math.log2(ratio) * MM_PER_SIZE_DOUBLING * scale)
                    if self.radial_inv.get():
                        d_r = -d_r

                    # image y grows downwards, so raising the hand lowers y
                    d_z = -(y - y0) * MM_PER_FRAME_HEIGHT * scale

                    d_r = clamp(d_r, -self.args.max_offset, self.args.max_offset)
                    d_z = clamp(d_z, -self.args.max_offset, self.args.max_offset)

                    raw = from_cylindrical(r0 + d_r, th0 + d_theta, z0 + d_z)
                    self.cyl_readout = (r0 + d_r, math.degrees(th0 + d_theta),
                                        z0 + d_z)
            elif self.mode.get() == "torso":
                # ---- TORSO
                #
                # Base rotation follows the BODY, not the arm. Turn your
                # shoulders and the base turns with you; swing your arm and it
                # does not. That separation is the point: reach and height are
                # the arm's job, rotation is the body's.
                #
                # Measured against hand azimuth, body yaw is NOT less noisy
                # -- both come out around 0.85 deg of jitter for 4 mm of
                # landmark error. What it buys is independence: arm noise and
                # arm movement cannot rotate the base at all, so holding your
                # body still holds the base still no matter what your arm is
                # doing. The deadband deals with the remaining jitter.
                if self.neutral_torso is None or self.yaw is None:
                    raw = self.neutral_pos
                    self.cyl_readout = None
                    off_hits.append("recapture neutral")
                else:
                    yaw0, reach0, h0 = self.neutral_torso
                    r0, th0, z0 = self.neutral_cyl

                    d_theta = ((self.yaw - yaw0 + math.pi) % (2 * math.pi)
                               - math.pi) * float(self.azim_gain.get())
                    if self.azim_inv.get():
                        d_theta = -d_theta
                    d_theta = clamp(d_theta,
                                    -math.radians(self.args.max_azimuth),
                                    math.radians(self.args.max_azimuth))
                    if abs(d_theta) >= math.radians(self.args.max_azimuth) - 1e-6:
                        off_hits.append("rotation")

                    d_r = (self.reach - reach0) * 1000.0 * scale
                    d_z = (self.height - h0) * 1000.0 * scale
                    if self.radial_inv.get():
                        d_r = -d_r

                    d_r = clamp(d_r, -self.args.max_offset, self.args.max_offset)
                    d_z = clamp(d_z, -self.args.max_offset, self.args.max_offset)

                    raw = from_cylindrical(r0 + d_r, th0 + d_theta, z0 + d_z)
                    self.cyl_readout = (r0 + d_r, math.degrees(th0 + d_theta),
                                        z0 + d_z)
            elif self.mode.get() == "cylindrical":
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

            target, box_hits = clamp_workspace(raw)

            # Deadband: keep the previous target until the new one has moved
            # far enough to be a real movement rather than measurement noise.
            if self.held_target is None:
                self.held_target = target
            elif float(np.linalg.norm(target - self.held_target)) > \
                    self.args.deadband:
                self.held_target = target

            self.target_pos = self.held_target
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
                    actual_deg = [math.degrees(a) for a in actual]
                    lead = self.lead_for(dt)

                    for i in range(6):
                        lo, hi = K.SOFT_LIMITS_DEG[i]
                        self.filtered[i] += alpha * (desired[i] - self.filtered[i])
                        step = clamp(self.filtered[i] - self.command[i],
                                     -max_step, max_step)
                        self.command[i] = clamp(self.command[i] + step, lo, hi)

                        # Closed-loop guard: never get more than `lead` degrees
                        # ahead of the real arm. This is what stops the command
                        # running away and the arm oscillating to catch up.
                        self.command[i] = clamp(self.command[i],
                                                actual_deg[i] - lead,
                                                actual_deg[i] + lead)

                    # If the arm is a long way behind, it is not following.
                    worst = max(abs(c - a)
                                for c, a in zip(self.command, actual_deg))
                    if worst > self.args.auto_stop:
                        self.enabled = False
                        self.reason = (f"STOPPED: the arm is not following "
                                       f"({worst:.1f} deg behind)")
                        self.go.config(bg="#2d7a3e", text=self.idle_text)
                    elif not self.args.dry_run:
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
            self.status.config(
                text="cannot see your hand" if self.use_hands
                else "cannot see your arm", fg="#b26a00")
        elif not state_fresh:
            self.status.config(text="no /joint_states", fg="#b26a00")
        elif self.neutral_pos is None:
            self.status.config(text="capture a neutral pose", fg="#1f5fa9")
        else:
            self.status.config(text="ready", fg="#1a7f37")

        self.detail.config(text=self.reason)

        # Markers at a few Hz -- RViz does not need them at the loop rate.
        self._marker_count = getattr(self, "_marker_count", 0) + 1
        if self._marker_count % 5 == 0:
            self.node.publish_markers(
                self.target_pos if self.enabled else None)

        self.check_home(actual, state_fresh)
        self.root.after(int(1000.0 / LOOP_HZ), self.tick)

    def tick_hands(self, frame, dt):
        """Vision for MediaPipe Hands: measure, smooth, and draw."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        res = self.hands.process(rgb)

        view = cv2.flip(frame, 1)
        view = cv2.resize(view,
                          (960, int(960 * view.shape[0] / view.shape[1])))
        view = np.ascontiguousarray(view)
        h, w = view.shape[:2]

        if res.multi_hand_landmarks:
            lms = res.multi_hand_landmarks[0]
            m = hand_measures(lms)
            if m is not None:
                x, y, size, pinch = m

                a = 1.0 - math.exp(-2.0 * math.pi *
                                   self.args.hand_smooth_hz * dt)
                if self.hand_m is None:
                    self.hand_m = np.array([x, y, size], dtype=float)
                else:
                    self.hand_m += a * (np.array([x, y, size]) - self.hand_m)

                # Hysteresis on the pinch: one threshold to close, a higher
                # one to open. A single threshold would chatter on and off
                # while the fingers hover near it.
                if self.pinch_closed and pinch > PINCH_OFF:
                    self.pinch_closed = False
                elif not self.pinch_closed and pinch < PINCH_ON:
                    self.pinch_closed = True

                self.tracking = True
                self.hand = self.hand_m.copy()

                # Measured 3D position, when the depth stream is up. The
                # wrist is sampled rather than the palm centre: it sits on a
                # flatter, broader part of the hand, so the depth patch is
                # less likely to straddle an edge and read the wall behind.
                self.depth_ok = False
                if self.args.use_depth:
                    lmw = lms.landmark[H_WRIST]
                    pt3 = self.node.hand_point(lmw.x, lmw.y)
                    if pt3 is not None:
                        if self.hand_xyz is None:
                            self.hand_xyz = pt3
                        else:
                            self.hand_xyz += a * (pt3 - self.hand_xyz)
                        self.depth_ok = True

            # draw the skeleton, mirrored to match the view
            pts = [(int((1.0 - p.x) * w), int(p.y * h)) for p in lms.landmark]
            for a_i, b_i in mp_hands.HAND_CONNECTIONS:
                cv2.line(view, pts[a_i], pts[b_i], (90, 200, 120), 2,
                         cv2.LINE_AA)
            for i, p in enumerate(pts):
                r = 7 if i in (H_THUMB_TIP, H_INDEX_TIP) else 4
                cv2.circle(view, p, r, (255, 255, 255), -1, cv2.LINE_AA)

            if self.tracking:
                col = (60, 200, 90) if self.pinch_closed else (200, 160, 60)
                cv2.line(view, pts[H_THUMB_TIP], pts[H_INDEX_TIP], col, 3,
                         cv2.LINE_AA)
                cv2.putText(view,
                            "PINCHED" if self.pinch_closed else "open",
                            (20, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2,
                            cv2.LINE_AA)
        else:
            self.hand_m = None
            self.hand_xyz = None
            self.depth_ok = False
            self.pinch_closed = False
            cv2.putText(view, "NO HAND", (20, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (60, 60, 230), 2,
                        cv2.LINE_AA)

        if self.tracking:
            cv2.putText(view, "TRACKED", (20, 44), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (80, 220, 120), 2, cv2.LINE_AA)
            if self.args.use_depth:
                if self.depth_ok:
                    txt = f"depth {self.hand_xyz[2] * 100:.0f} cm"
                    col = (80, 220, 120)
                else:
                    txt = "no depth here"
                    col = (60, 160, 230)
                cv2.putText(view, txt, (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, col, 2,
                            cv2.LINE_AA)

        cv2.imshow("hand teleop  -  right arm", view)
        cv2.waitKey(1)

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
    ap.add_argument("--source", choices=("ros", "webcam"), default="ros",
                    help="ros reads the RealSense node's image topic; webcam "
                         "opens a plain camera directly (default ros)")
    ap.add_argument("--topic", default=IMAGE_TOPIC,
                    help="image topic when --source ros")
    ap.add_argument("--device", type=int, default=0,
                    help="camera index when --source webcam (try 1 if 0 fails)")
    ap.add_argument("--tracker", choices=("hands", "pose"), default="hands",
                    help="hands: track the hand itself, close to the camera, "
                         "with pinch-to-move (default, most precise). "
                         "pose: track the whole arm from a distance")
    ap.add_argument("--no-depth", dest="use_depth", action="store_false",
                    default=True,
                    help="do not use the depth camera; fall back to inferring "
                         "distance from how big the hand looks")
    ap.add_argument("--no-pinch", dest="pinch", action="store_false",
                    default=True,
                    help="in hands mode, use the button instead of pinching")
    ap.add_argument("--mode",
                    choices=("torso", "cylindrical", "cartesian"),
                    default="torso",
                    help="torso: body turn drives the base, arm drives reach "
                         "and height (default, steadiest). cylindrical: arm "
                         "sweep drives the base. cartesian: hand XYZ to "
                         "tool XYZ")
    ap.add_argument("--max-azimuth", type=float, default=MAX_AZIMUTH_DEG,
                    help=f"max base rotation from neutral, degrees "
                         f"(default {MAX_AZIMUTH_DEG:.0f})")
    ap.add_argument("--max-offset", type=float, default=MAX_OFFSET_MM,
                    help=f"max millimetres the tool may stray from the "
                         f"neutral position, per axis "
                         f"(default {MAX_OFFSET_MM:.0f})")
    ap.add_argument("--home-tolerance", type=float, default=15.0,
                    help="degrees from the home pose before the arm counts as "
                         "out of position (default 15)")
    ap.add_argument("--allow-any-start", action="store_true",
                    help="allow capturing a neutral pose even when the arm is "
                         "far from home. Usually a bad idea")
    ap.add_argument("--hand-smooth-hz", type=float, default=HAND_SMOOTH_HZ,
                    help=f"smoothing on the measured hand position, lower is "
                         f"steadier (default {HAND_SMOOTH_HZ:.1f})")
    ap.add_argument("--deadband", type=float, default=DEADBAND_MM,
                    help=f"ignore target movement smaller than this, in mm "
                         f"(default {DEADBAND_MM:.0f})")
    ap.add_argument("--max-lead", type=float, default=MAX_LEAD_DEG,
                    help=f"max degrees the command may get ahead of the real "
                         f"arm (default {MAX_LEAD_DEG:.1f})")
    ap.add_argument("--auto-stop", type=float, default=AUTO_STOP_ERROR_DEG,
                    help=f"stop if the arm falls this far behind "
                         f"(default {AUTO_STOP_ERROR_DEG:.0f})")
    ap.add_argument("--max-jump", type=float, default=MAX_JUMP_DEG,
                    help=f"refuse IK solutions needing more than this many "
                         f"degrees on any joint, which would be a "
                         f"configuration flip (default {MAX_JUMP_DEG:.0f})")
    args = ap.parse_args()

    if args.max_speed <= 0 or args.max_speed > 45:
        sys.exit("--max-speed must be between 0 and 45 deg/s.")

    rclpy.init()
    node = TeleopNode(args)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    root = tk.Tk()
    App(root, node, args)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
