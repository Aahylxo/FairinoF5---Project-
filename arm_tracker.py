#!/usr/bin/env python3
"""
arm_tracker.py  —  Phase 1 of FR5 arm teleoperation.

Watches the operator's RIGHT arm through the RealSense colour stream,
draws a stick figure over it, and computes joint angles.

THIS SCRIPT DOES NOT TOUCH THE ROBOT. It only looks and reports.

Angles produced (degrees):
    shoulder_elev   0   = upper arm hanging straight down beside the torso
                    90  = upper arm horizontal
                    180 = upper arm straight up
    shoulder_azim   0   = arm out to your right side
                    +90 = arm reaching forward (towards the camera)
                    -90 = arm reaching backward
    elbow_flex      180 = arm straight
                    90  = right angle at the elbow
                    0   = fully folded
    wrist_roll      rough forearm rotation estimate. NOISY — see note below.

Published on /human_arm_angles as Float32MultiArray in that order,
so Phase 2 can consume it without re-running the vision stack.

Usage
-----
    # from the running RealSense ROS node (normal case)
    python3 arm_tracker.py

    # from a plain webcam instead, for testing without the RealSense
    python3 arm_tracker.py --source webcam

    q or ESC in the window to quit.
"""

import argparse
import math
import sys
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

# ---------------------------------------------------------------- mediapipe
try:
    import mediapipe as mp
except ImportError:
    sys.exit("mediapipe is not installed.  Run:\n"
             "  pip3 install mediapipe --break-system-packages")

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils

# MediaPipe landmark indices. "RIGHT" here means the person's anatomical
# right, which appears on the LEFT of a non-mirrored camera image.
L_SHOULDER, R_SHOULDER = 11, 12
L_ELBOW,    R_ELBOW    = 13, 14
L_WRIST,    R_WRIST    = 15, 16
R_PINKY,    R_INDEX    = 18, 20
L_HIP,      R_HIP      = 23, 24

# Which landmarks we draw for the right arm stick figure
ARM_CHAIN = [(R_SHOULDER, R_ELBOW), (R_ELBOW, R_WRIST)]
TORSO_CHAIN = [(R_SHOULDER, L_SHOULDER), (R_SHOULDER, R_HIP), (L_SHOULDER, L_HIP)]


# ------------------------------------------------------------------- maths
def unit(v):
    """Normalise a vector, returning None if it has no length."""
    n = np.linalg.norm(v)
    if n < 1e-9:
        return None
    return v / n


def angle_between(v1, v2):
    """Angle between two vectors, in degrees, 0..180."""
    u1, u2 = unit(v1), unit(v2)
    if u1 is None or u2 is None:
        return None
    return math.degrees(math.acos(float(np.clip(np.dot(u1, u2), -1.0, 1.0))))


def torso_frame(lm):
    """
    Build a right-handed coordinate frame attached to the operator's torso.

    Doing the angle maths in this frame rather than in camera coordinates
    means the numbers stay the same when the operator turns or steps sideways,
    which matters a lot once these drive a robot.

        x = out to the operator's right
        y = up along the spine
        z = forward, out of the operator's chest

    Returns (origin, R) where R's columns are x, y, z, or None if the
    landmarks are too degenerate to build a frame from.
    """
    r_sh, l_sh = lm[R_SHOULDER], lm[L_SHOULDER]
    r_hp, l_hp = lm[R_HIP], lm[L_HIP]

    mid_sh = (r_sh + l_sh) / 2.0
    mid_hp = (r_hp + l_hp) / 2.0

    y = unit(mid_sh - mid_hp)          # up the spine
    x_raw = r_sh - l_sh                # across the shoulders, to the right
    if y is None:
        return None

    # Remove any up-component from x so the frame is orthogonal
    x = unit(x_raw - np.dot(x_raw, y) * y)
    if x is None:
        return None

    z = np.cross(x, y)                 # forward
    z = unit(z)
    if z is None:
        return None

    return mid_sh, np.column_stack((x, y, z))


def compute_angles(lm):
    """
    lm: (33, 3) array of MediaPipe *world* landmarks, in metres, hip-centred.
    Returns a dict of angles in degrees, with None for any that can't be found.
    """
    out = {"shoulder_elev": None, "shoulder_azim": None,
           "elbow_flex": None, "wrist_roll": None}

    frame = torso_frame(lm)
    if frame is None:
        return out
    _, R = frame

    sh, el, wr = lm[R_SHOULDER], lm[R_ELBOW], lm[R_WRIST]

    # ---- upper arm, expressed in the torso frame
    upper_arm = R.T @ (el - sh)
    ua = unit(upper_arm)
    if ua is not None:
        # Elevation measured from "straight down" (-y)
        out["shoulder_elev"] = angle_between(ua, np.array([0.0, -1.0, 0.0]))

        # Azimuth: where the arm points in the horizontal plane.
        # +x is out to the right, +z is forward.
        out["shoulder_azim"] = math.degrees(math.atan2(ua[2], ua[0]))

    # ---- elbow: angle at the elbow between upper arm and forearm
    out["elbow_flex"] = angle_between(sh - el, wr - el)

    # ---- forearm roll. Approximated from the hand's width direction
    #      projected perpendicular to the forearm. This is the least
    #      reliable number here; see the note in the module docstring.
    forearm = unit(wr - el)
    hand_width = lm[R_INDEX] - lm[R_PINKY]
    if forearm is not None:
        hw = hand_width - np.dot(hand_width, forearm) * forearm
        hw = unit(hw)
        if hw is not None:
            # Reference direction: torso "up" projected perpendicular to forearm
            up = R[:, 1]
            ref = up - np.dot(up, forearm) * forearm
            ref = unit(ref)
            if ref is not None:
                side = np.cross(ref, forearm)
                out["wrist_roll"] = math.degrees(
                    math.atan2(float(np.dot(hw, side)), float(np.dot(hw, ref))))

    return out


# ------------------------------------------------------------------ drawing
def draw_overlay(img, lm_px, vis, angles, fps, tracking_ok, min_vis, s=1.0):
    """
    Draw the right-arm stick figure and a readout panel.

    s scales the text, markers and panel with the window size, so the overlay
    stays readable when the view is enlarged.
    """
    h, w = img.shape[:2]
    thick = max(1, int(round(2 * s)))
    limb = max(2, int(round(5 * s)))
    dot = max(4, int(round(9 * s)))

    GREEN  = (80, 220, 120)
    GREY   = (130, 130, 130)
    RED    = (60, 60, 230)
    WHITE  = (240, 240, 240)
    PANEL  = (28, 28, 32)

    # --- torso, drawn faintly for context
    for a, b in TORSO_CHAIN:
        if vis[a] > min_vis and vis[b] > min_vis:
            cv2.line(img, lm_px[a], lm_px[b], GREY, thick, cv2.LINE_AA)

    # --- the right arm itself, drawn boldly
    for a, b in ARM_CHAIN:
        ok = vis[a] > min_vis and vis[b] > min_vis
        colour = GREEN if ok else RED
        cv2.line(img, lm_px[a], lm_px[b], colour, limb, cv2.LINE_AA)

    # --- joints
    for idx, name in ((R_SHOULDER, "shoulder"), (R_ELBOW, "elbow"), (R_WRIST, "wrist")):
        ok = vis[idx] > min_vis
        colour = GREEN if ok else RED
        cv2.circle(img, lm_px[idx], dot, colour, -1, cv2.LINE_AA)
        cv2.circle(img, lm_px[idx], dot, (20, 20, 20), thick, cv2.LINE_AA)
        cv2.putText(img, name,
                    (lm_px[idx][0] + int(16 * s), lm_px[idx][1] + int(6 * s)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5 * s, WHITE, thick, cv2.LINE_AA)

    # --- readout panel
    pad = int(10 * s)
    panel_w, panel_h = int(300 * s), int(170 * s)
    panel_w, panel_h = min(panel_w, w - 2 * pad), min(panel_h, h - 2 * pad)
    sub = img[pad:pad + panel_h, pad:pad + panel_w]
    sub[:] = cv2.addWeighted(sub, 0.25, np.full_like(sub, PANEL, np.uint8), 0.75, 0)

    def line(i, text, colour=WHITE):
        cv2.putText(img, text, (pad + int(14 * s), pad + int((30 + i * 27) * s)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.56 * s, colour, thick, cv2.LINE_AA)

    if tracking_ok:
        line(0, "RIGHT ARM TRACKED", GREEN)
    else:
        line(0, "NO ARM DETECTED", RED)

    def fmt(key, label):
        v = angles.get(key)
        return f"{label:<10s} ---" if v is None else f"{label:<10s} {v:7.1f}"

    line(1, fmt("shoulder_elev", "sh elev"))
    line(2, fmt("shoulder_azim", "sh azim"))
    line(3, fmt("elbow_flex",    "elbow"))
    line(4, fmt("wrist_roll",    "wrist*"), GREY)

    cv2.putText(img, f"{fps:4.1f} fps   q to quit   * wrist is unreliable",
                (int(14 * s), h - int(14 * s)), cv2.FONT_HERSHEY_SIMPLEX,
                0.45 * s, GREY, thick, cv2.LINE_AA)


# -------------------------------------------------------------------- node
class ArmTracker(Node):

    def __init__(self, args):
        super().__init__("arm_tracker")
        self.args = args
        self.min_vis = args.min_visibility

        self.pub = self.create_publisher(Float32MultiArray, "/human_arm_angles", 10)

        self.pose = mp_pose.Pose(
            model_complexity=args.model_complexity,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6,
            smooth_landmarks=True,
        )

        self._t_last = time.time()
        self._fps = 0.0
        self._frames = 0
        self._last_frame_time = None

        if args.source == "ros":
            self.bridge = None
            try:
                from cv_bridge import CvBridge
                self.bridge = CvBridge()
            except ImportError:
                sys.exit("cv_bridge is not installed.  Run:\n"
                         "  sudo apt install -y ros-humble-cv-bridge")
            self.create_subscription(Image, args.topic, self.on_image, 10)
            self.get_logger().info(f"Subscribed to {args.topic}")
            self.create_timer(5.0, self.check_stream)
        else:
            self.cap = cv2.VideoCapture(args.device)
            if not self.cap.isOpened():
                sys.exit(f"Could not open webcam {args.device}")
            self.create_timer(1.0 / 30.0, self.on_webcam_tick)
            self.get_logger().info(f"Reading webcam {args.device}")

    # --- image sources -----------------------------------------------------
    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f"Could not convert image: {exc}")
            return
        self.process(frame)

    def on_webcam_tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Webcam read failed")
            return
        self.process(frame)

    def check_stream(self):
        if self._last_frame_time is None:
            self.get_logger().warn(
                f"No images on {self.args.topic} yet. Is the RealSense node running?")

    # --- main work ---------------------------------------------------------
    def process(self, frame):
        self._last_frame_time = time.time()

        src_h, src_w = frame.shape[:2]

        # Detection runs on the UNMIRRORED frame.
        #
        # MediaPipe labels landmarks by the anatomy it sees, so running it on a
        # mirrored image makes it call the operator's LEFT arm "right". Mirror
        # only the picture we display, never the picture we measure.
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.pose.process(rgb)

        # Build the view: mirror for natural interaction, then enlarge.
        display = cv2.flip(frame, 1) if self.args.mirror else frame
        s = 1.0
        if self.args.width and self.args.width > src_w:
            s = self.args.width / float(src_w)
            display = cv2.resize(display, (int(src_w * s), int(src_h * s)),
                                 interpolation=cv2.INTER_LINEAR)
        display = np.ascontiguousarray(display)
        h, w = display.shape[:2]

        angles = {"shoulder_elev": None, "shoulder_azim": None,
                  "elbow_flex": None, "wrist_roll": None}
        tracking_ok = False

        if results.pose_landmarks and results.pose_world_landmarks:
            # Pixel coordinates for drawing, mapped onto the displayed image.
            # Landmarks come from the unmirrored frame, so mirror their x here
            # to line the skeleton up with the mirrored view.
            def to_px(p):
                x = (1.0 - p.x) if self.args.mirror else p.x
                return (int(x * w), int(p.y * h))

            lm_px = [to_px(p) for p in results.pose_landmarks.landmark]
            vis = [p.visibility for p in results.pose_landmarks.landmark]

            # Metric world coordinates, for the angle maths
            lm_world = np.array([[p.x, p.y, p.z]
                                 for p in results.pose_world_landmarks.landmark])

            arm_visible = all(vis[i] > self.min_vis
                              for i in (R_SHOULDER, R_ELBOW, R_WRIST))
            torso_visible = all(vis[i] > self.min_vis
                                for i in (L_SHOULDER, R_HIP, L_HIP))

            if arm_visible and torso_visible:
                angles = compute_angles(lm_world)
                tracking_ok = True

            draw_overlay(display, lm_px, vis, angles, self._fps,
                         tracking_ok, self.min_vis, s)
        else:
            cv2.putText(display, "NO PERSON DETECTED",
                        (int(24 * s), int(44 * s)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8 * s, (60, 60, 230), max(2, int(2 * s)), cv2.LINE_AA)

        self.publish(angles, tracking_ok)
        self.tick_fps()

        cv2.imshow("FR5 arm tracker  -  right arm", display)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            raise KeyboardInterrupt

    def publish(self, angles, tracking_ok):
        """
        Publish the four angles plus a validity flag.

        NaN is used for any angle that could not be measured. Phase 3 must
        treat NaN as 'freeze', never as zero — a zero here would command the
        robot to a real pose.
        """
        def val(k):
            v = angles.get(k)
            return float("nan") if v is None else float(v)

        msg = Float32MultiArray()
        msg.data = [val("shoulder_elev"), val("shoulder_azim"),
                    val("elbow_flex"), val("wrist_roll"),
                    1.0 if tracking_ok else 0.0]
        self.pub.publish(msg)

    def tick_fps(self):
        self._frames += 1
        now = time.time()
        dt = now - self._t_last
        if dt >= 0.5:
            self._fps = self._frames / dt
            self._frames = 0
            self._t_last = now

    def destroy_node(self):
        try:
            self.pose.close()
        except Exception:                             # noqa: BLE001
            pass
        if self.args.source == "webcam":
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=("ros", "webcam"), default="ros",
                    help="where to read images from (default: ros)")
    ap.add_argument("--topic", default="/camera/camera/color/image_raw",
                    help="ROS image topic when --source ros")
    ap.add_argument("--device", type=int, default=0,
                    help="webcam index when --source webcam")
    ap.add_argument("--mirror", action="store_true", default=True,
                    help="mirror the DISPLAY only, so moving right moves right "
                         "on screen; detection always runs unmirrored")
    ap.add_argument("--no-mirror", dest="mirror", action="store_false")
    ap.add_argument("--width", type=int, default=1280,
                    help="window width in pixels; the frame is upscaled to "
                         "this (default 1280, 0 for native size)")
    ap.add_argument("--min-visibility", type=float, default=0.6,
                    help="landmark confidence below which a joint is untrusted")
    ap.add_argument("--model-complexity", type=int, default=1, choices=(0, 1, 2),
                    help="0 fastest / 2 most accurate (default 1)")
    args = ap.parse_args()

    rclpy.init()
    node = ArmTracker(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
