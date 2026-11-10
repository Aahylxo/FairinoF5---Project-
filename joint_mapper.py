#!/usr/bin/env python3
"""
joint_mapper.py  —  Phase 2 of FR5 teleoperation.

A control panel for deciding which human arm angle drives which robot joint.

    human angle  ->  invert  ->  gain  ->  offset  ->  clamp  ->  robot joint

Reads /human_arm_angles (published by arm_tracker.py) and publishes the mapped
result on /fr5_joint_targets.

THIS DOES NOT MOVE THE ROBOT. It computes and displays target joint angles so
you can check a mapping is sane before anything acts on it. Phase 3 is what
would consume /fr5_joint_targets, and it does not exist yet.

Run it alongside the tracker:

    # terminal 1 -- camera
    ros2 launch realsense2_camera rs_launch.py \
        rgb_camera.color_profile:=640x360x15 \
        enable_depth:=false enable_infra1:=false enable_infra2:=false

    # terminal 2 -- vision
    python3 arm_tracker.py

    # terminal 3 -- this panel
    python3 joint_mapper.py

Needs tkinter:   sudo apt install -y python3-tk
"""

import json
import math
import os
import sys
import threading

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except ImportError:
    sys.exit("tkinter is not installed.  Run:\n"
             "  sudo apt install -y python3-tk")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ------------------------------------------------------------------ config
SOURCES = ["(none)", "shoulder_elev", "shoulder_azim", "elbow_flex", "wrist_roll"]

# Order of the values inside /human_arm_angles
SOURCE_INDEX = {
    "shoulder_elev": 0,
    "shoulder_azim": 1,
    "elbow_flex": 2,
    "wrist_roll": 3,
}

# Human-side ranges, degrees. Used only to show a sensible suggested gain.
HUMAN_RANGE = {
    "shoulder_elev": (0.0, 180.0),
    "shoulder_azim": (-90.0, 90.0),
    "elbow_flex": (0.0, 180.0),
    "wrist_roll": (-180.0, 180.0),
}

# FR5 SOFT limits, degrees. These are the robot's own limits, NOT the shipped
# URDF values -- the URDF wrongly gives joint 3 as +/-162 (README Part 6.3).
SOFT_LIMITS = [
    (-175.0, 175.0),
    (-265.0,  85.0),
    (-150.0, 150.0),
    (-265.0,  85.0),
    (-175.0, 175.0),
    (-175.0, 175.0),
]

# Conservative working limits, well inside the soft limits. A mapping should
# normally stay inside these; the panel warns when the output leaves them.
SAFE_LIMITS = [
    (-90.0,  90.0),
    (-180.0,  0.0),
    (-120.0, 120.0),
    (-180.0,  0.0),
    (-120.0, 120.0),
    (-120.0, 120.0),
]

HOME = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]

CONFIG_PATH = os.path.expanduser("~/fairino_380_ws/joint_mapping.json")


# -------------------------------------------------------------------- ROS
class MapperNode(Node):
    """Subscribes to the tracker, publishes the mapped joint targets."""

    def __init__(self):
        super().__init__("joint_mapper")
        self.lock = threading.Lock()
        self.angles = [float("nan")] * 4
        self.tracking = False
        self.msg_count = 0

        self.create_subscription(
            Float32MultiArray, "/human_arm_angles", self.on_angles, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, "/fr5_joint_targets", 10)

    def on_angles(self, msg):
        if len(msg.data) < 5:
            return
        with self.lock:
            self.angles = list(msg.data[:4])
            self.tracking = msg.data[4] > 0.5
            self.msg_count += 1

    def snapshot(self):
        with self.lock:
            return list(self.angles), self.tracking, self.msg_count

    def publish_targets(self, targets, valid):
        msg = Float32MultiArray()
        # six joint targets in degrees, then a validity flag
        msg.data = [float(t) for t in targets] + [1.0 if valid else 0.0]
        self.pub.publish(msg)


# -------------------------------------------------------------------- GUI
class JointRow:
    """One robot joint's mapping controls and live readout."""

    def __init__(self, parent, index, row, on_change):
        self.index = index
        self.on_change = on_change
        j = index + 1

        lo_s, hi_s = SOFT_LIMITS[index]
        lo_w, hi_w = SAFE_LIMITS[index]

        tk.Label(parent, text=f"j{j}", font=("TkDefaultFont", 11, "bold"),
                 width=4, anchor="w").grid(row=row, column=0, padx=(8, 2), pady=3)

        self.source = tk.StringVar(value="(none)")
        om = ttk.OptionMenu(parent, self.source, "(none)", *SOURCES,
                            command=lambda _: self.on_change())
        om.config(width=14)
        om.grid(row=row, column=1, padx=2)

        self.invert = tk.BooleanVar(value=False)
        ttk.Checkbutton(parent, variable=self.invert,
                        command=self.on_change).grid(row=row, column=2, padx=2)

        self.gain = tk.StringVar(value="1.00")
        e = ttk.Entry(parent, textvariable=self.gain, width=7, justify="right")
        e.grid(row=row, column=3, padx=2)
        e.bind("<KeyRelease>", lambda _: self.on_change())

        self.offset = tk.StringVar(value=f"{HOME[index]:.1f}")
        e2 = ttk.Entry(parent, textvariable=self.offset, width=8, justify="right")
        e2.grid(row=row, column=4, padx=2)
        e2.bind("<KeyRelease>", lambda _: self.on_change())

        self.out = tk.Label(parent, text="---", width=10, anchor="e",
                            font=("TkFixedFont", 11))
        self.out.grid(row=row, column=5, padx=(6, 2))

        self.status = tk.Label(parent, text="", width=16, anchor="w",
                               font=("TkDefaultFont", 9))
        self.status.grid(row=row, column=6, padx=2, sticky="w")

        tk.Label(parent, text=f"[{lo_w:.0f}, {hi_w:.0f}]  soft [{lo_s:.0f}, {hi_s:.0f}]",
                 font=("TkDefaultFont", 8), fg="#777").grid(
                     row=row, column=7, padx=(4, 8), sticky="w")

    # --- values -----------------------------------------------------------
    def numeric(self, var, fallback):
        try:
            return float(var.get())
        except ValueError:
            return fallback

    def compute(self, human, neutral, use_neutral):
        """
        Returns (value, state) where state is one of:
            "none"    no source selected
            "lost"    source selected but not tracked this frame
            "ok"      inside the conservative working limits
            "warn"    outside working limits, still inside soft limits
            "clamp"   clamped at a soft limit
        """
        src = self.source.get()
        if src == "(none)":
            return None, "none"

        idx = SOURCE_INDEX[src]
        raw = human[idx]
        if raw is None or (isinstance(raw, float) and math.isnan(raw)):
            return None, "lost"

        if use_neutral and neutral is not None:
            ref = neutral[idx]
            if ref is None or math.isnan(ref):
                return None, "lost"
            raw = raw - ref

        gain = self.numeric(self.gain, 1.0)
        offset = self.numeric(self.offset, 0.0)
        sign = -1.0 if self.invert.get() else 1.0

        value = offset + sign * gain * raw

        lo_s, hi_s = SOFT_LIMITS[self.index]
        lo_w, hi_w = SAFE_LIMITS[self.index]

        if value < lo_s or value > hi_s:
            return max(lo_s, min(hi_s, value)), "clamp"
        if value < lo_w or value > hi_w:
            return value, "warn"
        return value, "ok"

    def show(self, value, state):
        colours = {"none": ("#888", ""),
                   "lost": ("#c33", "tracking lost"),
                   "ok": ("#1a7f37", ""),
                   "warn": ("#b26a00", "outside working"),
                   "clamp": ("#c33", "CLAMPED at limit")}
        colour, note = colours[state]
        self.out.config(
            text="---" if value is None else f"{value:8.2f}", fg=colour)
        self.status.config(text=note, fg=colour)

    # --- persistence ------------------------------------------------------
    def to_dict(self):
        return {"source": self.source.get(),
                "invert": self.invert.get(),
                "gain": self.numeric(self.gain, 1.0),
                "offset": self.numeric(self.offset, 0.0)}

    def from_dict(self, d):
        self.source.set(d.get("source", "(none)"))
        self.invert.set(bool(d.get("invert", False)))
        self.gain.set(f"{float(d.get('gain', 1.0)):.2f}")
        self.offset.set(f"{float(d.get('offset', 0.0)):.1f}")


class App:

    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.neutral = None
        self.use_neutral = tk.BooleanVar(value=False)
        self.last_count = -1

        root.title("FR5 joint mapper  -  Phase 2")

        # ---- banner
        banner = tk.Label(
            root,
            text="  NOT CONNECTED TO THE ROBOT  -  this panel only computes and "
                 "displays target angles  ",
            bg="#1f5fa9", fg="white", font=("TkDefaultFont", 10, "bold"), pady=6)
        banner.pack(fill="x")

        # ---- human angles
        hframe = ttk.LabelFrame(root, text=" live human angles (degrees) ")
        hframe.pack(fill="x", padx=10, pady=(10, 4))

        self.human_labels = {}
        for i, name in enumerate(["shoulder_elev", "shoulder_azim",
                                  "elbow_flex", "wrist_roll"]):
            tk.Label(hframe, text=name, width=14, anchor="w").grid(
                row=0, column=i * 2, padx=(8, 2), pady=6)
            lab = tk.Label(hframe, text="---", width=9, anchor="e",
                           font=("TkFixedFont", 11))
            lab.grid(row=0, column=i * 2 + 1, padx=(0, 10))
            self.human_labels[name] = lab

        self.track_label = tk.Label(hframe, text="waiting for tracker...",
                                    font=("TkDefaultFont", 9, "bold"))
        self.track_label.grid(row=1, column=0, columnspan=8, pady=(0, 6))

        # ---- neutral reference
        nframe = ttk.Frame(root)
        nframe.pack(fill="x", padx=10, pady=2)
        ttk.Button(nframe, text="Capture neutral pose",
                   command=self.capture_neutral).pack(side="left")
        ttk.Checkbutton(nframe, text="map relative to neutral",
                        variable=self.use_neutral,
                        command=self.refresh_now).pack(side="left", padx=10)
        self.neutral_label = tk.Label(nframe, text="no neutral captured",
                                      fg="#777", font=("TkDefaultFont", 9))
        self.neutral_label.pack(side="left", padx=6)

        # ---- mapping table
        mframe = ttk.LabelFrame(root, text=" mapping ")
        mframe.pack(fill="both", expand=True, padx=10, pady=6)

        heads = ["joint", "human source", "invert", "gain", "offset",
                 "target", "", "limits (deg)"]
        for c, h in enumerate(heads):
            tk.Label(mframe, text=h, font=("TkDefaultFont", 9, "bold")).grid(
                row=0, column=c, padx=2, pady=(6, 2))

        self.rows = [JointRow(mframe, i, i + 1, self.refresh_now)
                     for i in range(6)]

        # ---- buttons
        bframe = ttk.Frame(root)
        bframe.pack(fill="x", padx=10, pady=(2, 10))
        ttk.Button(bframe, text="Save mapping",
                   command=self.save).pack(side="left")
        ttk.Button(bframe, text="Load mapping",
                   command=self.load).pack(side="left", padx=6)
        ttk.Button(bframe, text="Reset to home offsets",
                   command=self.reset).pack(side="left")
        self.note = tk.Label(bframe, text="", fg="#555",
                             font=("TkDefaultFont", 9))
        self.note.pack(side="right")

        tk.Label(root,
                 text="target = offset + (gain x human angle), clamped to the "
                      "FR5 soft limits.  Publishing on /fr5_joint_targets.",
                 fg="#666", font=("TkDefaultFont", 9)).pack(pady=(0, 8))

        self.tick()

    # --- actions ----------------------------------------------------------
    def capture_neutral(self):
        human, tracking, _ = self.node.snapshot()
        if not tracking or any(math.isnan(v) for v in human):
            messagebox.showwarning(
                "Cannot capture",
                "The tracker is not reporting a complete arm right now.\n\n"
                "Stand so both shoulders and both hips are visible, then try "
                "again.")
            return
        self.neutral = list(human)
        self.neutral_label.config(
            text="neutral: " + "  ".join(f"{v:.0f}" for v in self.neutral),
            fg="#1a7f37")
        self.use_neutral.set(True)
        self.refresh_now()

    def reset(self):
        for i, row in enumerate(self.rows):
            row.offset.set(f"{HOME[i]:.1f}")
            row.gain.set("1.00")
        self.refresh_now()

    def save(self):
        data = {"rows": [r.to_dict() for r in self.rows],
                "use_neutral": self.use_neutral.get(),
                "neutral": self.neutral}
        try:
            with open(CONFIG_PATH, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2)
            self.note.config(text=f"saved to {CONFIG_PATH}")
        except OSError as exc:
            messagebox.showerror("Save failed", str(exc))

    def load(self):
        try:
            with open(CONFIG_PATH, encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Load failed", str(exc))
            return
        for row, d in zip(self.rows, data.get("rows", [])):
            row.from_dict(d)
        self.neutral = data.get("neutral")
        self.use_neutral.set(bool(data.get("use_neutral", False)))
        if self.neutral:
            self.neutral_label.config(
                text="neutral: " + "  ".join(f"{v:.0f}" for v in self.neutral),
                fg="#1a7f37")
        self.note.config(text=f"loaded from {CONFIG_PATH}")
        self.refresh_now()

    def refresh_now(self):
        self.last_count = -1      # force a recompute on the next tick

    # --- main loop --------------------------------------------------------
    def tick(self):
        human, tracking, count = self.node.snapshot()

        if count == 0:
            self.track_label.config(
                text="waiting for /human_arm_angles  -  is arm_tracker.py "
                     "running?", fg="#b26a00")
        elif tracking:
            self.track_label.config(text="TRACKING", fg="#1a7f37")
        else:
            self.track_label.config(text="ARM NOT TRACKED", fg="#c33")

        for name, idx in SOURCE_INDEX.items():
            v = human[idx]
            self.human_labels[name].config(
                text="---" if math.isnan(v) else f"{v:7.1f}",
                fg="#888" if math.isnan(v) else "#111")

        targets = []
        mapped_any = False
        all_valid = tracking
        for row in self.rows:
            value, state = row.compute(human, self.neutral,
                                       self.use_neutral.get())
            row.show(value, state)

            if state != "none":
                mapped_any = True

            if value is None:
                targets.append(float("nan"))
                # A joint with no source assigned is not a failure -- it simply
                # holds position downstream. Only a joint that HAS a source but
                # could not be computed (tracking lost) invalidates the output.
                if state != "none":
                    all_valid = False
            else:
                # A clamped value is still a usable command: clamping IS the
                # protection, so it does not invalidate. The panel shows it red.
                targets.append(value)

        if not mapped_any:
            all_valid = False

        self.node.publish_targets(targets, all_valid)
        self.last_count = count

        self.root.after(50, self.tick)


# -------------------------------------------------------------------- main
def main():
    rclpy.init()
    node = MapperNode()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    App(root, node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
