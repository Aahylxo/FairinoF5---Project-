#!/usr/bin/env python3
"""
teleop_node.py  —  Phase 3 of FR5 teleoperation.

Takes the mapped joint targets from joint_mapper.py and drives the robot,
through a safety chain that stands between your arm and 22 kg of moving metal.

    /fr5_joint_targets          (raw mapped targets, from joint_mapper.py)
        |
        +-- validity check      NaN / tracking lost / stale -> FREEZE
        +-- clamp               to the FR5 SOFT limits
        +-- low-pass filter     absorbs pose-estimation jitter
        +-- slew-rate limit     caps degrees per second, whatever you do
        +-- deadman             motion only while the button is held
        |
    /fairino5_controller/joint_trajectory

Nothing is published unless the deadman button is held down. Releasing it,
losing tracking, or losing the target stream all stop motion immediately.

ORDER OF TESTING -- do not skip:

    1. --dry-run, no robot at all. Watch the numbers.
    2. Simulated robot. Set the xacro back to mock_components (README 3.3),
       launch demo.launch.py, and drive the fake arm in RViz.
    3. Real robot, with a supervisor, hand on the physical e-stop.

Usage
-----
    python3 teleop_node.py --dry-run
    python3 teleop_node.py                      # publishes to the controller
    python3 teleop_node.py --max-speed 5        # even slower

Needs tkinter:   sudo apt install -y python3-tk
"""

import argparse
import math
import sys
import threading
import time

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    sys.exit("tkinter is not installed.  Run:\n"
             "  sudo apt install -y python3-tk")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]

# FR5 SOFT limits, degrees -- the robot's own, not the URDF's wrong j3 value.
SOFT_LIMITS = [
    (-175.0, 175.0),
    (-265.0,  85.0),
    (-150.0, 150.0),
    (-265.0,  85.0),
    (-175.0, 175.0),
    (-175.0, 175.0),
]

CONTROLLER_TOPIC = "/fairino5_controller/joint_trajectory"

# --- safety defaults ------------------------------------------------------
MAX_SPEED_DEG_S = 10.0    # per joint, regardless of how fast you move
FILTER_HZ = 2.0           # low-pass cutoff; lower = smoother but laggier
PUBLISH_HZ = 25.0
TARGET_TIMEOUT = 0.5      # seconds without a target before freezing
STATE_TIMEOUT = 1.0       # seconds without /joint_states before refusing
ENGAGE_TOLERANCE = 15.0   # degrees; max gap allowed when engaging


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class TeleopNode(Node):

    def __init__(self, args):
        super().__init__("fr5_teleop")
        self.args = args
        self.lock = threading.Lock()

        self.targets = [float("nan")] * 6
        self.targets_valid = False
        self.targets_stamp = 0.0

        self.actual = None            # degrees, from /joint_states
        self.actual_stamp = 0.0

        self.create_subscription(
            Float32MultiArray, "/fr5_joint_targets", self.on_targets, 10)
        self.create_subscription(
            JointState, "/joint_states", self.on_state, 10)

        self.pub = self.create_publisher(
            JointTrajectory, CONTROLLER_TOPIC, 10)

    # --- inputs -----------------------------------------------------------
    def on_targets(self, msg):
        if len(msg.data) < 7:
            return
        with self.lock:
            self.targets = list(msg.data[:6])
            self.targets_valid = msg.data[6] > 0.5
            self.targets_stamp = time.time()

    def on_state(self, msg):
        # /joint_states does NOT publish j1..j6 in order -- match by name.
        found = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                found[name] = math.degrees(pos)
        if len(found) == 6:
            with self.lock:
                self.actual = [found[n] for n in JOINTS]
                self.actual_stamp = time.time()

    def snapshot(self):
        with self.lock:
            return (list(self.targets), self.targets_valid, self.targets_stamp,
                    None if self.actual is None else list(self.actual),
                    self.actual_stamp)

    # --- output -----------------------------------------------------------
    def send(self, command_deg, dt):
        msg = JointTrajectory()
        msg.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [math.radians(a) for a in command_deg]
        # Give the controller slightly longer than one cycle to get there, so
        # consecutive goals blend instead of fighting each other.
        horizon = max(0.05, dt * 2.0)
        point.time_from_start = Duration(
            sec=int(horizon), nanosec=int((horizon % 1.0) * 1e9))
        msg.points = [point]
        self.pub.publish(msg)


class TeleopApp:

    def __init__(self, root, node, args):
        self.root = root
        self.node = node
        self.args = args

        self.enabled = False
        self.command = None            # the filtered, rate-limited command
        self.filtered = None           # filter state
        self.last_tick = time.time()
        self.reason = "idle"

        root.title("FR5 teleoperation  -  Phase 3")
        root.protocol("WM_DELETE_WINDOW", self.quit)

        mode = ("DRY RUN  -  nothing is published"
                if args.dry_run else
                f"LIVE  -  publishing to {CONTROLLER_TOPIC}")
        colour = "#1f5fa9" if args.dry_run else "#a93226"
        tk.Label(root, text=f"  {mode}  ", bg=colour, fg="white",
                 font=("TkDefaultFont", 11, "bold"), pady=7).pack(fill="x")

        # ---- status
        sframe = ttk.LabelFrame(root, text=" status ")
        sframe.pack(fill="x", padx=10, pady=(10, 4))
        self.status = tk.Label(sframe, text="idle",
                               font=("TkDefaultFont", 12, "bold"),
                               fg="#777", pady=6)
        self.status.pack()
        self.detail = tk.Label(sframe, text="", font=("TkDefaultFont", 9),
                               fg="#555")
        self.detail.pack(pady=(0, 6))

        # ---- joint table
        jframe = ttk.LabelFrame(root, text=" joints (degrees) ")
        jframe.pack(fill="both", expand=True, padx=10, pady=4)

        for c, head in enumerate(["", "target", "commanded", "actual", "error"]):
            tk.Label(jframe, text=head,
                     font=("TkDefaultFont", 9, "bold")).grid(
                         row=0, column=c, padx=8, pady=(6, 2))

        self.cells = []
        for i, name in enumerate(JOINTS):
            tk.Label(jframe, text=name, width=4,
                     font=("TkDefaultFont", 11, "bold")).grid(
                         row=i + 1, column=0, padx=8, pady=2)
            row = []
            for c in range(1, 5):
                lab = tk.Label(jframe, text="---", width=10, anchor="e",
                               font=("TkFixedFont", 11))
                lab.grid(row=i + 1, column=c, padx=8)
                row.append(lab)
            self.cells.append(row)

        # ---- deadman
        dframe = ttk.Frame(root)
        dframe.pack(fill="x", padx=10, pady=8)

        self.idle_text = ("CLICK TO START MOVING" if args.toggle
                          else "HOLD TO MOVE THE ROBOT")
        self.live_text = ("CLICK TO STOP" if args.toggle
                          else "MOVING  -  release to stop")

        self.deadman = tk.Button(
            dframe, text=self.idle_text,
            font=("TkDefaultFont", 14, "bold"),
            bg="#2d7a3e", fg="white", activebackground="#1f5a2c",
            activeforeground="white", relief="raised", bd=4, height=2)
        self.deadman.pack(fill="x")

        if args.toggle:
            # Toggle mode: one click on, one click off.
            self.deadman.config(command=self.toggle)

            stop = tk.Button(dframe, text="STOP   (Esc)",
                             font=("TkDefaultFont", 12, "bold"),
                             bg="#a93226", fg="white",
                             activebackground="#7b241c",
                             activeforeground="white", bd=3,
                             command=self.release)
            stop.pack(fill="x", pady=(6, 0))
        else:
            # Hold mode: press and keep holding.
            #
            # Deliberately NOT bound to <Leave>: nudging the pointer off the
            # button while watching RViz would silently disengage.
            self.deadman.bind("<ButtonPress-1>", self.press)
            self.deadman.bind("<ButtonRelease-1>", self.release)
            root.bind("<FocusOut>", self.release)

        # Escape always stops, in either mode.
        root.bind("<Escape>", self.release)
        root.bind("<space>", self.release)

        mode_note = ("TOGGLE mode - click on, click off, Esc stops"
                     if args.toggle else
                     "HOLD mode - motion only while the button is held")
        tk.Label(root,
                 text=f"{mode_note}   |   max {args.max_speed:.0f} deg/s "
                      f"per joint   |   the physical e-stop is still the "
                      f"real stop",
                 fg="#666", font=("TkDefaultFont", 9)).pack(pady=(0, 8))

        self.tick()

    # --- deadman ----------------------------------------------------------
    def toggle(self):
        """Toggle mode: one click engages, the next disengages."""
        if self.enabled:
            self.release()
        else:
            self.press()

    def press(self, _event=None):
        targets, valid, t_stamp, actual, a_stamp = self.node.snapshot()
        now = time.time()

        if not valid or now - t_stamp > TARGET_TIMEOUT:
            self.reason = "refused: no valid targets"
            return
        if actual is None or now - a_stamp > STATE_TIMEOUT:
            self.reason = "refused: no /joint_states - is the robot running?"
            return

        # Engage gate: refuse if a mapped target is far from where the arm
        # actually is. Without this, enabling would command a large jump.
        #
        # Joints with a NaN target are unmapped and simply hold position, so
        # they are skipped -- comparing NaN to a number yields nonsense.
        gaps = [(i, abs(t - a))
                for i, (t, a) in enumerate(zip(targets, actual))
                if t is not None and not math.isnan(t)]

        if not gaps:
            self.reason = "refused: no joints are mapped"
            return

        bad = [f"{JOINTS[i]} off by {g:.0f}"
               for i, g in gaps if g > self.args.tolerance]
        if bad:
            self.reason = "refused: " + ", ".join(bad)
            return

        # Start the command from where the arm IS, not from the target.
        self.command = list(actual)
        self.filtered = list(actual)
        self.enabled = True
        self.reason = "engaged"
        self.deadman.config(bg="#a93226", text=self.live_text)

    def release(self, _event=None):
        if self.enabled:
            self.reason = "stopped: deadman released"
        self.enabled = False
        self.deadman.config(bg="#2d7a3e", text=self.idle_text)

    def quit(self):
        self.release()
        self.root.destroy()

    # --- main loop --------------------------------------------------------
    def tick(self):
        now = time.time()
        dt = max(1e-3, now - self.last_tick)
        self.last_tick = now

        targets, valid, t_stamp, actual, a_stamp = self.node.snapshot()

        target_fresh = (now - t_stamp) < TARGET_TIMEOUT
        state_fresh = actual is not None and (now - a_stamp) < STATE_TIMEOUT

        # ---- reasons to drop out of enabled, checked every cycle
        if self.enabled:
            if not valid or not target_fresh:
                self.enabled = False
                self.reason = "FROZE: tracking lost"
                self.deadman.config(bg="#2d7a3e", text=self.idle_text)
            elif not state_fresh:
                self.enabled = False
                self.reason = "FROZE: lost /joint_states"
                self.deadman.config(bg="#2d7a3e", text=self.idle_text)

        # ---- the safety chain
        if self.enabled and self.command is not None:
            alpha = 1.0 - math.exp(-2.0 * math.pi * self.args.filter_hz * dt)
            max_step = self.args.max_speed * dt

            for i in range(6):
                raw = targets[i]
                if raw is None or math.isnan(raw):
                    continue                      # hold this joint
                lo, hi = SOFT_LIMITS[i]
                desired = clamp(raw, lo, hi)

                # low-pass: absorb jitter
                self.filtered[i] += alpha * (desired - self.filtered[i])

                # slew-rate limit: the hard cap on how fast the robot may move
                delta = self.filtered[i] - self.command[i]
                delta = clamp(delta, -max_step, max_step)
                self.command[i] = clamp(self.command[i] + delta, lo, hi)

            if not self.args.dry_run:
                self.node.send(self.command, dt)

        # ---- display
        for i in range(6):
            t = targets[i]
            c = self.command[i] if self.command is not None else None
            a = actual[i] if actual is not None else None

            self.cells[i][0].config(
                text="---" if t is None or math.isnan(t) else f"{t:8.2f}",
                fg="#c33" if (t is None or math.isnan(t)) else "#111")
            self.cells[i][1].config(
                text="---" if c is None else f"{c:8.2f}")
            self.cells[i][2].config(
                text="---" if a is None else f"{a:8.2f}")
            if c is not None and a is not None:
                err = c - a
                self.cells[i][3].config(
                    text=f"{err:8.2f}",
                    fg="#c33" if abs(err) > 5 else "#555")
            else:
                self.cells[i][3].config(text="---", fg="#555")

        if self.enabled:
            self.status.config(text="MOVING", fg="#a93226")
        elif not target_fresh:
            self.status.config(text="no targets  -  is joint_mapper running?",
                               fg="#b26a00")
        elif not valid:
            self.status.config(text="targets invalid  -  tracking lost",
                               fg="#b26a00")
        elif not state_fresh:
            self.status.config(text="no /joint_states  -  is the robot up?",
                               fg="#b26a00")
        else:
            self.status.config(text="ready  -  hold the button", fg="#1a7f37")

        self.detail.config(text=self.reason)

        self.root.after(int(1000.0 / PUBLISH_HZ), self.tick)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true",
                    help="compute everything but publish nothing")
    ap.add_argument("--max-speed", type=float, default=MAX_SPEED_DEG_S,
                    help=f"max degrees per second per joint "
                         f"(default {MAX_SPEED_DEG_S:.0f})")
    ap.add_argument("--filter-hz", type=float, default=FILTER_HZ,
                    help=f"low-pass cutoff, lower is smoother "
                         f"(default {FILTER_HZ:.1f})")
    ap.add_argument("--toggle", action="store_true",
                    help="click to start, click to stop, instead of "
                         "hold-to-move. Convenient, but a held deadman is "
                         "safer on real hardware")
    ap.add_argument("--tolerance", type=float, default=ENGAGE_TOLERANCE,
                    help=f"max degrees between target and actual when "
                         f"engaging (default {ENGAGE_TOLERANCE:.0f})")
    args = ap.parse_args()

    if args.max_speed <= 0 or args.max_speed > 45:
        sys.exit("--max-speed must be between 0 and 45 deg/s. "
                 "Start at 10 and only raise it once you trust the mapping.")

    rclpy.init()
    node = TeleopNode(args)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    root = tk.Tk()
    TeleopApp(root, node, args)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
