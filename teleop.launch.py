#!/usr/bin/env python3
"""
teleop.launch.py  —  start the camera and the hand teleoperation in one go.

    ros2 launch ~/fairino_380_ws/teleop.launch.py

That replaces three separate terminals (camera, tracker, control) with one.
You still need a second terminal for the robot itself:

    ros2 launch fairino5_v6_moveit2_config demo.launch.py

Arguments
---------
    dry_run:=true        compute everything, publish nothing
    hold:=true           hold-to-move instead of click-to-toggle
    depth:=false         colour only: no depth stream (if frames drop over WSL);
                         distance is then estimated from hand size, ~9x noisier
    scale:=1.8           hand motion amplification
    max_speed:=5.0       degrees per second per joint
    filter_hz:=1.0       low-pass cutoff, lower is smoother
    width / height / fps colour stream settings

Example:

    ros2 launch ~/fairino_380_ws/teleop.launch.py dry_run:=true scale:=2.0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WS = os.path.expanduser("~/fairino_380_ws")

ARGUMENTS = [
    ("mode", "torso", "torso | cylindrical | cartesian"),
    ("source", "ros", "ros = RealSense node; webcam = plain camera, no RealSense"),
    ("device", "0", "camera index when source:=webcam"),
    ("dry_run", "false", "compute everything, publish nothing"),
    ("hold", "false", "hold-to-move instead of click-to-toggle"),
    ("depth", "true", "true = aligned depth for distance; false = colour only"),
    ("scale", "1.0", "hand motion amplification"),
    ("max_speed", "5.0", "degrees per second per joint"),
    ("filter_hz", "2.0", "low-pass cutoff, lower is smoother"),
    ("max_jump", "30.0", "reject IK solutions needing more than this"),
    ("hand_smooth_hz", "2.5", "smoothing on the measured hand position"),
    ("deadband", "12.0", "ignore target movement smaller than this, mm"),
    ("width", "640", "colour stream width"),
    ("height", "360", "colour stream height"),
    ("fps", "15", "colour stream rate"),
]


def is_true(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def launch_setup(context, *_args, **_kwargs):
    """Resolve the launch arguments and build the processes from real values."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    colour_profile = f"{arg('width')}x{arg('height')}x{arg('fps')}"

    # Depth is ON by default and aligned to the colour image: hand_teleop.py
    # reads /camera/camera/aligned_depth_to_color/image_raw for the hand's
    # distance (~2 mm jitter, vs ~18.5 mm estimating it from hand size -- see
    # WHY.md#why-a-depth-camera). Without the depth stream it silently falls
    # back to the hand-size estimate. The infrared streams stay off. If the
    # virtual USB link into WSL drops frames, use depth:=false.
    use_depth = is_true(arg("depth"))
    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        output="screen",
        parameters=[{
            "rgb_camera.color_profile": colour_profile,
            "enable_depth": use_depth,
            "align_depth.enable": use_depth,
            # A small depth stream: the D455's default 848x480x30 is more than
            # the virtual USB link into WSL carries (REFERENCE.md 4.5). The
            # parameter is spelled two ways across realsense-ros 4.x releases;
            # the one a release does not know is ignored.
            "depth_module.depth_profile": f"480x270x{arg('fps')}",
            "depth_module.profile": f"480x270x{arg('fps')}",
            "enable_infra1": False,
            "enable_infra2": False,
        }],
    )

    cmd = [
        "python3", os.path.join(WS, "hand_teleop.py"),
        "--mode", arg("mode"),
        "--source", arg("source"),
        "--device", arg("device"),
        "--scale", arg("scale"),
        "--max-speed", arg("max_speed"),
        "--filter-hz", arg("filter_hz"),
        "--max-jump", arg("max_jump"),
        "--hand-smooth-hz", arg("hand_smooth_hz"),
        "--deadband", arg("deadband"),
    ]
    if is_true(arg("dry_run")):
        cmd.append("--dry-run")
    if is_true(arg("hold")):
        cmd.append("--hold")

    # A plain process rather than a Node: it owns a Tk window and an OpenCV
    # window, and takes its settings on the command line.
    teleop = ExecuteProcess(cmd=cmd, output="screen")

    mode = "DRY RUN (publishing nothing)" if is_true(arg("dry_run")) else "LIVE"

    processes = [camera, teleop] if arg("source") == "ros" else [teleop]

    return [
        LogInfo(msg=(
            "\n"
            "  FR5 hand teleoperation\n"
            "  ----------------------\n"
            f"  mode     : {mode}\n"
            f"  camera   : {'RealSense ' + colour_profile + (', aligned depth on' if use_depth else ', depth off (hand-size estimate)') if arg('source') == 'ros' else 'webcam index ' + arg('device')}\n"
            f"  scale    : {arg('scale')} x\n"
            f"  max speed: {arg('max_speed')} deg/s per joint\n"
            "\n"
            "  The robot must be launched separately, in its own terminal:\n"
            "      ros2 launch fairino5_v6_moveit2_config demo.launch.py\n"
        )),
    ] + processes


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=default,
                                      description=desc)
                for name, default, desc in ARGUMENTS]
    return LaunchDescription(declared + [OpaqueFunction(function=launch_setup)])
