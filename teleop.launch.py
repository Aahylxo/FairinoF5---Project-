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
    ("dry_run", "false", "compute everything, publish nothing"),
    ("hold", "false", "hold-to-move instead of click-to-toggle"),
    ("scale", "1.4", "hand motion amplification"),
    ("max_speed", "10.0", "degrees per second per joint"),
    ("filter_hz", "2.0", "low-pass cutoff, lower is smoother"),
    ("max_jump", "30.0", "reject IK solutions needing more than this"),
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

    # Depth and the infrared streams are off: the tracker only uses colour, and
    # the virtual USB link into WSL cannot carry the full set without dropping
    # frames. See README Part 4.5.
    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        output="screen",
        parameters=[{
            "rgb_camera.color_profile": colour_profile,
            "enable_depth": False,
            "enable_infra1": False,
            "enable_infra2": False,
        }],
    )

    cmd = [
        "python3", os.path.join(WS, "hand_teleop.py"),
        "--scale", arg("scale"),
        "--max-speed", arg("max_speed"),
        "--filter-hz", arg("filter_hz"),
        "--max-jump", arg("max_jump"),
    ]
    if is_true(arg("dry_run")):
        cmd.append("--dry-run")
    if is_true(arg("hold")):
        cmd.append("--hold")

    # A plain process rather than a Node: it owns a Tk window and an OpenCV
    # window, and takes its settings on the command line.
    teleop = ExecuteProcess(cmd=cmd, output="screen")

    mode = "DRY RUN (publishing nothing)" if is_true(arg("dry_run")) else "LIVE"

    return [
        LogInfo(msg=(
            "\n"
            "  FR5 hand teleoperation\n"
            "  ----------------------\n"
            f"  mode     : {mode}\n"
            f"  camera   : {colour_profile}, depth off\n"
            f"  scale    : {arg('scale')} x\n"
            f"  max speed: {arg('max_speed')} deg/s per joint\n"
            "\n"
            "  The robot must be launched separately, in its own terminal:\n"
            "      ros2 launch fairino5_v6_moveit2_config demo.launch.py\n"
        )),
        camera,
        teleop,
    ]


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=default,
                                      description=desc)
                for name, default, desc in ARGUMENTS]
    return LaunchDescription(declared + [OpaqueFunction(function=launch_setup)])
