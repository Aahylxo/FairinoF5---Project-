#!/usr/bin/env python3
"""
rviz_real.launch.py  —  the FR5 in RViz with NO MoveIt.

Starts only what is needed to drive the arm through ros2_control:

    robot_state_publisher      URDF + /joint_states  ->  /tf
    ros2_control_node          loads the hardware plugin (mock OR real,
                               whichever the ros2_control xacro says)
    joint_state_broadcaster    publishes /joint_states
    fairino5_controller        JointTrajectoryController, takes commands
                               (open_loop_control on: slider_controllers.yaml,
                               which must sit next to this file)
    rviz2                      plain RViz, no MotionPlanning panel

No move_group is started. MoveItConfigsBuilder is used ONLY to read the
robot description out of the config package the same way demo.launch.py
does -- it is a config loader, not the planner.

    ros2 launch ~/fairino_380_ws/rviz_real.launch.py

Then, in a second terminal, the sliders:

    python3 ~/fairino_380_ws/slider_bridge.py
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    cfg = MoveItConfigsBuilder(
        "fairino5_v6_robot",
        package_name="fairino5_v6_moveit2_config").to_moveit_configs()

    # Our own copy of the controller config, with open_loop_control on --
    # see slider_controllers.yaml. MoveIt's demo.launch.py is not affected.
    controllers_yaml = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "slider_controllers.yaml")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[cfg.robot_description],
    )

    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[cfg.robot_description, controllers_yaml],
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fairino5_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",      # loads ~/.rviz2/default.rviz -- File > Save Config
    )

    return LaunchDescription([rsp, control, spawn_jsb,
                              spawn_arm, rviz])
