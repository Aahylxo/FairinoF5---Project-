#!/usr/bin/env bash
# versions.sh -- record every version this project depends on.
#
#   cd ~/fairino_380_ws
#   bash versions.sh | tee VERSIONS.txt
#
# Then paste VERSIONS.txt into README Part 1.0, or just commit the file.
#
# Run it on the machine the demonstration will happen on. Versions are the
# first thing to check when something works on one laptop and not another.

set -uo pipefail

line() { printf '%s\n' "------------------------------------------------------------"; }
item() { printf '  %-34s %s\n' "$1" "${2:-NOT FOUND}"; }

# Run a command, print its output on one line, or NOT FOUND if it fails.
try() { local out; out=$("$@" 2>/dev/null | head -n1) && [ -n "$out" ] && printf '%s' "$out"; }

echo
echo "FAIRINO FR5 — recorded versions"
echo "generated: $(date -u '+%Y-%m-%d %H:%M UTC')"
echo "hostname : $(hostname)"
line

echo "OPERATING SYSTEM"
item "Windows build"   "$(cmd.exe /c ver 2>/dev/null | tr -d '\000\r\n' | tr -s ' ' | sed 's/^ *//')"
item "WSL"             "$(try wsl.exe --version | tr -d '\r\0')"
item "Ubuntu"          "$(try lsb_release -ds)"
item "kernel"          "$(uname -r)"
item "architecture"    "$(uname -m)"
line

echo "ROS 2"
item "ROS_DISTRO"      "${ROS_DISTRO:-}"
item "ros-humble-ros2cli" "$(dpkg-query -W -f='${Version}' ros-humble-ros2cli 2>/dev/null)"
item "RMW"             "${RMW_IMPLEMENTATION:-<unset>}$([ "${RMW_IMPLEMENTATION:-}" = rmw_fastrtps_cpp ] || echo '   <-- expected rmw_fastrtps_cpp, see README Part 8')"
item "ROS_LOCALHOST_ONLY" "${ROS_LOCALHOST_ONLY:-<unset>}$([ "${ROS_LOCALHOST_ONLY:-}" = 1 ] || echo '   <-- expected 1, see README Part 8')"
item "CYCLONEDDS_URI"  "${CYCLONEDDS_URI:-<unset -- correct>}"
for p in ros-humble-desktop ros-humble-moveit ros-humble-ros2-control \
         ros-humble-ros2-controllers ros-humble-realsense2-camera \
         ros-humble-cv-bridge ros-humble-rviz2 ros-humble-xacro \
         ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui; do
    item "$p" "$(dpkg-query -W -f='${Version}' "$p" 2>/dev/null)"
done
line

echo "BUILD TOOLS"
item "gcc"             "$(try gcc -dumpfullversion)"
item "cmake"           "$(try cmake --version | awk '{print $3}')"
item "colcon"          "$(python3 -c 'import colcon_core;print(colcon_core.__version__)' 2>/dev/null)"
item "git"             "$(try git --version | awk '{print $3}')"
line

echo "PYTHON"
item "python3"         "$(try python3 --version | awk '{print $2}')"
item "pip3"            "$(try pip3 --version | awk '{print $2}')"
for m in numpy mediapipe cv2 scipy; do
    v=$(python3 - "$m" <<'EOF' 2>/dev/null
import importlib, sys
try:
    print(getattr(importlib.import_module(sys.argv[1]), "__version__", "?"))
except Exception:
    pass
EOF
)
    item "python: $m" "$v"
done
item "python3-tk"      "$(python3 -c 'import tkinter; print(tkinter.TkVersion)' 2>/dev/null)"
line

echo "FAIRINO DRIVER"
SRC="$HOME/fairino_380_ws/src/frcobot_ros2"
if [ -d "$SRC/.git" ]; then
    item "git tag"     "$(git -C "$SRC" describe --tags --always 2>/dev/null)"
    item "git commit"  "$(git -C "$SRC" rev-parse --short HEAD 2>/dev/null)"
    item "git remote"  "$(git -C "$SRC" config --get remote.origin.url 2>/dev/null)"
    item "uncommitted vendor changes" "$(git -C "$SRC" status --porcelain 2>/dev/null | wc -l) file(s)"
elif [ -d "$SRC" ]; then
    item "git tag"     "NO .git FOLDER -- version cannot be proven"
    echo "  !! The driver source is present but its git history is not, so this"
    echo "  !! script cannot record which version you built. The project is"
    echo "  !! pinned to tag V3.0.0_RobotV3.8.0 (commit 7c683cf). See the note"
    echo "  !! under FAIRINO DRIVER in README Part 1.0 for how to restore it."
else
    item "vendor driver" "NOT FOUND at $SRC"
fi
PLUGIN=$(grep -oE '(mock_components/GenericSystem|fairino_hardware/FairinoHardwareInterface)' \
        "$SRC/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro" 2>/dev/null | head -n1)
case "$PLUGIN" in
    mock_components/GenericSystem)             MODE="SIMULATION" ;;
    fairino_hardware/FairinoHardwareInterface) MODE="REAL ROBOT -- will fail with no robot attached" ;;
    *)                                         MODE="could not read" ;;
esac
item "hardware plugin in use" "$PLUGIN"
item "  -> mode"              "$MODE"
item "controller update_rate" \
     "$(grep -oE 'update_rate:[[:space:]]*[0-9]+' \
        "$SRC/fairino5_v6_moveit2_config/config/ros2_controllers.yaml" 2>/dev/null | head -n1 | grep -oE '[0-9]+') Hz"
line

echo "THIS REPOSITORY"
WS="$HOME/fairino_380_ws"
if [ -d "$WS/.git" ]; then
    item "git commit"  "$(git -C "$WS" rev-parse --short HEAD 2>/dev/null)"
    item "git branch"  "$(git -C "$WS" rev-parse --abbrev-ref HEAD 2>/dev/null)"
    item "last commit" "$(git -C "$WS" log -1 --format='%ad  %s' --date=short 2>/dev/null)"
    item "uncommitted" "$(git -C "$WS" status --porcelain 2>/dev/null | wc -l) file(s)"
fi
line

echo "CAMERA"
for lr in librealsense2 librealsense2-utils ros-humble-librealsense2; do
    v=$(dpkg-query -W -f='${Version}' "$lr" 2>/dev/null) && [ -n "$v" ] && item "$lr" "$v"
done
item "USB device"      "$(lsusb 2>/dev/null | grep -i intel | head -n1)"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    item "firmware"    "$(rs-enumerate-devices -s 2>/dev/null | awk 'NR==3{print $NF}')"
fi
line

echo "ROBOT CONTROLLER (needs the Ethernet link up)"
if ping -c1 -W1 192.168.58.2 >/dev/null 2>&1; then
    item "192.168.58.2" "reachable"
    item "port 8080"    "$(timeout 2 bash -c 'echo > /dev/tcp/192.168.58.2/8080' 2>/dev/null && echo open || echo closed)"
    item "port 8081"    "$(timeout 2 bash -c 'echo > /dev/tcp/192.168.58.2/8081' 2>/dev/null && echo open || echo closed)"
    echo
    echo "  Controller and servo firmware are on the teach pendant, under"
    echo "  System -> About / Version. Record them by hand -- there is no"
    echo "  command that reads them over the network."
else
    item "192.168.58.2" "not reachable (robot off, or no cable -- fine if you are in simulation)"
fi
line

echo "PROJECT FILES"
cd "$WS" 2>/dev/null && for f in hand_teleop.py teleop.launch.py fr5_kinematics.py \
        arm_tracker.py joint_mapper.py teleop_node.py fairino_home.py \
        requirements.txt README.md FOLLOW_ALONG.md; do
    if [ -f "$f" ]; then
        item "$f" "$(wc -l < "$f" | tr -d ' ') lines   md5 $(md5sum "$f" | cut -c1-8)"
    else
        item "$f" "MISSING"
    fi
done
line
echo
