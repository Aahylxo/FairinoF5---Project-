#!/usr/bin/env bash
# patch_driver.sh -- fix the FAIRINO ros2_control driver's ServoJ handling.
#
#   bash patch_driver.sh            # check and show what it would do
#   bash patch_driver.sh --apply    # make the changes
#   bash patch_driver.sh --revert   # put the original back
#
# WHAT IS WRONG
# -------------
# The driver streams ServoJ commands without ever calling ServoMoveStart().
# The SDK's own header describes ServoMoveStart/ServoMoveEnd as "Start/End
# servo motion, used with ServoJ and ServoCart commands" -- so the bracket is
# required and the vendor's hardware interface omits it.
#
# Without it the controller appears to treat each ServoJ as a separate buffered
# motion. A PickNik fork measured 2.1 s of command-to-motion latency on an FR5;
# with the bracket it dropped to ~0.2 s. The commanded position runs ahead of
# the arm, and when the arm catches up the controller sees a joint speed it
# will not accept -- it faults, and then refuses every command after that.
#
# That matches what this robot does: a move runs for about three seconds, then
# the pendant shows "command speed in the joint space of Axis 4 exceeds the
# limit" and the log fills with ServoJ error 14 forever after.
#
# WHAT THIS CHANGES
# -----------------
#   1. on_activate    calls ServoMoveStart() once the arm is connected
#   2. on_deactivate  calls ServoMoveEnd() before stopping
#   3. write()        logs a refusal ONCE at ERROR instead of 250 times a
#                     second at INFO, and says what to do about it
#
# It does NOT change what is commanded, how fast, or any limit. Nothing here
# can make the arm move further or faster than before.

set -uo pipefail

WS="$HOME/fairino_380_ws"
SRC="$WS/src/frcobot_ros2/fairino_hardware/src/fairino_hardware_interface.cpp"
BAK="$SRC.orig"
MODE="${1:---check}"

red()  { printf '\033[31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[32m%s\033[0m\n' "$*"; }
ylw()  { printf '\033[33m%s\033[0m\n' "$*"; }

[ -f "$SRC" ] || { red "Not found: $SRC"; exit 1; }

# ---------------------------------------------------------------- revert ----
if [ "$MODE" = "--revert" ]; then
    if [ -f "$BAK" ]; then
        cp "$BAK" "$SRC"
        grn "Original restored. Rebuild with:"
        echo "    cd $WS && colcon build --packages-select fairino_hardware --symlink-install"
    else
        red "No backup at $BAK -- nothing to revert to."
    fi
    exit 0
fi

# ------------------------------------------------- does the SDK have it? ----
echo
echo "1. Checking the SDK actually provides ServoMoveStart / ServoMoveEnd"
HDR=$(find "$WS/src/frcobot_ros2" -name "robot.h" 2>/dev/null | head -n1)
if [ -z "$HDR" ]; then
    ylw "   robot.h not found -- cannot confirm the function exists."
    ylw "   If it does not, the build will fail. That is safe: nothing"
    ylw "   reaches the robot and you can --revert."
else
    echo "   header: $HDR"
    if grep -q "ServoMoveStart" "$HDR"; then
        grn "   found:  $(grep -n 'ServoMoveStart' "$HDR" | head -n1 | cut -c1-110)"
        grn "   found:  $(grep -n 'ServoMoveEnd'   "$HDR" | head -n1 | cut -c1-110)"
    else
        red "   ServoMoveStart is NOT in this SDK header."
        red "   Do not apply this patch -- it will not compile."
        exit 1
    fi
fi

# ----------------------------------------------------------- already on? ----
if grep -q "ServoMoveStart" "$SRC"; then
    ylw "
2. This file already calls ServoMoveStart -- patch appears to be applied.
   Use --revert to go back to the original."
    exit 0
fi

echo
echo "2. The three changes:"
echo "   a) on_activate   -> ServoMoveStart() after the arm connects"
echo "   b) on_deactivate -> ServoMoveEnd() before StopMotion()"
echo "   c) write()       -> log a refusal once, at ERROR, with the fix"

if [ "$MODE" != "--apply" ]; then
    echo
    ylw "Nothing changed. Run with --apply to make the changes."
    exit 0
fi

# ------------------------------------------------------------- apply it ----
[ -f "$BAK" ] || cp "$SRC" "$BAK"
echo
echo "3. Backup: $BAK"

python3 - "$SRC" <<'PY'
import sys
p = sys.argv[1]
s = open(p, encoding="utf-8", errors="surrogateescape").read()

def once(old, new, label):
    global s
    n = s.count(old)
    if n != 1:
        print(f"   FAILED: {label} -- found {n} matches, expected 1")
        sys.exit(1)
    s = s.replace(old, new)
    print(f"   ok: {label}")

# (a) start the servo stream once the arm is connected and the initial
#     command position has been synced to the real one.
once(
'''        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂硬件启动成功!");
        return hardware_interface::CallbackReturn::SUCCESS;''',
'''        // --- PATCH: announce the servo stream -------------------------------
        // The SDK requires ServoMoveStart() before ServoJ. Without it the
        // controller buffers each ServoJ as a separate motion, the command
        // runs ahead of the arm, and the catch-up trips a joint speed limit.
        int servo_start = _ptr_robot->ServoMoveStart();
        if(servo_start != 0){
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                "ServoMoveStart failed, code %d. The robot may be in a fault "
                "state -- open http://192.168.58.2 and clear the alarm.",
                servo_start);
        }else{
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "ServoMoveStart ok -- servo stream open.");
        }
        // --- end patch ------------------------------------------------------
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂硬件启动成功!");
        return hardware_interface::CallbackReturn::SUCCESS;''',
"on_activate -> ServoMoveStart()")

# (b) close the stream on the way out.
once(
'''    _ptr_robot->StopMotion();//停止机器人''',
'''    _ptr_robot->ServoMoveEnd();// PATCH: close the servo stream
    _ptr_robot->StopMotion();//停止机器人''',
"on_deactivate -> ServoMoveEnd()")

# (c) a refusal is an ERROR that happens once, not an INFO 250 times a second.
once(
'''        int returncode = _ptr_robot->ServoJ(&cmd,&extcmd,0,0,0.008,0,0);
        if(returncode != 0){
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ指令下发错误,错误码:%d",returncode);
        }''',
'''        int returncode = _ptr_robot->ServoJ(&cmd,&extcmd,0,0,0.008,0,0);
        // --- PATCH: say it once, loudly, with the remedy --------------------
        // The original logged at INFO every control cycle -- 250 lines a
        // second, which buries the one line that matters.
        static int  last_servoj_code = 0;
        static long servoj_refusals  = 0;
        if(returncode != 0){
            if(returncode != last_servoj_code){
                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                    "ServoJ REFUSED by the controller, error code %d. The arm "
                    "will not move until this clears. Open http://192.168.58.2 "
                    "(admin / 123), read the alarm and clear it. Code 14 means "
                    "'interface execution failed' and usually means the robot "
                    "has latched a fault.", returncode);
                last_servoj_code = returncode;
                servoj_refusals  = 0;
            }
            servoj_refusals++;
        }else if(last_servoj_code != 0){
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                "ServoJ accepted again after %ld refused commands.",
                servoj_refusals);
            last_servoj_code = 0;
            servoj_refusals  = 0;
        }
        // --- end patch ------------------------------------------------------''',
"write() -> log a refusal once, at ERROR")

open(p, "w", encoding="utf-8", errors="surrogateescape").write(s)
PY

[ $? -eq 0 ] || { red "
Patch failed. The file is unchanged apart from the backup.
Restore with:  bash patch_driver.sh --revert"; exit 1; }

echo
grn "4. Patched."
echo
echo "   Now rebuild:"
echo "       cd $WS"
echo "       source /opt/ros/humble/setup.bash"
echo "       colcon build --packages-select fairino_hardware --symlink-install"
echo "       source install/local_setup.bash"
echo
ylw "   If the build fails, nothing reached the robot. Revert with:"
echo "       bash patch_driver.sh --revert"
echo
ylw "   Clear the alarm at http://192.168.58.2 before the next run,"
ylw "   and keep your hand on the e-stop. This changes how the arm is"
ylw "   commanded, so treat the first move as a first move."
echo
