#!/bin/bash
# One-button full restart: robot stack (terminal) + camera pipeline (background).
# run_all_launch.sh already stops any previous stack instance; the camera is
# restarted here (it is deliberately excluded from the stack cleanup so that
# stack-only restarts leave it running).

pkill -INT -f "0_4_estimate_normal.py" 2> /dev/null && sleep 2

# Closing the control panel shuts the stack down (all.launch.py); this terminal
# then stops the camera started by this same run (pid file keyed by run, so a
# terminal left over from a previous run cannot kill the new camera) and closes.
# It only stays open on error.
LOG_DIR="$HOME/tms_session_logs"
mkdir -p "$LOG_DIR"
CAM_PIDFILE="$LOG_DIR/camera_$$.pid"
gnome-terminal --title="TMS Robot Bringup" -- bash -c \
  "$HOME/tms_ws/install/tms_robot_bringup/share/tms_robot_bringup/desktop/run_all_launch.sh; rc=\$?; \
   [ -f $CAM_PIDFILE ] && { kill -INT \$(cat $CAM_PIDFILE) 2> /dev/null; rm -f $CAM_PIDFILE; }; \
   [ \$rc -eq 0 ] || { echo ''; echo \"Stack exited with code \$rc. Press Enter to close...\"; read; }"

# Camera normal estimation in the background (no terminal); output goes to ~/tms_session_logs/camera_*.log.
sleep 5
(
  source /opt/ros/jazzy/setup.bash
  source "$HOME/tms_ws/install/local_setup.bash"
  cd "$HOME/Documents/TMS/local_codes/camera_ros2_topic"
  exec nohup python3 -u 0_4_estimate_normal.py > "$LOG_DIR/camera_$(date +%F_%H-%M-%S).log" 2>&1
) &
echo $! > "$CAM_PIDFILE"
