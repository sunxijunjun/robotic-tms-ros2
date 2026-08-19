#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE}")" && pwd)
WS_DIR=$(cd "$SCRIPT_DIR/../../../../.." && pwd)

# Relaunch must always work: stop any previous stack instance (including
# orphaned nodes whose launch parent died) before starting a new one.
# SIGINT first for clean ROS shutdown, SIGKILL whatever survives 10 s.
# Note: the camera/AI pipeline (ai_results_publisher, launched separately from
# Documents/TMS/local_codes) is deliberately NOT in this list -- a bringup
# restart must leave the camera running.
STACK_RE="ros2 launch tms_robot_bringup|robot_task_executor|ros2_control_node|move_group|tms_robot_ui|tcp_pose_publisher|rq_sensor|dashboard_client|controller_stopper|ur_robot_state_helper|demo_operator"
if pgrep -f "$STACK_RE" > /dev/null; then
  echo "Stopping previous TMS stack..."
  pkill -INT -f "$STACK_RE"
  for _ in $(seq 1 20); do
    pgrep -f "$STACK_RE" > /dev/null || break
    sleep 0.5
  done
  pkill -KILL -f "$STACK_RE" 2> /dev/null
  sleep 1
  echo "Previous stack stopped."
fi

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/local_setup.bash"
export RCUTILS_COLORIZED_OUTPUT=1
export QT_QPA_PLATFORM=xcb

# Session log for post-hoc debugging: every line of node output, stamped with
# wall-clock time, colors stripped. One file per bringup session.
LOG_DIR="$HOME/tms_session_logs"
mkdir -p "$LOG_DIR"
SESSION_LOG="$LOG_DIR/session_$(date +%F_%H-%M-%S).log"
echo "Session log: $SESSION_LOG"
export PYTHONUNBUFFERED=1   # launch's own stdout is a pipe now; keep it line-buffered
set -o pipefail
# The log writers ignore SIGINT so Ctrl+C in this terminal still captures the
# whole shutdown sequence; they exit when the launch closes the pipe.
ros2 launch tms_robot_bringup all.launch.py 2>&1 \
  | { trap '' INT; shopt -s extglob
      while IFS= read -r line; do printf '%(%F %T)T %s\n' -1 "${line//$'\e'\[*([0-9;])m/}"; done; } \
  | { trap '' INT; exec tee "$SESSION_LOG"; }
