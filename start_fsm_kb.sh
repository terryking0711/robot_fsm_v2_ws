#!/usr/bin/env bash
set -euo pipefail

SESSION="fsm"
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE="source '$WORKSPACE/use_fsm.sh'"

if [[ ! -f "$WORKSPACE/install/local_setup.bash" ]]; then
  echo "robot_fsm is not built. Run: $WORKSPACE/build_fsm.sh" >&2
  exit 1
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION"

# Pane 0: navigation_server
tmux send-keys -t "$SESSION:0" "$SOURCE && ros2 launch robot_navigation navigation_server.launch.py" Enter

# 水平切出 Pane 1: teleop_twist_keyboard
tmux split-window -h -t "$SESSION:0"
tmux send-keys -t "$SESSION:0.1" "$SOURCE && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r /cmd_vel:=/teleop/cmd_vel \
  -p repeat_rate:=20.0 \
  -p key_timeout:=0.3" Enter

# 垂直切出 Pane 2: stm_communication_node
tmux split-window -v -t "$SESSION:0.1"
tmux send-keys -t "$SESSION:0.2" "$SOURCE && ros2 run robot_fsm stm_communication_node --ros-args \
  -p input_cmd_vel_topic:=/teleop/cmd_vel \
  -p output_cmd_vel_topic:=/cmd_vel \
  -p cmd_vel_rate_hz:=20.0 \
  -p cmd_vel_stale_timeout_sec:=0.5" Enter

# attach
tmux attach -t "$SESSION"
