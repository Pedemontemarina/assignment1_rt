#!/bin/bash

SESSION="assignment1_rt_session"

# If the session already exists, kill it
tmux has-session -t $SESSION 2>/dev/null
if [ $? == 0 ]; then
    tmux kill-session -t $SESSION
fi

# Function to handle Ctrl+C
cleanup() {
    echo ""
    echo "Ctrl+C detected. Killing tmux session..."
    tmux kill-session -t $SESSION
    exit 0
}

# Set trap for SIGINT (Ctrl+C)
trap cleanup SIGINT

# Create new tmux session
tmux new-session -d -s $SESSION

# =========================
# Terminal 1: launch file
# =========================
tmux rename-window "launch"
tmux send-keys "source /home/ubuntu/ros_ws2/install/setup.bash && ros2 launch assignment1_rt assignment_launch.py" C-m

# =========================
# Terminal 2: turtle_spawn node
# =========================
tmux new-window -t $SESSION -n "turtle_spawn"
tmux send-keys "source /home/ubuntu/ros_ws2/install/setup.bash && ros2 run assignment1_rt turtle_spawn" C-m

# =========================
# Terminal 3: distance_node
# =========================
tmux new-window -t $SESSION -n "distance_node"
tmux send-keys "source /home/ubuntu/ros_ws2/install/setup.bash && ros2 run assignment1_rt distance_node" C-m

# =========================
# Terminal 4: ui_node
# =========================
tmux new-window -t $SESSION -n "ui_node"
tmux send-keys "source /home/ubuntu/ros_ws2/install/setup.bash && ros2 run assignment1_rt ui_node" C-m

# Attach to the tmux session
tmux attach-session -t $SESSION
