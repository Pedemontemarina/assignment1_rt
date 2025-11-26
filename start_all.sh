#!/bin/bash

# Name of the tmux session
SESSION="ros_nodes"
tmux new-session -d -s $SESSION

# =========================
# Terminal 1: launch file
# =========================
tmux rename-window "launch"
tmux send-keys "source /home/ubuntu/ros_ws2/install/setup.bash && ros2 launch assignment1_rt assignment_launch.py" C-m

# =========================
# Terminal 2: turtle_spawn
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

tmux attach-session -t $SESSION
