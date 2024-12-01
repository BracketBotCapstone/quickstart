#!/bin/bash

# Start a new tmux session named 'roomba'
tmux new-session -d -s roomba

# Enable mouse support in tmux
tmux set-option -g mouse on

# Split the window into a 3x2 grid
tmux split-window -h -t roomba
tmux split-window -v -t roomba:0.0
tmux split-window -v -t roomba:0.1
tmux split-window -v -t roomba:0.3
tmux split-window -v -t roomba:0.4

# Select each pane and launch the corresponding node
tmux select-pane -t roomba:0.0
tmux send-keys "python3 imu.py" C-m

tmux select-pane -t roomba:0.1
tmux send-keys "cd localization && python3 localization_imu_enc.py" C-m

tmux select-pane -t roomba:0.2
tmux send-keys "cd mapping && python3 mapping_wavemap.py" C-m

tmux select-pane -t roomba:0.3
tmux send-keys "python3 control_loop.py" C-m

# tmux select-pane -t roomba:0.4
# tmux send-keys "python3 ../nodes/rerun_viewer/main.py" C-m

tmux select-pane -t roomba:0.5
tmux send-keys "python3 roomba.py" C-m

# Attach to the tmux session
tmux attach-session -t roomba
