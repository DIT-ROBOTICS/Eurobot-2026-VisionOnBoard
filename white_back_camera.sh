#!/bin/bash

SESSION="white_back_camera_test"

# Check if session exists and kill it if it does
tmux has-session -t $SESSION 2>/dev/null
if [ $? -eq 0 ]; then
    echo "Session $SESSION already exists. Killing it."
    tmux kill-session -t $SESSION
fi

# Create a new detached session
tmux new-session -d -s $SESSION

# Get the window ID to be robust against base-index settings
WIN_ID=$(tmux list-windows -t $SESSION -F "#{window_index}" | head -n 1)

# Split the window into 4 panes in a grid
# Start with Pane 0 (Whole screen)

# 1. Split vertically: Top (0) and Bottom (1)
tmux split-window -v -t ${SESSION}:${WIN_ID}.0

# 2. Split Top (0) horizontally: TL (0) and TR (1). Old Bottom becomes 2.
tmux split-window -h -t ${SESSION}:${WIN_ID}.0

# 3. Split Bottom (2) horizontally: BL (2) and BR (3).
tmux split-window -h -t ${SESSION}:${WIN_ID}.2

# Ensure tiled layout
tmux select-layout -t ${SESSION}:${WIN_ID} tiled

# Common commands for ALL panes
for i in {0..3}; do
    TARGET="${SESSION}:${WIN_ID}.${i}"
    tmux send-keys -t $TARGET "docker exec -it vision-ws bash" C-m
    tmux send-keys -t $TARGET "export ROS_DOMAIN_ID=11" C-m
    tmux send-keys -t $TARGET "colcon build" C-m
    tmux send-keys -t $TARGET "source install/setup.bash" C-m
    tmux send-keys -t $TARGET "clear" C-m
done

# Upper Left (Pane 0)
tmux send-keys -t ${SESSION}:${WIN_ID}.0 "ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=back \
    camera_name:=back \
    serial_no:=\"'218622278918'\" \
    rgb_camera.color_profile:=640,480,15 \
    enable_depth:=false" C-m

# Upper Right (Pane 1)
tmux send-keys -t ${SESSION}:${WIN_ID}.1 "ros2 run aruco_object aruco_row_scanner --ros-args \
    -p camera_position:=back \
    -p team_color:=yellow" C-m

# Lower Left (Pane 2)
tmux send-keys -t ${SESSION}:${WIN_ID}.2 "ros2 run aruco_object aruco_detector_node --ros-args \
    -p camera_position:=back" C-m

# Lower Right (Pane 3)
tmux send-keys -t ${SESSION}:${WIN_ID}.3 "ros2 topic hz /detected_dock_pose" C-m

# Attach to session
tmux attach -t $SESSION

# Kill session when detached (script continues here after user detaches)
tmux kill-session -t $SESSION
