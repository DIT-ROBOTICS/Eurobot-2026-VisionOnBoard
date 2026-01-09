#!/bin/bash

SESSION_NAME="vision"

# Check if the session exists
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? == 0 ]; then
  # Session exists, kill it to start fresh
  echo "Session $SESSION_NAME already exists. Killing it."
  tmux kill-session -t $SESSION_NAME
fi

# Create a new tmux session named 'vision' but don't attach to it yet
tmux new-session -d -s $SESSION_NAME

# Send commands to the first pane (pane 0)
# Enter the running docker container
tmux send-keys -t $SESSION_NAME:0 'docker exec -it vision-ws bash' C-m
# Wait a brief moment for the container shell to be ready
sleep 1
# Source ROS 2 Humble setup and local workspace setup
tmux send-keys -t $SESSION_NAME:0 'export ROS_DOMAIN_ID=14 && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash' C-m
# Launch the RealSense camera node
tmux send-keys -t $SESSION_NAME:0 'ros2 launch realsense2_camera rs_launch.py' C-m

# Split the window horizontally to create a second pane
tmux split-window -h -t $SESSION_NAME:0

# Send commands to the second pane (pane 1)
# Enter the running docker container
tmux send-keys -t $SESSION_NAME:0.1 'docker exec -it vision-ws bash' C-m
# Wait a brief moment
sleep 1
# Source ROS 2 Humble setup and local workspace setup
tmux send-keys -t $SESSION_NAME:0.1 'export ROS_DOMAIN_ID=14 && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash' C-m
# Run the Aruco object detector node
tmux send-keys -t $SESSION_NAME:0.1 'ros2 run aruco_object object_detector' C-m

# Attach to the tmux session to view the output
tmux attach-session -t $SESSION_NAME
