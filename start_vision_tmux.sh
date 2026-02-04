#!/bin/bash

SESSION_NAME="vision"
xhost +local:docker

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
# tmux send-keys -t $SESSION_NAME:0 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py camera_name:=rb_back_camera' C-m
tmux send-keys -t $SESSION_NAME:0 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py' C-m

# Split the window horizontally to create a second pane
tmux split-window -h -t $SESSION_NAME:0

# Send commands to the second pane (pane 1)
# Enter the running docker container
tmux send-keys -t $SESSION_NAME:0.1 'docker exec -it vision-ws bash' C-m
# Wait a brief moment
sleep 1
# Source ROS 2 Humble setup and local workspace setup
tmux send-keys -t $SESSION_NAME:0.1 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch aruco_object object_detector.launch.py' C-m

# Split the detector pane (pane 1) vertically to create a third pane for the scanner
tmux split-window -v -t $SESSION_NAME:0.1

# Send commands to the third pane (pane 2)
tmux send-keys -t $SESSION_NAME:0.2 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:0.2 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch aruco_object scanner.launch.py' C-m

# Create a new window for Rviz
tmux new-window -t $SESSION_NAME:1 -n 'rviz'
# Send commands to the new window
tmux send-keys -t $SESSION_NAME:1 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:1 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && source install/setup.bash' C-m
tmux send-keys -t $SESSION_NAME:1 'ros2 run rviz2 rviz2' C-m

# Function to kill the session when script exits
cleanup() {
    tmux kill-session -t $SESSION_NAME
}

# Trap EXIT signal (happens when script ends or user detaches)
trap cleanup EXIT

# Attach to the tmux session to view the output
tmux select-window -t $SESSION_NAME:0
tmux attach-session -t $SESSION_NAME
