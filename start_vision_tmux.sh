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

# === Pane 0: Camera Node ===
tmux send-keys -t $SESSION_NAME:0 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:0 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py camera_namespace:=back' C-m

# Split the window horizontally to create a second pane
tmux split-window -h -t $SESSION_NAME:0

# === Pane 1: Watchdog + ArUco Launcher ===
# This pane waits for camera topics to be available, then launches detector and scanner
WATCHDOG_CMD='
export ROS_DOMAIN_ID=13
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== Waiting for camera topics... ==="

# Wait for any camera_info topic matching pattern */camera/color/camera_info
while true; do
    TOPICS=$(ros2 topic list 2>/dev/null | grep -E "/camera/color/camera_info")
    if [ -n "$TOPICS" ]; then
        echo "=== Camera topic found: $TOPICS ==="
        break
    fi
    echo "Waiting for camera topics..."
    sleep 2
done

# Small delay to ensure camera is fully ready
sleep 2

echo "=== Launching ArUco Detector ==="
ros2 launch aruco_object detector_multi.launch.py &
DETECTOR_PID=$!

sleep 1

echo "=== Launching ArUco Scanner ==="
ros2 launch aruco_object scanner_multi.launch.py &
SCANNER_PID=$!

echo "=== All nodes launched! Detector PID: $DETECTOR_PID, Scanner PID: $SCANNER_PID ==="

# Wait for both processes
wait $DETECTOR_PID $SCANNER_PID
'

tmux send-keys -t $SESSION_NAME:0.1 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:0.1 "$WATCHDOG_CMD" C-m

# Create a new window for Rviz
tmux new-window -t $SESSION_NAME:1 -n 'rviz'
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
