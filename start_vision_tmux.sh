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

# Camera positions and bandwidth-safe settings
# Camera positions and bandwidth-safe settings
CAMERA_POSITIONS=("front" "back" "left" "right")
CAMERA_SETTINGS='rgb_camera.color_profile:=640,480,15 enable_depth:=false enable_infra:=false'

# Camera Serial Numbers
FRONT_SERIAL="313522070126"
BACK_SERIAL="419122270813"
LEFT_SERIAL="218622276534"
RIGHT_SERIAL="218622276687"

# === Window 0: Cameras (4 panes, one for each camera) ===
# Pane 0: First camera (front)
tmux send-keys -t $SESSION_NAME:0 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:0 "export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py camera_namespace:=front camera_name:=front serial_no:=$FRONT_SERIAL $CAMERA_SETTINGS" C-m

# Split horizontally and launch second camera (back)
tmux split-window -h -t $SESSION_NAME:0
tmux send-keys -t $SESSION_NAME:0.1 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py camera_namespace:=back camera_name:=back serial_no:=$BACK_SERIAL $CAMERA_SETTINGS" C-m

# Split pane 0 vertically for third camera (left)
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.1 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py camera_namespace:=left camera_name:=left serial_no:=$LEFT_SERIAL $CAMERA_SETTINGS" C-m

# Split pane 2 vertically for fourth camera (right)
tmux split-window -v -t $SESSION_NAME:0.2
tmux send-keys -t $SESSION_NAME:0.3 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:0.3 "export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch realsense2_camera rs_launch.py camera_namespace:=right camera_name:=right serial_no:=$RIGHT_SERIAL $CAMERA_SETTINGS" C-m

# === Window 1: Detector & Scanner ===
tmux new-window -t $SESSION_NAME:1 -n 'nodes'

# === Pane 0: Object Detector (waits for all 4 cameras) ===
DETECTOR_CMD='
export ROS_DOMAIN_ID=13
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== [Detector] Waiting for all 4 camera topics... ==="
while true; do
    FRONT=$(ros2 topic list 2>/dev/null | grep -E "^/front/color/camera_info$")
    BACK=$(ros2 topic list 2>/dev/null | grep -E "^/back/color/camera_info$")
    LEFT=$(ros2 topic list 2>/dev/null | grep -E "^/left/color/camera_info$")
    RIGHT=$(ros2 topic list 2>/dev/null | grep -E "^/right/color/camera_info$")
    if [ -n "$FRONT" ] && [ -n "$BACK" ] && [ -n "$LEFT" ] && [ -n "$RIGHT" ]; then
        echo "=== [Detector] All 4 cameras found! ==="
        break
    fi
    echo "[Detector] Waiting... (front:${FRONT:-missing} back:${BACK:-missing} left:${LEFT:-missing} right:${RIGHT:-missing})"
    sleep 2
done
sleep 2
echo "=== [Detector] Launching ArUco Detector ==="
ros2 launch aruco_object detector_multi.launch.py
'

tmux send-keys -t $SESSION_NAME:1 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:1 "$DETECTOR_CMD" C-m

# Split for Scanner pane
tmux split-window -h -t $SESSION_NAME:1

# === Pane 1: Row Scanner (waits for all 4 cameras) ===
SCANNER_CMD='
export ROS_DOMAIN_ID=13
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== [Scanner] Waiting for all 4 camera topics... ==="
while true; do
    FRONT=$(ros2 topic list 2>/dev/null | grep -E "^/front/color/camera_info$")
    BACK=$(ros2 topic list 2>/dev/null | grep -E "^/back/color/camera_info$")
    LEFT=$(ros2 topic list 2>/dev/null | grep -E "^/left/color/camera_info$")
    RIGHT=$(ros2 topic list 2>/dev/null | grep -E "^/right/color/camera_info$")
    if [ -n "$FRONT" ] && [ -n "$BACK" ] && [ -n "$LEFT" ] && [ -n "$RIGHT" ]; then
        echo "=== [Scanner] All 4 cameras found! ==="
        break
    fi
    echo "[Scanner] Waiting... (front:${FRONT:-missing} back:${BACK:-missing} left:${LEFT:-missing} right:${RIGHT:-missing})"
    sleep 2
done
sleep 2
echo "=== [Scanner] Launching ArUco Scanner ==="
ros2 launch aruco_object scanner_multi.launch.py
'

tmux send-keys -t $SESSION_NAME:1.1 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:1.1 "$SCANNER_CMD" C-m

# === Window 2: RViz ===
tmux new-window -t $SESSION_NAME:2 -n 'rviz'
tmux send-keys -t $SESSION_NAME:2 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t $SESSION_NAME:2 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && source install/setup.bash' C-m
tmux send-keys -t $SESSION_NAME:2 'ros2 run rviz2 rviz2' C-m

# Function to kill the session when script exits
cleanup() {
    tmux kill-session -t $SESSION_NAME
}

# Trap EXIT signal (happens when script ends or user detaches)
trap cleanup EXIT

# Attach to the tmux session to view the output
tmux select-window -t $SESSION_NAME:0
tmux attach-session -t $SESSION_NAME
