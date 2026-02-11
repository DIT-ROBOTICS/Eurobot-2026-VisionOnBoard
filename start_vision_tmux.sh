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

# Create a new detached session
tmux new-session -d -s $SESSION_NAME

# Get the window ID to be robust against base-index settings
WIN_ID=$(tmux list-windows -t $SESSION_NAME -F "#{window_index}" | head -n 1)

# === Window 0: Vision System (2x2 grid) ===
# Start with Pane 0 (Top Left)

# 1. Split vertically: Top (0) and Bottom (1)
tmux split-window -v -t ${SESSION_NAME}:${WIN_ID}

# 2. Split Top (0) horizontally: TL (0) and TR (1). Old Bottom becomes 2.
tmux split-window -h -t ${SESSION_NAME}:${WIN_ID}.0

# 3. Split Bottom (2) horizontally: BL (2) and BR (3).
tmux split-window -h -t ${SESSION_NAME}:${WIN_ID}.2

# Ensure tiled layout
tmux select-layout -t ${SESSION_NAME}:${WIN_ID} tiled

# Common setup for all panes
# Note: Panes are indexed 0 (TL), 1 (TR), 2 (BL), 3 (BR) *after* the splits and select-layout tiled?
# Let's double check standard behavior or rely on targeting.
# With "tiled", indices might shift or be stable. 
# Layout logic:
# 0 -> TL
# 1 -> TR
# 2 -> BL
# 3 -> BR
# Let's verify targeting.

# Pane 0 (Top Left): All Cameras
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.0 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.0 "export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 launch realsense2_camera rs_multi_camera_launch.py" C-m

# Pane 1 (Top Right): Detector
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.1 'docker exec -it vision-ws bash' C-m
sleep 1
DETECTOR_CMD='
export ROS_DOMAIN_ID=13
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== [Detector] Waiting for BACK, LEFT, RIGHT camera topics... ==="
while true; do
    BACK=$(ros2 topic list 2>/dev/null | grep -E "/back/back/color/camera_info")
    LEFT=$(ros2 topic list 2>/dev/null | grep -E "/left/left/color/camera_info")
    RIGHT=$(ros2 topic list 2>/dev/null | grep -E "/right/right/color/camera_info")
    if [ -n "$BACK" ] && [ -n "$LEFT" ] && [ -n "$RIGHT" ]; then
        echo "=== [Detector] All 3 cameras found! ==="
        break
    fi
    echo "[Detector] Waiting... (back:${BACK:-missing} left:${LEFT:-missing} right:${RIGHT:-missing})"
    sleep 2
done
sleep 2
echo "=== [Detector] Launching ArUco Detector ==="
ros2 launch aruco_object detector_multi.launch.py
'
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.1 "$DETECTOR_CMD" C-m

# Pane 2 (Bottom Left): Scanner
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.2 'docker exec -it vision-ws bash' C-m
sleep 1
SCANNER_CMD='
export ROS_DOMAIN_ID=13
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== [Scanner] Waiting for BACK, LEFT, RIGHT camera topics... ==="
while true; do
    BACK=$(ros2 topic list 2>/dev/null | grep -E "/back/back/color/camera_info")
    LEFT=$(ros2 topic list 2>/dev/null | grep -E "/left/left/color/camera_info")
    RIGHT=$(ros2 topic list 2>/dev/null | grep -E "/right/right/color/camera_info")
    if [ -n "$BACK" ] && [ -n "$LEFT" ] && [ -n "$RIGHT" ]; then
        echo "=== [Scanner] All 3 cameras found! ==="
        break
    fi
    echo "[Scanner] Waiting... (back:${BACK:-missing} left:${LEFT:-missing} right:${RIGHT:-missing})"
    sleep 2
done
sleep 2
echo "=== [Scanner] Launching ArUco Scanner ==="
ros2 launch aruco_object scanner_multi.launch.py
'
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.2 "$SCANNER_CMD" C-m

# Pane 3 (Bottom Right): Shell / Utils
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.3 'docker exec -it vision-ws bash' C-m
sleep 1
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.3 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.bash && source install/setup.bash' C-m
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.3 'echo "Ready to use. Run: ros2 topic list"' C-m
tmux send-keys -t ${SESSION_NAME}:${WIN_ID}.3 'ros2 topic list' C-m

# Function to kill the session when script exits
cleanup() {
    tmux kill-session -t $SESSION_NAME
}

# Trap EXIT signal (happens when script ends or user detaches)
trap cleanup EXIT

# Attach to the tmux session to view the output
tmux select-window -t ${SESSION_NAME}:${WIN_ID}
tmux attach-session -t $SESSION_NAME
