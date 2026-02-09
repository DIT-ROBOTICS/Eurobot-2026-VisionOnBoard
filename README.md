# Eurobot-2026-VisionOnBoard

Vision system for Eurobot 2026 competition. Supports 4 RealSense cameras with ArUco marker detection.

## Quick Start

### 1. Start Docker
```bash
cd docker/
bash start_docker.sh
```

### 2. Launch Vision System

**Option A: tmux (recommended for debugging)**
```bash
./start_vision_tmux.sh
```
- Window 0: 4 camera panes (front, back, left, right)
- Window 1: Detector + Scanner nodes
- Window 2: RViz

**Option B: Single launch file**
```bash
# Inside Docker container
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch aruco_object multi_camera.launch.py
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `default_active_camera` | `right` | Initial active camera (front/back/left/right) |
| `team_color` | `blue` | Team color (blue/yellow) |
| `log_level` | `info` | ROS log level |

Example:
```bash
ros2 launch aruco_object multi_camera.launch.py default_active_camera:=front team_color:=yellow
```

## Switch Active Camera

Only one camera processes at a time. Publish to `/robot/dock_side` (Int16):

| Side | Value |
|------|-------|
| Front | 0 |
| Right | 1 |
| Back | 2 |
| Left | 3 |

```bash
ros2 topic pub /robot/dock_side std_msgs/msg/Int16 "data: 2" -1  # Switch to back
```

## RViz

```bash
xhost +local:docker  # Run BEFORE entering container
ros2 run rviz2 rviz2
```

## Architecture

```
start_vision_tmux.sh / multi_camera.launch.py
├── 4× RealSense cameras (640x480@15fps, depth disabled)
├── 4× object_detector nodes (ArUco pose detection)
└── 4× aruco_row_scanner nodes (hazelnut flip detection)
```

All nodes subscribe to `/robot/dock_side` - only the matching camera side processes images.

## Individual Node Launch

Launch individual camera, detector, or scanner nodes with camera position:

### Camera Node (RealSense)
```bash
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=front camera_name:=front \
    rgb_camera.color_profile:=640,480,15 enable_depth:=false
```
Replace `front` with: `front`, `back`, `left`, or `right`

### Object Detector Node
```bash
ros2 run aruco_object aruco_detector_node --ros-args \
    -p camera_position:=front
```

### Row Scanner Node
```bash
ros2 run aruco_object aruco_row_scanner --ros-args \
    -p camera_position:=front \
    -p team_color:=blue
```

### Multi-Node Launch Files
```bash
# Launch all 4 detector nodes
ros2 launch aruco_object detector_multi.launch.py default_dock_side:=1

# Launch all 4 scanner nodes
ros2 launch aruco_object scanner_multi.launch.py default_dock_side:=1 team_color:=blue
```

