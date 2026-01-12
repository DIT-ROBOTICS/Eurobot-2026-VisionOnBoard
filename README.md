# Eurobot-2026-VisionOnBoard
This repository contains the vision system for the Eurobot 2026 competition.
## Start with Docker
### Build image and container
```bash
cd docker/
bash start_docker.sh
```

### Launch realsense node and hazelnut detector (objector_detector.cpp) together
```
bash start_vision_tmux.sh
```

### Run rviz
Before going in the container, run this in terminal to allow allow GUI window inside container
```
xhost +local:docker
```
Open RViz to visualize camera images and TF frames:
```bash
ros2 run rviz2 rviz2
```
