#!/bin/bash

# Demo script for the dual RealSense camera simulation system
# This script launches the complete simulation environment with the dual camera setup

echo "Starting the dual RealSense camera simulation demo..."

# Source ROS setup.bash (Docker container path)
source /opt/ros/noetic/setup.bash

# Source the workspace's setup.bash (absolute path)
source /root/dual_camera_ws/devel/setup.bash

# Launch the simulation
echo "Launching simulation environment with both cameras, ArUco marker detection, and point cloud processing..."
roslaunch dual_camera_simulation simulation.launch

echo "Demo has been stopped."
exit 0