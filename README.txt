# Dual Camera Point Cloud Merging with ArUco Marker Calibration

## Overview
This ROS project integrates two Intel RealSense depth cameras (e.g., D435i and D405) to produce a merged point cloud. It utilizes ArUco markers for spatial calibration and TF (transform) management between the cameras and a world reference frame (often defined by a primary ArUco marker).

## Key Features
- Processes streams from two RealSense cameras.
- Detects ArUco markers to establish camera-to-marker transformations.
- Merges point clouds from both cameras into a common reference frame.
- Provides launch files for camera operation, calibration procedures, and the main merging application.
- Includes RViz configurations for visualizing camera views, TFs, and merged point clouds.
- Offers configurable parameters for camera settings (resolution, FPS, clip_distance), ArUco marker details (ID, size, dictionary), and point cloud filtering (voxel grid, passthrough, outlier removal).

## Workspace Structure
- `/home/monder/catkin_ws/`: Root of the Catkin workspace.
  - `launch/`: ROS launch files.
    - `dual_camera_aruco_merge.launch`: Main launch file for starting cameras and merging point clouds.
    - `calibrate_*.launch`, `multi_marker_alignment.launch`: Launch files for various calibration routines.
  - `scripts/`: Python helper scripts (e.g., ArUco detectors, TF visualizers, calibration utilities).
  - `src/`: Source code for ROS packages.
    - `realsense_aruco_merger/`: Primary package containing custom nodes and scripts for this project.
    - C++ nodes (e.g., `aruco_pointcloud_merger_node.cpp`) for performance-critical tasks.
  - `rviz/`: RViz configuration files.
  - `pcl/`: Contains the Point Cloud Library (PCL). Note: This might be a direct clone or submodule.
  - `README.txt`: This explanation file.

## Prerequisites
- ROS (e.g., Noetic)
- Intel RealSense SDK 2.0 (librealsense2)
- RealSense ROS Wrapper (`realsense2_camera` package)
- ArUco marker support for ROS
- Point Cloud Library (PCL) and its ROS interfaces (`pcl_ros`)
- Eigen (typically a PCL dependency)

## Setup & Build
1.  Ensure all prerequisites are installed.
2.  If this repository is cloned fresh, place it appropriately in your Catkin workspace (e.g. `catkin_ws/src/your_project_package_name`).
3.  If `realsense_aruco_merger` or other components are submodules, initialize them: `git submodule update --init --recursive`.
4.  Install project-specific dependencies (e.g., using `install_dependencies.sh` if provided).
5.  Navigate to the Catkin workspace root: `cd /home/monder/catkin_ws`
6.  Build the workspace: `catkin_make`
7.  Source the workspace: `source devel/setup.bash`

## Running the System

### 1. Calibration
- **Purpose:** To determine the spatial relationship (TF transform) between cameras and the world frame (e.g., `marker0`).
- **Process:**
    - Refer to launch files like `calibrate_cam2_to_marker0.launch` or `multi_marker_alignment.launch`.
    - Position markers as required by the calibration routine.
    - Run the calibration launch file.
    - Use `tf_echo <source_frame> <target_frame>` to get the transform values.
    - Update the static transform publisher arguments in `dual_camera_aruco_merge.launch` for `cam2` with these values.
    - The `clip_distance` for `cam2` (D405) is set to 0.5m.

### 2. Merging Point Clouds
- **Launch Command:** `roslaunch realsense_aruco_merger dual_camera_aruco_merge.launch` (adjust package name if it differs).
- **Functionality:**
    - Starts both RealSense cameras.
    - `cam1` detects a primary ArUco marker (e.g., `marker_id="0"`) serving as the `world_frame`.
    - A static transform (from calibration) defines `cam2`'s pose relative to this `world_frame`.
    - Point clouds are transformed and merged by `aruco_pointcloud_merger_node`.
    - RViz (`dual_camera_merge_optimized.rviz`) visualizes the process.

## Key Configuration Points in `dual_camera_aruco_merge.launch`
- Camera serial numbers.
- Marker parameters (size, ID, dictionary).
- `world_frame` definition.
- Static TF for `cam2`.
- `clip_distance` for D405.
- Point cloud filtering parameters.

## Notes
- The `pcl/` directory at `catkin_ws/pcl/` contains PCL source. Ensure it's correctly built if used directly, or rely on system/ROS PCL.
- The `realsense-ros-ros1-legacy/` directory appears to be an older RealSense ROS wrapper. The system likely uses the standard `realsense2_camera` package.
- Use `rosrun tf view_frames` and RViz for debugging.
