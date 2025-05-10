# RealSense ArUco-based Point Cloud Merging System ROS

This ROS  package enables the fusion of point clouds from two Intel RealSense depth cameras using ArUco markers for extrinsic calibration. The system dynamically calculates the transformation between cameras by detecting the same ArUco marker from both camera views, then merges the point clouds into a unified coordinate frame.

## Features

- **Dual Camera ArUco Detection**: Both cameras detect the same ArUco marker to establish spatial relationships
- **Dynamic Transformation Calculation**: Real-time computation of camera-to-camera transformations
- **Point Cloud Merging**: Unified point cloud from multiple cameras in a common coordinate frame
- **Visualization Tools**: RViz configurations for visualizing all aspects of the system
- **Flexible Configuration**: Support for different marker sizes, camera models, and positioning

## Requirements

- Ubuntu 20.04 with ROS Noetic
- Two Intel RealSense depth cameras (tested with D435 and D435i)
- ArUco markers (4x4 dictionary, marker size 20mm, ID 1 recommended)
- Proper lighting conditions for marker detection

## Installation

1. **Clone this repository into your catkin workspace:**

```bash
cd ~/catkin_ws/src/
git clone https://github.com/jonaskoda/ROB8-862-semester-project.git -b pointcloud-merging realsense_aruco_merger
```

2. **Install dependencies:**

```bash
cd ~/catkin_ws/src/realsense_aruco_merger
chmod +x install_dependencies.sh
./install_dependencies.sh
```

3. **Build the package:**

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## System Setup

### Hardware Setup

1. Connect both RealSense cameras to your computer
2. Position the cameras to have overlapping fields of view
3. Place an ArUco marker (ID 1, 20mm size) where both cameras can see it clearly

### Software Configuration

The system provides three main launch files for different use cases:

1. **Basic test for ArUco detection:**
```bash
roslaunch realsense_aruco_merger test_aruco_detector.launch
```

2. **Original approach (marker attached to one camera):**
```bash
roslaunch realsense_aruco_merger merge_cameras.launch
```

3. **Recommended approach (both cameras see the same marker):**
```bash
roslaunch realsense_aruco_merger dual_camera_aruco_merge.launch
```

## Using the System

### Running the Recommended Dual Camera Mode

1. Make sure both cameras are connected to your computer
2. Place ArUco marker ID 1 (4x4 dictionary, 20mm size) in the visible area of both cameras
3. Launch the system:
   ```bash
   roslaunch realsense_aruco_merger dual_camera_aruco_merge.launch
   ```
4. The RViz window will show:
   - Live feeds from both cameras with marker detection
   - Individual point clouds from both cameras
   - Merged point cloud (highlighted in green)
   - Transform (TF) tree showing the camera relationships

### Adjusting Parameters

You can adjust various parameters in the launch file:

- **Camera serial numbers**: Update `serial_no_cam1` and `serial_no_cam2` if you have different cameras
- **Marker properties**: Adjust `marker_size` and `marker_id` if using different markers
- **Frame IDs**: Change the frame names if needed for integration with other systems

Example:
```bash
roslaunch realsense_aruco_merger dual_camera_aruco_merge.launch marker_size:=0.03 marker_id:=2
```

## Troubleshooting

### No ArUco Detection

- Ensure adequate lighting conditions
- Check that you're using the correct ArUco dictionary (4x4_50)
- Verify the marker is the correct ID (1 by default)
- Make sure the marker size parameter matches your physical marker (default: 20mm = 0.02m)

### Point Clouds Not Merging

- Check if both cameras can simultaneously see the ArUco marker
- Ensure transformations are being published (check TF tree in RViz)
- Look for error messages in the console about transformation failures

### Camera Issues

- Verify the serial numbers in the launch file match your cameras
- Check USB connections and bandwidth
- Update to the latest RealSense SDK if having driver issues

## System Architecture

The system consists of three main components:

1. **Dual Camera ArUco Detector**: Detects ArUco markers in both camera feeds and computes transformations
2. **Point Cloud Merger**: Transforms and combines point clouds using the calculated transformations
3. **Visualization System**: Displays camera feeds, point clouds, and transformations

## Key Topics Published

- `/cam1/aruco_result`, `/cam2/aruco_result`: Images showing detected markers
- `/cam1/cam1/depth/color/points`, `/cam2/cam2/depth/color/points`: Original point clouds
- `/merged_pointcloud`: Combined point cloud in the reference frame
- `/tf`: Transformation tree including camera-to-marker and camera-to-camera relationships

## Contributing

Feel free to create issues or pull requests if you have suggestions for improvements or bug fixes.

## License

[MIT License](LICENSE)
