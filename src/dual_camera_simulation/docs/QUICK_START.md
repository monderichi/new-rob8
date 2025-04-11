# Quick Start Guide: Dual RealSense Camera System

This guide provides quick instructions to get started with the dual RealSense camera simulation system that performs automatic calibration using an ArUco marker and fuses point clouds.

## Prerequisites

Make sure you have installed the required dependencies as described in the main README.md file.

## Running the Simulation

### 1. Run the complete simulation

The simplest way to start everything is using the demo script:

```bash
cd ~/dual_camera_ws
source devel/setup.bash
./src/dual_camera_simulation/scripts/demo.sh
```

This will launch:
- Gazebo with the dual camera setup and simulated environment
- Camera nodes with filtering
- ArUco marker detection
- Camera calibration
- Point cloud fusion
- RViz visualization

### 2. Individual components

If you prefer to run components individually:

**Start the simulation environment:**
```bash
roslaunch dual_camera_simulation simulation.launch rviz:=false
```

**Start RViz separately:**
```bash
roslaunch dual_camera_simulation rviz.launch
```

## Understanding the System

### Camera setup
- Camera 1 is the "reference" camera
- Camera 2 has an ArUco marker attached to its back
- Camera 1 detects the ArUco marker and calculates the transform to Camera 2

### Point cloud filtering
Each camera's point cloud is filtered using:
1. PassThrough filter to remove distant points
2. VoxelGrid filter for downsampling
3. Statistical Outlier Removal for noise reduction

### Visualization
In RViz, you'll see:
- The dual camera model and ArUco marker
- Individual filtered point clouds (different colors)
- The fused point cloud
- Camera and marker transforms

## Utility Tools

### Evaluate the calibration quality

To see the transform between camera frames after calibration:
```bash
rosrun dual_camera_simulation visualize_calibration.py camera1_link camera2_link
```

### Evaluate point cloud quality

To monitor point cloud statistics and fusion quality:
```bash
rosrun dual_camera_simulation evaluate_pointcloud.py --topic /merged_point_cloud
```

You can also monitor individual cameras:
```bash
rosrun dual_camera_simulation evaluate_pointcloud.py --topic /camera1/depth/filtered_points
```

## Troubleshooting

### ArUco marker not detected
- Make sure Camera 1 can see the marker in its field of view
- Adjust the lighting in Gazebo (use World menu → Scene → Ambient)
- Check the marker detection parameters in `config/aruco_detector_params.yaml`

### Poor point cloud fusion
- Verify camera calibration is complete (use the visualize_calibration.py tool)
- Check that both cameras are publishing point clouds
- Adjust the filtering parameters in the camera config files

### Nothing appears in RViz
- Make sure you've selected the correct Fixed Frame (usually "base_link")
- Ensure all RViz displays are enabled
- Verify that TF transforms are being published correctly

## Next Steps

1. Experiment with different filter parameters in `config/camera*_params.yaml`
2. Try modifying the camera positions in `models/dual_camera_setup.urdf.xacro`
3. Use the point cloud data in your own applications by subscribing to `/merged_point_cloud`