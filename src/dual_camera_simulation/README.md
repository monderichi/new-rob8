# Dual RealSense Camera Simulation

A ROS Noetic package for simulating two Intel RealSense D435 cameras with automatic calibration using ArUco markers and point cloud fusion.

![System Architecture](docs/system_architecture.png)

## Features

- **Dual Camera Simulation** - Simulates two RealSense D435 cameras in Gazebo
- **ArUco Marker-based Calibration** - Automatically calibrates the relative pose between cameras
- **Point Cloud Processing** - Provides filtering and merging of point clouds
- **Visualization Tools** - RViz configuration for viewing the merged point cloud
- **Diagnostic Utilities** - Scripts for evaluating calibration quality and point cloud fusion

## Dependencies

- ROS Noetic
- Gazebo 11
- PCL (Point Cloud Library)
- OpenCV (with ArUco support)
- Aruco ROS

## Installation

### 1. Install ROS dependencies:

```bash
sudo apt update
sudo apt install ros-noetic-aruco-ros ros-noetic-pcl-ros ros-noetic-gazebo-ros-pkgs
```

### 2. Clone the repository:

```bash
cd ~/dual_camera_ws/src
git clone https://github.com/your_username/dual_camera_simulation.git
cd ..
```

### 3. Build the workspace:

```bash
catkin_make
source devel/setup.bash
```

## Quick Start

### Run the Demo

The simplest way to start everything is using the demo script:

```bash
./src/dual_camera_simulation/scripts/demo.sh
```

This will launch:
- Gazebo with the dual camera setup and simulated environment
- Camera nodes with filtering
- ArUco marker detection
- Camera calibration
- Point cloud fusion
- RViz visualization

### Running Individual Components

If you prefer to run components individually:

**Start the simulation environment:**
```bash
roslaunch dual_camera_simulation simulation.launch rviz:=false
```

**Start RViz separately:**
```bash
roslaunch dual_camera_simulation rviz.launch
```

**Record point cloud data:**
```bash
roslaunch dual_camera_simulation record_pointcloud.launch duration:=30
```

## System Components

### Camera Model
The system simulates two Intel RealSense D435 depth cameras with the following specifications:
- RGB Resolution: 640×480 pixels
- Depth Resolution: 640×480 pixels
- Field of View: 87° × 58°
- Depth Range: 0.1 - 3.0 meters

### ArUco Marker
An ArUco marker (ID: 23, Size: 5cm) is attached to camera 2 and used for calibration.

### Point Cloud Processing Pipeline

Each camera's point cloud goes through the following processing steps:
1. **PassThrough filter**: Removes points outside the depth range
2. **VoxelGrid filter**: Downsamples the point cloud for efficiency
3. **Statistical Outlier Removal**: Eliminates noise and outliers
4. **Transform**: Transforms both point clouds to a common reference frame
5. **Fusion**: Merges the point clouds into a unified cloud

## Utility Tools

### Evaluate Calibration Quality

```bash
rosrun dual_camera_simulation visualize_calibration.py camera1_link camera2_link
```

### Evaluate Point Cloud Quality

```bash
rosrun dual_camera_simulation evaluate_pointcloud.py --topic /merged_point_cloud
```

## Configuration

### Camera Parameters

Configuration files in the `config` directory control the camera properties and filtering parameters:
- `camera1_params.yaml`
- `camera2_params.yaml`
- `aruco_detector_params.yaml`

Adjust these files to modify camera properties and filtering behavior.

## Documentation

Detailed documentation is available in the `docs` directory:
- [Quick Start Guide](docs/QUICK_START.md) - Getting started with the system
- [Technical Documentation](docs/TECHNICAL.md) - Detailed system architecture

## Extending the System

### Adding More Cameras
1. Add new camera definitions to the URDF model
2. Update the calibration node to handle multiple camera transforms
3. Modify the point cloud merger to accept additional point clouds

### Custom Filtering Pipeline
1. Modify `point_cloud_filter_node.cpp` to add or replace filters
2. Update parameter configuration in YAML files

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

- Intel RealSense for the camera specifications
- ROS and Gazebo communities for simulation tools
- OpenCV for ArUco marker detection
- PCL for point cloud processing tools