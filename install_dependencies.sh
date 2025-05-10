#!/bin/bash
#
# Installation script for RealSense ArUco Marker Point Cloud Merger
# This script installs all necessary dependencies for the project
#

echo "====================================================="
echo "RealSense ArUco Point Cloud Merger - Dependency Setup"
echo "====================================================="

# Make sure we have the latest package lists
echo -e "\n[1/8] Updating package lists..."
sudo apt-get update

# Install basic ROS dependencies
echo -e "\n[2/8] Installing ROS dependencies..."
sudo apt-get install -y ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-message-filters python3-rosdep

# Install PCL dependencies
echo -e "\n[3/8] Installing PCL dependencies..."
sudo apt-get install -y ros-noetic-pcl-ros ros-noetic-pcl-conversions

# Install image processing dependencies
echo -e "\n[4/8] Installing image processing dependencies..."
sudo apt-get install -y ros-noetic-cv-bridge ros-noetic-image-transport

# Install RealSense packages
echo -e "\n[5/8] Installing Intel RealSense packages..."
# Add RealSense repository
if [ ! -f /etc/apt/sources.list.d/realsense.list ]; then
    echo "Adding RealSense repository..."
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
fi

# Install the core packages
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg ros-noetic-realsense2-camera

# Clone ArUco ROS wrapper if not already installed
echo -e "\n[6/8] Installing ArUco ROS wrapper..."
if [ ! -d /home/monder/catkin_ws/src/aruco_ros ]; then
    cd /home/monder/catkin_ws/src
    echo "Cloning ArUco ROS repository..."
    git clone https://github.com/pal-robotics/aruco_ros.git
else
    echo "ArUco ROS repository already exists, updating..."
    cd /home/monder/catkin_ws/src/aruco_ros
    git pull
fi

# Create the source files directory if not exists
echo -e "\n[7/8] Setting up Python scripts..."
mkdir -p /home/monder/catkin_ws/src/realsense_aruco_merger/src

# Create the Python scripts
cat > /home/monder/catkin_ws/src/realsense_aruco_merger/src/pointcloud_merger.py << 'EOL'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import message_filters
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped

class PointCloudMerger:
    def __init__(self):
        rospy.init_node('pointcloud_merger', anonymous=True)
        
        # Get parameters
        self.cam1_frame = rospy.get_param('~cam1_frame', 'cam1_link')
        self.cam2_frame = rospy.get_param('~cam2_frame', 'cam2_link')
        self.target_frame = rospy.get_param('~target_frame', 'cam1_link')  # Reference frame for merged cloud
        self.cam1_topic = rospy.get_param('~cam1_topic', '/cam1/depth/color/points')
        self.cam2_topic = rospy.get_param('~cam2_topic', '/cam2/depth/color/points')
        self.merged_topic = rospy.get_param('~merged_topic', '/merged_pointcloud')
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher for merged pointcloud
        self.merged_pub = rospy.Publisher(self.merged_topic, PointCloud2, queue_size=1)
        
        # Setup synchronization for receiving point clouds
        self.cam1_sub = message_filters.Subscriber(self.cam1_topic, PointCloud2)
        self.cam2_sub = message_filters.Subscriber(self.cam2_topic, PointCloud2)
        
        # Time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.cam1_sub, self.cam2_sub], 
            queue_size=5, 
            slop=0.1  # Allow up to 0.1 second time difference
        )
        self.sync.registerCallback(self.pointcloud_callback)
        
        rospy.loginfo("PointCloud merger initialized.")
        rospy.loginfo(f"Merging: {self.cam1_topic} and {self.cam2_topic}")
        rospy.loginfo(f"Publishing to: {self.merged_topic}")
        rospy.loginfo(f"Reference frame: {self.target_frame}")
        
    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),  # Get latest available transform
                rospy.Duration(1.0)  # Wait up to 1 second for transform
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get transform from {source_frame} to {target_frame}: {e}")
            return None
        
    def pointcloud_callback(self, cloud1_msg, cloud2_msg):
        # Get the transforms
        try:
            # If cam1 is the target frame, we don't need to transform it
            if self.cam1_frame != self.target_frame:
                transform1 = self.get_transform(self.target_frame, cloud1_msg.header.frame_id)
                if transform1 is None:
                    return
                cloud1_transformed = do_transform_cloud(cloud1_msg, transform1)
            else:
                cloud1_transformed = cloud1_msg  # No transform needed
                
            # Get transform from cam2 to target frame
            transform2 = self.get_transform(self.target_frame, cloud2_msg.header.frame_id)
            if transform2 is None:
                return
            cloud2_transformed = do_transform_cloud(cloud2_msg, transform2)
            
            # Merge pointclouds by concatenating them
            # In a real-world scenario, we might want to do more complex processing,
            # like filtering overlapping points or applying weights
            points1 = list(pc2.read_points(cloud1_transformed))
            points2 = list(pc2.read_points(cloud2_transformed))
            
            # Create merged pointcloud
            merged_cloud = PointCloud2()
            merged_cloud.header = cloud1_msg.header  # Use header from first cloud
            merged_cloud.header.frame_id = self.target_frame
            
            # Combine the point clouds
            # Use the same field structure as the input clouds
            if len(points1) > 0 and len(points2) > 0:
                fields = cloud1_transformed.fields
                merged_points = points1 + points2
                
                # Create and publish the merged cloud
                merged_cloud = pc2.create_cloud(
                    cloud1_transformed.header,
                    fields,
                    merged_points
                )
                self.merged_pub.publish(merged_cloud)
                rospy.logdebug(f"Published merged cloud with {len(merged_points)} points")
            else:
                rospy.logwarn("One of the point clouds was empty. Skipping merge operation.")
                
        except Exception as e:
            rospy.logerr(f"Error in point cloud merger: {e}")

if __name__ == '__main__':
    try:
        merger = PointCloudMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
EOL

cat > /home/monder/catkin_ws/src/realsense_aruco_merger/src/aruco_tf_visualizer.py << 'EOL'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, PoseStamped
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray
from std_msgs.msg import ColorRGBA

class ArucoTFVisualizer:
    def __init__(self):
        rospy.init_node('aruco_tf_visualizer', anonymous=True)
        
        # Get parameters
        self.marker_frame = rospy.get_param('~marker_frame', 'aruco_marker_frame')
        self.marker_size = float(rospy.get_param('~marker_size', 0.02))  # marker size in meters
        
        # Subscribers
        rospy.Subscriber('/aruco_single/pose', PoseStamped, self.marker_callback)
        
        # Publishers
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=1)

        # TF buffer for listening to transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("ArUco TF visualizer initialized")
        
    def marker_callback(self, pose_msg):
        # Create a MarkerArray to hold all visualization markers
        marker_array = MarkerArray()
        
        # Create a marker for the ArUco tag
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "aruco_markers"
        marker.id = 0  # Using ID 0 since we're tracking a specific marker
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set marker size based on our ArUco marker size
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = 0.001  # Very thin in Z direction
        
        # Set marker color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        
        # Set marker pose (identity - because it will inherit from the frame)
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        
        marker_array.markers.append(marker)
        
        # Also publish marker as individual marker for compatibility
        self.marker_pub.publish(marker)
        
        # Publish the marker array
        if marker_array.markers:
            self.marker_array_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        visualizer = ArucoTFVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
EOL

# Make the Python scripts executable
chmod +x /home/monder/catkin_ws/src/realsense_aruco_merger/src/pointcloud_merger.py
chmod +x /home/monder/catkin_ws/src/realsense_aruco_merger/src/aruco_tf_visualizer.py

# Update the launch file
echo -e "\n[8/8] Setting up launch file..."
if [ ! -d /home/monder/catkin_ws/src/realsense_aruco_merger/launch ]; then
    mkdir -p /home/monder/catkin_ws/src/realsense_aruco_merger/launch
fi

cat > /home/monder/catkin_ws/src/realsense_aruco_merger/launch/merge_cameras.launch << 'EOL'
<launch>

    <!-- Arguments for camera serial numbers and marker details -->
    <arg name="serial_no_cam1"    default="841612071768"/> <!-- D435i (Sees the marker) -->
    <arg name="serial_no_cam2"    default="827112072033"/> <!-- D435 (Has marker attached) -->

    <arg name="marker_size"       default="0.02"/> <!-- Marker size in meters (20mm = 0.02m) -->
    <arg name="marker_id"         default="0"/>
    <arg name="marker_dict"       default="DICT_4X4_50"/> <!-- 4x4 dictionary as requested -->
    <arg name="marker_frame"      default="aruco_marker_frame"/>
    <arg name="cam1_base_frame"   default="cam1_link"/>
    <arg name="cam2_base_frame"   default="cam2_link"/>
    <!-- Frame where the marker is detected (optical frame of cam1 color sensor) -->
    <arg name="cam1_color_optical_frame" default="cam1_color_optical_frame"/>
    <!-- Frame relative to which the marker is attached (e.g., optical frame of cam2 color sensor) -->
    <arg name="cam2_attachment_frame" default="cam2_color_optical_frame"/>
    <arg name="merged_topic" default="/merged_pointcloud"/>

    <!-- === Camera 1 (D435i - Observer) === -->
    <group ns="cam1">
        <include file="$(find realsense2_camera)/launch/rs_camera.launch">
            <arg name="serial_no"           value="$(arg serial_no_cam1)"/>
            <arg name="camera"              value="cam1"/> <!-- Namespace AND TF prefix -->
            <arg name="enable_pointcloud"   value="true"/>
            <arg name="enable_sync"         value="true"/>
            <arg name="align_depth"         value="true"/>
        </include>
    </group>

    <!-- === Camera 2 (D435 - Target) === -->
    <group ns="cam2">
        <include file="$(find realsense2_camera)/launch/rs_camera.launch">
            <arg name="serial_no"           value="$(arg serial_no_cam2)"/>
            <arg name="camera"              value="cam2"/> <!-- Namespace AND TF prefix -->
            <arg name="enable_pointcloud"   value="true"/>
            <arg name="enable_sync"         value="true"/>
            <arg name="align_depth"         value="true"/>
       </include>
    </group>

    <!-- ArUco Marker Detection Node -->  
    <node name="aruco_single" pkg="aruco_ros" type="single">
        <!-- Input topics from Camera 1 -->
        <remap from="/camera_info" to="/cam1/color/camera_info" />
        <remap from="/image" to="/cam1/color/image_raw" />

        <!-- Parameters -->
        <param name="image_is_rectified" value="true"/>
        <param name="marker_size"        value="$(arg marker_size)"/>
        <param name="marker_id"          value="$(arg marker_id)"/>
        <param name="reference_frame"    value="$(arg cam1_color_optical_frame)"/>
        <param name="camera_frame"       value="$(arg cam1_color_optical_frame)"/>
        <param name="marker_frame"       value="$(arg marker_frame)" />
        <param name="dictionary"         value="$(arg marker_dict)"/>
    </node>

    <!-- ArUco TF Visualizer -->
    <node name="aruco_tf_visualizer" pkg="realsense_aruco_merger" type="aruco_tf_visualizer.py" output="screen">
        <param name="marker_frame"       value="$(arg marker_frame)"/>
        <param name="marker_size"        value="$(arg marker_size)"/>
    </node>
    
    <!-- === Static Transform: Marker -> Camera 2 === -->
    <!-- Publishes the fixed transform from the detected marker frame to Camera 2's attachment frame -->
    <!-- IMPORTANT: You MUST measure and adjust the 'args' values below! -->
    <!-- x y z yaw pitch roll (in meters and radians) -->
    <!-- Example: Marker is 5cm behind cam2's optical frame origin, facing opposite direction (180 deg Pitch rotation) -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="marker_to_cam2_tf"
          args="0 0 -0.05 0 3.14159 0 $(arg marker_frame) $(arg cam2_attachment_frame)" />
          <!-- Adjust X Y Z Roll Pitch Yaw based on how the marker is physically attached to Camera 2 -->
          <!-- X: Forward from marker center to cam2 frame center -->
          <!-- Y: Left from marker center to cam2 frame center -->
          <!-- Z: Up from marker center to cam2 frame center -->
          <!-- Roll: Rotation around X -->
          <!-- Pitch: Rotation around Y -->
          <!-- Yaw: Rotation around Z -->
          <!-- Remember: This defines cam2_attachment_frame's pose *relative* to marker_frame -->

    <!-- Point Cloud Merger Node -->
    <node name="pointcloud_merger" pkg="realsense_aruco_merger" type="pointcloud_merger.py" output="screen">
        <param name="cam1_frame"         value="$(arg cam1_base_frame)"/>
        <param name="cam2_frame"         value="$(arg cam2_base_frame)"/>
        <param name="target_frame"       value="$(arg cam1_base_frame)"/> <!-- Reference frame for merged cloud -->
        <param name="cam1_topic"         value="/cam1/depth/color/points"/>
        <param name="cam2_topic"         value="/cam2/depth/color/points"/>
        <param name="merged_topic"       value="$(arg merged_topic)"/>
    </node>

    <!-- === RViz Visualization === -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find realsense_aruco_merger)/rviz/merge_config.rviz" required="true" />

</launch>
EOL

# Create the RViz configuration directory if it doesn't exist
if [ ! -d /home/monder/catkin_ws/src/realsense_aruco_merger/rviz ]; then
    mkdir -p /home/monder/catkin_ws/src/realsense_aruco_merger/rviz
fi

# Create the RViz configuration file
cat > /home/monder/catkin_ws/src/realsense_aruco_merger/rviz/merge_config.rviz << 'EOL'
Panels:
  - Class: rviz/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /TF1
        - /PointCloud_Cam11
        - /PointCloud_Cam21
        - /Merged_PointCloud1
        - /ArUco Detection Image1
        - /ArUco Marker Viz1
      Splitter Ratio: 0.5
    Tree Height: 531
  - Class: rviz/Selection
    Name: Selection
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.588679
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.3
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud_Cam1
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.01
      Style: Points
      Topic: /cam1/depth/color/points
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud_Cam2
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.01
      Style: Points
      Topic: /cam2/depth/color/points
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 0; 255; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: Merged_PointCloud
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.005
      Style: Points
      Topic: /merged_pointcloud
      Use Fixed Frame: true
      Use rainbow: false
      Value: true
    - Class: rviz/Image
      Enabled: true
      Image Topic: /aruco_single/result
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: ArUco Detection Image
      Normalize Range: true
      Queue Size: 2
      Transport Hint: raw
      Value: true
    - Class: rviz/Marker
      Enabled: true
      Marker Topic: /visualization_marker
      Name: ArUco Marker Viz
      Namespaces:
        {}
      Queue Size: 100
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: cam1_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/SetInitialPose
      Topic: /initialpose
    - Class: rviz/SetGoal
      Topic: /move_base_simple/goal
    - Class: rviz/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 3.5
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0.5
        Y: 0
        Z: 0
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.65
      Target Frame: <Fixed Frame>
      Value: Orbit
      Yaw: 5.5
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 750
  Hide Left Dock: false
  Hide Right Dock: false
  ArUco Detection Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd00000001000000000000021c00000266fc0200000007fb000000100044006900730070006c006100790073010000002800000266000000dd00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007300000001eb0000010c000000d700fffffffb0000001200530065006c0065006300740069006f006e000000010f0000004c0000004a00fffffffb0000000a00560069006500770073000000011d0000010c000000ae00fffffffb0000000c0049006d006100670065010000045e000001640000008900ffffff00000001000001df00000266fc0200000003fb0000000800540069006d00650100000000000001df0000023100fffffffb0000000c004d006f00740069006f006e00730000000000000000000000000000000000010000010f0000004c0000004a00ffffff0000030300000001000000020000000100000002fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Width: 1100
  X: 50
  Y: 25
EOL

echo -e "\n[9/8] Building the packages..."
cd /home/monder/catkin_ws
catkin_make

echo -e "\nAll dependencies have been installed!"
echo -e "\nTo use your RealSense ArUco Point Cloud Merger:"
echo -e "1. First, connect both RealSense cameras."
echo -e "2. Attach the 20mm ArUco marker (ID 0, 4x4 dictionary) to the back of camera 2."
echo -e "3. Position camera 1 so that it can see the ArUco marker attached to camera 2."
echo -e "4. Run the application with: roslaunch realsense_aruco_merger merge_cameras.launch"
echo -e "\nIf you need to adjust the transformation between the marker and camera 2,"
echo -e "edit the static_transform_publisher in the launch file."

echo -e "\nDone! Setup complete."