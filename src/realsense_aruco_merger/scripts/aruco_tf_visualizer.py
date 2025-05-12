#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import tf2_ros
from aruco_msgs.msg import MarkerArray

class ArucoTFVisualizer:
    def __init__(self):
        rospy.init_node('aruco_tf_visualizer', anonymous=True)
        
        # Get parameters
        self.camera1_ns = rospy.get_param('~camera1_ns', 'cam1')
        self.camera2_ns = rospy.get_param('~camera2_ns', 'cam2')
        self.marker_id = rospy.get_param('~marker_id', 0)
        
        # Set up frames
        self.cam1_frame = rospy.get_param('~cam1_frame', 'cam1_color_optical_frame')
        self.cam2_frame = rospy.get_param('~cam2_frame', 'cam2_color_optical_frame')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Store the latest transforms
        self.latest_cam1_transform = None
        self.latest_cam2_transform = None
        self.world_to_cam1_transform = None
        self.world_to_cam2_transform = None
        
        # Create the static transform to world frame if needed
        self.publish_static_world_transform()
        
        # Subscribe to ArUco markers
        rospy.Subscriber(f"/{self.camera1_ns}/aruco_marker_publisher/markers", MarkerArray, self.cam1_marker_callback)
        rospy.Subscriber(f"/{self.camera2_ns}/aruco_marker_publisher/markers", MarkerArray, self.cam2_marker_callback)
        
        # Timer to update transforms
        self.update_timer = rospy.Timer(rospy.Duration(0.05), self.update_transforms)
        
        rospy.loginfo("ArUco TF Visualizer initialized")
        rospy.loginfo(f"Tracking marker ID: {self.marker_id}")
        rospy.loginfo(f"Camera 1 namespace: {self.camera1_ns}, frame: {self.cam1_frame}")
        rospy.loginfo(f"Camera 2 namespace: {self.camera2_ns}, frame: {self.cam2_frame}")
        rospy.loginfo(f"World frame: {self.world_frame}")
        
    def publish_static_world_transform(self):
        """Publish a static transform for the world frame"""
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "map"  # Parent frame
        static_transform.child_frame_id = self.world_frame
        
        # Identity transform
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(static_transform)
        rospy.loginfo(f"Published static transform from map to {self.world_frame}")
        
    def cam1_marker_callback(self, msg):
        """Process ArUco markers from camera 1"""
        for marker in msg.markers:
            if marker.id == self.marker_id:
                # Store the transform from camera to marker
                pos = marker.pose.pose.position
                ori = marker.pose.pose.orientation
                
                # Using the marker as reference frame (inverse transform)
                self.latest_cam1_transform = (
                    (-pos.x, -pos.y, -pos.z),
                    (ori.x, ori.y, ori.z, ori.w)
                )
                rospy.loginfo_throttle(2.0, f"Received marker {self.marker_id} from camera 1")
                break
                
    def cam2_marker_callback(self, msg):
        """Process ArUco markers from camera 2"""
        for marker in msg.markers:
            if marker.id == self.marker_id:
                # Store the transform from camera to marker
                pos = marker.pose.pose.position
                ori = marker.pose.pose.orientation
                
                # Using the marker as reference frame (inverse transform)
                self.latest_cam2_transform = (
                    (-pos.x, -pos.y, -pos.z),
                    (ori.x, ori.y, ori.z, ori.w)
                )
                rospy.loginfo_throttle(2.0, f"Received marker {self.marker_id} from camera 2")
                break
    
    def update_transforms(self, event):
        """Update and publish transforms based on marker detections"""
        stamp = rospy.Time.now()
        
        # Define a common marker frame in world coordinates that both cameras can reference
        marker_in_world = (
            (0.0, 0.0, 0.0),  # Position at origin of world
            (0.0, 0.0, 0.0, 1.0)  # Identity orientation
        )
        
        # Publish marker->cam1 transform if available
        if self.latest_cam1_transform:
            marker_pos, marker_ori = self.latest_cam1_transform
            
            # Publish transform from marker to camera frame
            self.tf_broadcaster.sendTransform(
                marker_pos,
                marker_ori,
                stamp,
                f"aruco_marker_{self.camera1_ns}",
                self.cam1_frame
            )
            
            # Publish the common marker in world coordinates
            marker_world_pos, marker_world_ori = marker_in_world
            self.tf_broadcaster.sendTransform(
                marker_world_pos,
                marker_world_ori,
                stamp,
                "aruco_marker_world",
                self.world_frame
            )
            
            # Calculate and publish transform from world to camera 1
            # This places cam1 relative to the world using the marker as reference
            self.tf_broadcaster.sendTransform(
                marker_pos,  # Using the same position as marker->cam1
                marker_ori,  # Using the same orientation as marker->cam1
                stamp,
                self.cam1_frame,
                self.world_frame
            )
            
        # Publish marker->cam2 transform if available
        if self.latest_cam2_transform:
            marker_pos, marker_ori = self.latest_cam2_transform
            
            # Publish transform from marker to camera frame
            self.tf_broadcaster.sendTransform(
                marker_pos,
                marker_ori,
                stamp,
                f"aruco_marker_{self.camera2_ns}",
                self.cam2_frame
            )
            
            # Calculate and publish transform from world to camera 2
            # This places cam2 relative to the world using the marker as reference
            self.tf_broadcaster.sendTransform(
                marker_pos,  # Using the same position as marker->cam2
                marker_ori,  # Using the same orientation as marker->cam2
                stamp,
                self.cam2_frame,
                self.world_frame
            )
                
        # If both cameras see the marker, log that the common reference is established
        if self.latest_cam1_transform and self.latest_cam2_transform:
            rospy.loginfo_throttle(2.0, "Both cameras are seeing the ArUco marker - transforms connected through world frame")

def main():
    visualizer = ArucoTFVisualizer()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass