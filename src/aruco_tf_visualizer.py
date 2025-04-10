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
