#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import tf2_ros
import geometry_msgs.msg
import tf.transformations

class ArucoDetectorROS:
    def __init__(self):
        rospy.init_node('aruco_detector_ros', anonymous=True)
        
        # ArUco parameters
        self.MARKER_SIZE = float(rospy.get_param('~marker_size', 0.02))  # 20mm marker size in meters
        self.marker_id = int(rospy.get_param('~marker_id', 0))  # Default to ID 0
        
        # Using 4x4 dictionary as specified
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        
        # Adjust parameters for better detection
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.minMarkerPerimeterRate = 0.02
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.errorCorrectionRate = 0.6
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = rospy.get_param('~camera_frame', 'cam1_color_optical_frame')
        self.marker_frame = rospy.get_param('~marker_frame', 'aruco_marker_frame')
        self.reference_frame = rospy.get_param('~reference_frame', 'cam1_link')
        
        # Get topics from parameters with defaults matching your actual topic structure
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/cam1/cam1/color/camera_info')
        self.image_topic = rospy.get_param('~image_topic', '/cam1/cam1/color/image_raw')
        
        # CV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Publishers
        self.result_pub = rospy.Publisher('aruco_result', Image, queue_size=1)
        self.pose_pub = rospy.Publisher('aruco_pose', PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('aruco_marker', Marker, queue_size=1)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers - using the actual topics from your system
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        rospy.loginfo("ArUco detector ROS node initialized")
        rospy.loginfo(f"Using 4x4 ArUco dictionary, looking for marker ID {self.marker_id}")
        rospy.loginfo(f"Marker size: {self.MARKER_SIZE} meters")
        rospy.loginfo(f"Subscribed to camera info: {self.camera_info_topic}")
        rospy.loginfo(f"Subscribed to image topic: {self.image_topic}")
        
    def camera_info_callback(self, msg):
        """Extract camera intrinsics from camera_info message"""
        if self.camera_matrix is None:
            self.camera_matrix = np.reshape(msg.K, (3, 3))
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera intrinsics received")
            
    def image_callback(self, msg):
        """Process incoming image and detect ArUco markers"""
        if self.camera_matrix is None:
            rospy.logwarn_throttle(5, "Camera intrinsics not available yet")
            return
            
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect markers
            markers_data = self.detect_aruco_marker(cv_image)
            
            # Draw results on image
            result_image = self.display_marker_info(cv_image.copy(), markers_data)
            
            # Convert back to ROS image and publish
            result_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
            result_msg.header = msg.header
            self.result_pub.publish(result_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
            
    def detect_aruco_marker(self, color_image):
        """Detect ArUco markers in the image and return their positions"""
        if color_image is None:
            return None
            
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        markers_data = []
        
        if ids is not None:
            # Filter for the specific marker ID we're looking for
            for i, marker_id in enumerate(ids):
                if marker_id[0] == self.marker_id:  # Only process the marker ID we're interested in
                    marker_corners = corners[i][0]
                    
                    # Define 3D points of marker in marker coordinate system
                    objPoints = np.array([
                        [-self.MARKER_SIZE/2, self.MARKER_SIZE/2, 0],
                        [self.MARKER_SIZE/2, self.MARKER_SIZE/2, 0],
                        [self.MARKER_SIZE/2, -self.MARKER_SIZE/2, 0],
                        [-self.MARKER_SIZE/2, -self.MARKER_SIZE/2, 0]
                    ], dtype=np.float32)
                    
                    # Solve for pose
                    success, rvec, tvec = cv2.solvePnP(
                        objPoints,
                        marker_corners,
                        self.camera_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE
                    )
                    
                    if success:
                        # Calculate marker center in pixels
                        center_x = int(np.mean([corner[0] for corner in marker_corners]))
                        center_y = int(np.mean([corner[1] for corner in marker_corners]))
                        
                        # Get rotation matrix and quaternion
                        R, _ = cv2.Rodrigues(rvec)
                        rot_mat = np.eye(4)
                        rot_mat[:3, :3] = R
                        q = tf.transformations.quaternion_from_matrix(rot_mat)
                        
                        marker_data = {
                            'id': marker_id[0],
                            'pixel_coordinates': (center_x, center_y),
                            'position_3d': (float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])),
                            'rotation_matrix': R,
                            'rvec': rvec,
                            'tvec': tvec,
                            'quaternion': q
                        }
                        
                        markers_data.append(marker_data)
                        
                        # Publish pose
                        self.publish_pose(marker_data)
                        
                        # Publish TF
                        self.publish_tf(marker_data)
                        
                        # Publish marker visualization
                        self.publish_marker_visualization(marker_data)
        
        return markers_data
        
    def publish_pose(self, marker_data):
        """Publish pose of the detected marker"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.camera_frame
        pose_msg.header.stamp = rospy.Time.now()
        
        # Position
        pose_msg.pose.position.x = marker_data['position_3d'][0]
        pose_msg.pose.position.y = marker_data['position_3d'][1]
        pose_msg.pose.position.z = marker_data['position_3d'][2]
        
        # Orientation
        q = marker_data['quaternion']
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        self.pose_pub.publish(pose_msg)
        
    def publish_tf(self, marker_data):
        """Publish TF transform from camera frame to marker frame"""
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = self.marker_frame
        
        # Position
        transform.transform.translation.x = marker_data['position_3d'][0]
        transform.transform.translation.y = marker_data['position_3d'][1]
        transform.transform.translation.z = marker_data['position_3d'][2]
        
        # Orientation
        q = marker_data['quaternion']
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(transform)
        
    def publish_marker_visualization(self, marker_data):
        """Publish RViz marker for visualization"""
        marker = Marker()
        marker.header.frame_id = self.camera_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "aruco_markers"
        marker.id = marker_data['id']
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set marker position
        marker.pose.position.x = marker_data['position_3d'][0]
        marker.pose.position.y = marker_data['position_3d'][1]
        marker.pose.position.z = marker_data['position_3d'][2]
        
        # Set marker orientation
        q = marker_data['quaternion']
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        # Set marker dimensions
        marker.scale.x = self.MARKER_SIZE
        marker.scale.y = self.MARKER_SIZE
        marker.scale.z = 0.001  # Very thin in Z direction
        
        # Set marker color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        
        self.marker_pub.publish(marker)
        
    def display_marker_info(self, color_image, markers_data):
        """Draw detected markers and information on the image"""
        if not markers_data:
            # Add text if no markers detected
            cv2.putText(color_image, "No marker detected", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return color_image
            
        for i, marker in enumerate(markers_data):
            # Get marker ID and position
            marker_id = marker['id']
            center_x, center_y = marker['pixel_coordinates']
            
            # Draw circle at center
            cv2.circle(color_image, (center_x, center_y), 3, (0, 255, 0), -1)
            
            # Draw coordinate axes
            rvec = marker['rvec']
            tvec = marker['tvec']
            cv2.drawFrameAxes(color_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.MARKER_SIZE/1.5)
            
            # Display information
            text_pos_y = 30 + i * 80  # Offset each marker's info
            cv2.putText(color_image, f"ID: {marker_id}", (10, text_pos_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.putText(color_image, f"Pos: ({marker['position_3d'][0]:.3f}, {marker['position_3d'][1]:.3f}, {marker['position_3d'][2]:.3f})m", 
                       (10, text_pos_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            
            distance = np.sqrt(sum([x**2 for x in marker['position_3d']]))
            cv2.putText(color_image, f"Dist: {distance:.3f}m", 
                       (10, text_pos_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        return color_image

def main():
    detector = ArucoDetectorROS()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass