#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import tf2_ros
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import tf.transformations

class DualCameraArucoDetector:
    def __init__(self):
        rospy.init_node('dual_camera_aruco_detector', anonymous=True)
        
        # ArUco parameters
        self.MARKER_SIZE = float(rospy.get_param('~marker_size', 0.02))  # 20mm marker size in meters
        self.marker_id = int(rospy.get_param('~marker_id', 1))  # Using ID 1 as requested
        
        # Using 4x4 dictionary as requested
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
        self.cam1_matrix = None
        self.cam1_dist_coeffs = None
        self.cam2_matrix = None
        self.cam2_dist_coeffs = None
        
        # Frame IDs
        self.cam1_frame = rospy.get_param('~cam1_frame', 'cam1_color_optical_frame')
        self.cam2_frame = rospy.get_param('~cam2_frame', 'cam2_color_optical_frame')
        self.marker_frame_cam1 = rospy.get_param('~marker_frame_cam1', 'aruco_marker_cam1')
        self.marker_frame_cam2 = rospy.get_param('~marker_frame_cam2', 'aruco_marker_cam2')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        
        # Topic names (with defaults matching your system's structure)
        self.cam1_info_topic = rospy.get_param('~cam1_info_topic', '/cam1/cam1/color/camera_info')
        self.cam2_info_topic = rospy.get_param('~cam2_info_topic', '/cam2/cam2/color/camera_info')
        self.cam1_image_topic = rospy.get_param('~cam1_image_topic', '/cam1/cam1/color/image_raw')
        self.cam2_image_topic = rospy.get_param('~cam2_image_topic', '/cam2/cam2/color/image_raw')
        
        # CV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Publishers for visualization and debug
        self.cam1_result_pub = rospy.Publisher('cam1/aruco_result', Image, queue_size=1)
        self.cam2_result_pub = rospy.Publisher('cam2/aruco_result', Image, queue_size=1)
        self.cam1_marker_pub = rospy.Publisher('cam1/aruco_marker', Marker, queue_size=1)
        self.cam2_marker_pub = rospy.Publisher('cam2/aruco_marker', Marker, queue_size=1)
        
        # For camera calibration
        self.camera_transformation_pub = rospy.Publisher('camera_transformation', TransformStamped, queue_size=1)
        
        # Camera info subscribers
        rospy.Subscriber(self.cam1_info_topic, CameraInfo, self.cam1_info_callback)
        rospy.Subscriber(self.cam2_info_topic, CameraInfo, self.cam2_info_callback)
        
        # Flag to track if we have both camera intrinsics
        self.intrinsics_ready = False
        
        rospy.loginfo("Dual camera ArUco detector initialized")
        rospy.loginfo(f"Using 4x4 ArUco dictionary, looking for marker ID {self.marker_id}")
        rospy.loginfo(f"Marker size: {self.MARKER_SIZE} meters")
        
    def cam1_info_callback(self, msg):
        """Extract camera intrinsics from camera_info message for camera 1"""
        if self.cam1_matrix is None:
            self.cam1_matrix = np.reshape(msg.K, (3, 3))
            self.cam1_dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera 1 intrinsics received")
            self._check_start_detection()
            
    def cam2_info_callback(self, msg):
        """Extract camera intrinsics from camera_info message for camera 2"""
        if self.cam2_matrix is None:
            self.cam2_matrix = np.reshape(msg.K, (3, 3))
            self.cam2_dist_coeffs = np.array(msg.D)
            rospy.loginfo("Camera 2 intrinsics received")
            self._check_start_detection()
    
    def _check_start_detection(self):
        """Check if we have both camera intrinsics, if so start the detection"""
        if self.cam1_matrix is not None and self.cam2_matrix is not None and not self.intrinsics_ready:
            self.intrinsics_ready = True
            rospy.loginfo("Both camera intrinsics received, starting detection")
            
            # Set up the synchronized subscribers for the two camera feeds
            cam1_sub = message_filters.Subscriber(self.cam1_image_topic, Image)
            cam2_sub = message_filters.Subscriber(self.cam2_image_topic, Image)
            
            # Synchronize the two camera feeds with a time tolerance
            ts = message_filters.ApproximateTimeSynchronizer([cam1_sub, cam2_sub], 10, 0.1)
            ts.registerCallback(self.images_callback)
            
    def images_callback(self, cam1_msg, cam2_msg):
        """Process incoming images from both cameras and detect ArUco markers"""
        try:
            # Convert ROS images to OpenCV format
            cv_image1 = self.bridge.imgmsg_to_cv2(cam1_msg, "bgr8")
            cv_image2 = self.bridge.imgmsg_to_cv2(cam2_msg, "bgr8")
            
            # Detect markers in both images
            markers_data1 = self.detect_aruco_marker(cv_image1, self.cam1_matrix, self.cam1_dist_coeffs, 
                                                   self.cam1_frame, self.marker_frame_cam1, True)
            markers_data2 = self.detect_aruco_marker(cv_image2, self.cam2_matrix, self.cam2_dist_coeffs, 
                                                   self.cam2_frame, self.marker_frame_cam2, False)
            
            # Calculate and publish the transformation between cameras via the marker
            self.calculate_camera_transformation(markers_data1, markers_data2)
            
            # Draw results on images for visualization
            result_image1 = self.display_marker_info(cv_image1.copy(), markers_data1, self.cam1_matrix, self.cam1_dist_coeffs)
            result_image2 = self.display_marker_info(cv_image2.copy(), markers_data2, self.cam2_matrix, self.cam2_dist_coeffs)
            
            # Convert back to ROS images and publish
            result_msg1 = self.bridge.cv2_to_imgmsg(result_image1, "bgr8")
            result_msg1.header = cam1_msg.header
            self.cam1_result_pub.publish(result_msg1)
            
            result_msg2 = self.bridge.cv2_to_imgmsg(result_image2, "bgr8")
            result_msg2.header = cam2_msg.header
            self.cam2_result_pub.publish(result_msg2)
            
        except Exception as e:
            rospy.logerr(f"Error processing images: {e}")
            
    def detect_aruco_marker(self, color_image, camera_matrix, dist_coeffs, camera_frame, marker_frame, is_cam1):
        """Detect ArUco markers in the image and return their positions"""
        if color_image is None:
            return None
            
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        markers_data = []
        
        if ids is not None:
            # Filter for the specific marker ID we're looking for
            for i, marker_id in enumerate(ids):
                if marker_id[0] == self.marker_id:
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
                        camera_matrix,
                        dist_coeffs,
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
                        
                        # Publish TF
                        self.publish_tf(marker_data, camera_frame, marker_frame)
                        
                        # Publish marker visualization
                        if is_cam1:
                            self.publish_marker_visualization(marker_data, camera_frame, self.cam1_marker_pub)
                        else:
                            self.publish_marker_visualization(marker_data, camera_frame, self.cam2_marker_pub)
        
        return markers_data
    
    def publish_tf(self, marker_data, camera_frame, marker_frame):
        """Publish TF transform from camera to marker"""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = camera_frame
        transform.child_frame_id = marker_frame
        
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
        
    def publish_marker_visualization(self, marker_data, camera_frame, publisher):
        """Publish RViz marker for visualization"""
        marker = Marker()
        marker.header.frame_id = camera_frame
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
        
        publisher.publish(marker)
    
    def calculate_camera_transformation(self, markers_data1, markers_data2):
        """Calculate the transformation between cameras using the marker as a common reference"""
        # Check if we have detected the marker in both cameras
        if not markers_data1 or not markers_data2:
            return
            
        marker1 = markers_data1[0]  # Only use the first detected marker
        marker2 = markers_data2[0]  # Only use the first detected marker
        
        try:
            # Get the transformation from camera1 to marker
            t1 = np.array(marker1['position_3d'])
            R1 = marker1['rotation_matrix']
            
            # Get the transformation from camera2 to marker
            t2 = np.array(marker2['position_3d'])
            R2 = marker2['rotation_matrix']
            
            # Calculate the transformation from camera1 to camera2
            # R2^-1 * R1 is the rotation from camera1 to camera2
            # -R2^-1 * t2 + R2^-1 * R1 * t1 is the translation from camera1 to camera2
            R2_inv = np.linalg.inv(R2)
            R_cam1_to_cam2 = np.matmul(R2_inv, R1)
            t_cam1_to_cam2 = np.matmul(-R2_inv, t2) + np.matmul(np.matmul(R2_inv, R1), t1)
            
            # Convert rotation matrix to quaternion
            rot_mat = np.eye(4)
            rot_mat[:3, :3] = R_cam1_to_cam2
            q = tf.transformations.quaternion_from_matrix(rot_mat)
            
            # Publish the transformation as a TF
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = self.cam1_frame
            transform.child_frame_id = self.cam2_frame
            
            # Apply the calculated transformation
            transform.transform.translation.x = float(t_cam1_to_cam2[0])
            transform.transform.translation.y = float(t_cam1_to_cam2[1])
            transform.transform.translation.z = float(t_cam1_to_cam2[2])
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            
            # Publish the transformation
            self.tf_broadcaster.sendTransform(transform)
            self.camera_transformation_pub.publish(transform)
            
            # Also publish a world frame with origin at camera1
            world_transform = TransformStamped()
            world_transform.header.stamp = rospy.Time.now()
            world_transform.header.frame_id = self.world_frame
            world_transform.child_frame_id = self.cam1_frame
            world_transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(world_transform)
            
        except Exception as e:
            rospy.logerr(f"Error calculating camera transformation: {e}")
        
    def display_marker_info(self, color_image, markers_data, camera_matrix, dist_coeffs):
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
            cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvec, tvec, self.MARKER_SIZE/1.5)
            
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
    detector = DualCameraArucoDetector()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass