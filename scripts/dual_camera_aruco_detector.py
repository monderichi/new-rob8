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
import math
import collections


# Add filter classes for stabilizing ArUco marker detection
class PoseFilter:
    """Base class for pose filtering"""
    def __init__(self):
        pass
    
    def filter(self, position, quaternion):
        """Filter the pose and return filtered position and quaternion"""
        return position, quaternion


class MovingAverageFilter(PoseFilter):
    """Moving average filter for pose stabilization"""
    def __init__(self, window_size=5):
        super(MovingAverageFilter, self).__init__()
        self.window_size = window_size
        self.position_buffer = collections.deque(maxlen=window_size)
        self.quaternion_buffer = collections.deque(maxlen=window_size)
    
    def filter(self, position, quaternion):
        """Apply moving average filter to position and quaternion"""
        if position is None or quaternion is None:
            return None, None
            
        # Add current measurements to buffer
        self.position_buffer.append(np.array(position))
        self.quaternion_buffer.append(np.array(quaternion))
        
        # Calculate average position
        if len(self.position_buffer) > 0:
            avg_position = np.mean(np.array(self.position_buffer), axis=0)
            
            # For quaternions, we need to handle them differently due to their non-linearity
            # Using the approach of averaging the rotation matrices and converting back to quaternion
            if len(self.quaternion_buffer) > 0:
                rotation_matrices = []
                for q in self.quaternion_buffer:
                    rotation_matrices.append(tf.transformations.quaternion_matrix(q)[:3, :3])
                
                avg_rotation_matrix = np.mean(np.array(rotation_matrices), axis=0)
                
                # Ensure the result is a valid rotation matrix
                u, s, vh = np.linalg.svd(avg_rotation_matrix)
                avg_rotation_matrix = u @ vh
                
                # Convert back to quaternion
                rot_mat_4x4 = np.eye(4)
                rot_mat_4x4[:3, :3] = avg_rotation_matrix
                avg_quaternion = tf.transformations.quaternion_from_matrix(rot_mat_4x4)
                
                return avg_position, avg_quaternion
                
        return position, quaternion
        

class OutlierRejectionFilter(PoseFilter):
    """Filter that rejects outlier poses based on distance threshold"""
    def __init__(self, position_threshold=0.05, rotation_threshold=0.2, history_size=10):
        super(OutlierRejectionFilter, self).__init__()
        self.position_threshold = position_threshold  # meters
        self.rotation_threshold = rotation_threshold  # radians
        self.history_size = history_size
        self.position_history = collections.deque(maxlen=history_size)
        self.quaternion_history = collections.deque(maxlen=history_size)
        self.last_valid_position = None
        self.last_valid_quaternion = None
    
    def filter(self, position, quaternion):
        """Reject outlier poses based on distance from median"""
        if position is None or quaternion is None:
            return self.last_valid_position, self.last_valid_quaternion
            
        # If we don't have enough history, accept the measurement
        if len(self.position_history) < 3:
            self.position_history.append(np.array(position))
            self.quaternion_history.append(np.array(quaternion))
            self.last_valid_position = position
            self.last_valid_quaternion = quaternion
            return position, quaternion
        
        # Calculate median position from history
        median_position = np.median(np.array(self.position_history), axis=0)
        
        # Calculate distance from current position to median
        distance = np.linalg.norm(np.array(position) - median_position)
        
        # For rotation, calculate the angle between current quaternion and previous quaternions
        rotation_distances = []
        for q in self.quaternion_history:
            # Calculate the angle between quaternions
            dot_product = np.abs(np.sum(q * quaternion))
            # Clamp to valid range to avoid numerical errors
            dot_product = min(max(dot_product, -1.0), 1.0)
            angle = 2 * np.arccos(dot_product)
            rotation_distances.append(angle)
        
        median_rotation_distance = np.median(rotation_distances)
        
        # Accept measurement if within thresholds
        if distance <= self.position_threshold and median_rotation_distance <= self.rotation_threshold:
            self.position_history.append(np.array(position))
            self.quaternion_history.append(np.array(quaternion))
            self.last_valid_position = position
            self.last_valid_quaternion = quaternion
            return position, quaternion
        else:
            rospy.logdebug(f"Rejected outlier: position_dist={distance:.4f}m, rotation_dist={median_rotation_distance:.4f}rad")
            return self.last_valid_position, self.last_valid_quaternion


class LowPassFilter(PoseFilter):
    """Low-pass filter for smoothing pose data"""
    def __init__(self, alpha=0.3):
        super(LowPassFilter, self).__init__()
        self.alpha = alpha  # Filter coefficient (0-1): lower values = more smoothing
        self.prev_position = None
        self.prev_quaternion = None
    
    def filter(self, position, quaternion):
        """Apply low-pass filter: output = alpha * input + (1 - alpha) * prev_output"""
        if position is None or quaternion is None:
            return self.prev_position, self.prev_quaternion
            
        # Initialize if this is the first measurement
        if self.prev_position is None:
            self.prev_position = np.array(position)
            self.prev_quaternion = np.array(quaternion)
            return position, quaternion
        
        # Filter position with exponential smoothing
        filtered_position = self.alpha * np.array(position) + (1 - self.alpha) * self.prev_position
        self.prev_position = filtered_position
        
        # For orientation, we need to handle quaternions differently
        # Use spherical linear interpolation (SLERP)
        t = self.alpha  # Use alpha as the interpolation parameter
        filtered_quaternion = tf.transformations.quaternion_slerp(
            self.prev_quaternion, quaternion, t
        )
        self.prev_quaternion = filtered_quaternion
        
        return filtered_position, filtered_quaternion


class CascadeFilter(PoseFilter):
    """Combines multiple filters in a cascade"""
    def __init__(self, filters):
        super(CascadeFilter, self).__init__()
        self.filters = filters
    
    def filter(self, position, quaternion):
        """Apply all filters in sequence"""
        current_position, current_quaternion = position, quaternion
        
        for filter_instance in self.filters:
            current_position, current_quaternion = filter_instance.filter(current_position, current_quaternion)
            
        return current_position, current_quaternion


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
        
        # Initialize pose filters for both cameras
        # Get filter parameters from ROS parameters
        filter_mode = rospy.get_param('~filter_mode', 'cascade')  # 'cascade', 'moving_avg', 'outlier', 'lowpass'
        ma_window_size = rospy.get_param('~ma_window_size', 5)
        outlier_position_threshold = rospy.get_param('~outlier_position_threshold', 0.05)
        outlier_rotation_threshold = rospy.get_param('~outlier_rotation_threshold', 0.2)
        lowpass_alpha = rospy.get_param('~lowpass_alpha', 0.3)
        
        # Create the filters based on the mode
        if filter_mode == 'cascade':
            # Cascade filter: Outlier rejection -> Moving average -> Low-pass
            self.cam1_filter = CascadeFilter([
                OutlierRejectionFilter(outlier_position_threshold, outlier_rotation_threshold),
                MovingAverageFilter(ma_window_size),
                LowPassFilter(lowpass_alpha)
            ])
            self.cam2_filter = CascadeFilter([
                OutlierRejectionFilter(outlier_position_threshold, outlier_rotation_threshold),
                MovingAverageFilter(ma_window_size),
                LowPassFilter(lowpass_alpha)
            ])
        elif filter_mode == 'moving_avg':
            self.cam1_filter = MovingAverageFilter(ma_window_size)
            self.cam2_filter = MovingAverageFilter(ma_window_size)
        elif filter_mode == 'outlier':
            self.cam1_filter = OutlierRejectionFilter(outlier_position_threshold, outlier_rotation_threshold)
            self.cam2_filter = OutlierRejectionFilter(outlier_position_threshold, outlier_rotation_threshold)
        elif filter_mode == 'lowpass':
            self.cam1_filter = LowPassFilter(lowpass_alpha)
            self.cam2_filter = LowPassFilter(lowpass_alpha)
        else:
            # Default to no filtering
            self.cam1_filter = PoseFilter()
            self.cam2_filter = PoseFilter()
        
        rospy.loginfo("Dual camera ArUco detector initialized")
        rospy.loginfo(f"Using filter mode: {filter_mode}")
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
                        
                        # Apply filtering to stabilize pose
                        if is_cam1:
                            filtered_position, filtered_quaternion = self.cam1_filter.filter((float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])), q)
                        else:
                            filtered_position, filtered_quaternion = self.cam2_filter.filter((float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])), q)
                        
                        marker_data = {
                            'id': marker_id[0],
                            'pixel_coordinates': (center_x, center_y),
                            'position_3d': filtered_position,
                            'rotation_matrix': R,
                            'rvec': rvec,
                            'tvec': tvec,
                            'quaternion': filtered_quaternion
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
            # Get the transformation from marker frame to camera1 frame
            t1 = np.array(marker1['position_3d'])
            R1 = marker1['rotation_matrix']
            T1 = np.eye(4)  # Homogeneous transformation matrix: marker -> cam1
            T1[:3, :3] = R1
            T1[:3, 3] = t1
            
            # Get the transformation from marker frame to camera2 frame
            t2 = np.array(marker2['position_3d'])
            R2 = marker2['rotation_matrix']
            T2 = np.eye(4)  # Homogeneous transformation matrix: marker -> cam2
            T2[:3, :3] = R2
            T2[:3, 3] = t2
            
            # Calculate the transformation from camera2 frame to camera1 frame
            T2_inv = np.linalg.inv(T2) # Transform: cam2 -> marker
            T_cam1_to_cam2 = np.matmul(T1, T2_inv) # Transform: cam2 -> cam1
            
            # Extract rotation matrix and translation vector for cam1 <- cam2
            R_cam1_to_cam2 = T_cam1_to_cam2[:3, :3]
            t_cam1_to_cam2 = T_cam1_to_cam2[:3, 3]
            
            # Convert rotation matrix to quaternion
            rot_mat_4x4 = np.eye(4)
            rot_mat_4x4[:3, :3] = R_cam1_to_cam2
            q = tf.transformations.quaternion_from_matrix(rot_mat_4x4)
            
            # Calculate and log Euler angles for easier understanding of orientation
            euler_angles = tf.transformations.euler_from_matrix(rot_mat_4x4, 'sxyz')
            euler_degrees = [math.degrees(angle) for angle in euler_angles]
            rospy.loginfo_throttle(1.0, f"Calculated TFs: Cam1 <- Cam2 Euler (deg): [{euler_degrees[0]:.2f}, {euler_degrees[1]:.2f}, {euler_degrees[2]:.2f}]")
            
            # Publish the transformation as a TF: cam1_frame -> cam2_frame
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now() # Use current time for dynamic transform
            transform.header.frame_id = self.cam1_frame # Parent frame
            transform.child_frame_id = self.cam2_frame  # Child frame
            
            # Apply the calculated transformation T_cam1_to_cam2
            transform.transform.translation.x = float(t_cam1_to_cam2[0])
            transform.transform.translation.y = float(t_cam1_to_cam2[1])
            transform.transform.translation.z = float(t_cam1_to_cam2[2])
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            
            # Publish the dynamic transformation
            self.tf_broadcaster.sendTransform(transform)
            self.camera_transformation_pub.publish(transform) 
            
            # Publish a static identity transform to define world = cam1_frame
            world_transform = TransformStamped()
            world_transform.header.stamp = rospy.Time.now() # Stamp needed even for static
            world_transform.header.frame_id = self.cam1_frame
            world_transform.child_frame_id = self.world_frame
            world_transform.transform.rotation.w = 1.0 # Identity rotation
            self.tf_broadcaster.sendTransform(world_transform) 

            # Log transformation details for debugging
            t_x, t_y, t_z = float(t_cam1_to_cam2[0]), float(t_cam1_to_cam2[1]), float(t_cam1_to_cam2[2])
            distance = np.sqrt(t_x**2 + t_y**2 + t_z**2)
            rospy.loginfo_throttle(5.0, f"Published TF: {self.cam1_frame} -> {self.cam2_frame}, translation=[{t_x:.4f}, {t_y:.4f}, {t_z:.4f}]m, distance={distance:.4f}m")
            
        except np.linalg.LinAlgError as e:
            rospy.logerr_throttle(5.0, f"Error calculating camera transformation: Matrix inversion failed. {e}")
        except Exception as e:
            rospy.logerr(f"Error calculating camera transformation: {e}")
            import traceback
            traceback.print_exc()
        
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