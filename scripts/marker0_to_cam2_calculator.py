#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import tf.transformations
import yaml
import os
import time
from pathlib import Path
from collections import deque

class Marker0ToCam2Calculator:
    """
    Calculates the transform from ArUco marker ID 0 to Camera 2's optical frame.
    Special configuration: Marker ID 0 is physically mounted on the back of Camera 2.
    Uses ArUco marker ID 1 (seen by both cameras) as a reference bridge.
    Saves the transform to a YAML file.
    """

    def __init__(self):
        rospy.init_node('marker0_to_cam2_calculator', anonymous=True)
        
        # Load parameters
        self.marker0_frame = rospy.get_param('~marker0_frame', 'aruco_marker_0')
        self.marker1_cam1_frame = rospy.get_param('~marker1_cam1_frame', 'aruco_marker_1')
        self.marker1_cam2_frame = rospy.get_param('~marker1_cam2_frame', 'aruco_marker_1')
        self.cam1_optical_frame = rospy.get_param('~cam1_optical_frame', '/cam1/cam1_color_optical_frame')
        self.cam2_optical_frame = rospy.get_param('~cam2_optical_frame', '/cam2/cam2_color_optical_frame')
        self.output_file = rospy.get_param('~output_file', 'marker0_to_cam2_transform.yaml')
        self.save_frequency = rospy.get_param('~save_frequency', 1.0)  # Hz
        self.filter_window_size = rospy.get_param('~filter_window_size', 10)  # Number of transforms to average
        
        # Create output directory if it doesn't exist
        output_dir = os.path.dirname(self.output_file)
        Path(output_dir).mkdir(parents=True, exist_ok=True)
        
        # Initialize TF listener
        self.tf_listener = tf.TransformListener()
        
        # Wait for TF to be available
        rospy.sleep(3.0)  # Give time for TF tree to be populated

        # Buffers for moving average filter
        self.trans_buffer = deque(maxlen=self.filter_window_size)
        self.quat_buffer = deque(maxlen=self.filter_window_size)
        
        # Create timer for calculating and saving transform
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.save_frequency), self.calculate_and_save_transform)
        
        # Keep track of the last successful transform for comparison
        self.last_saved_avg_transform_data = None
        self.transform_count = 0
        
        rospy.loginfo("Marker0 to Cam2 Calculator initialized - BACK MOUNT CONFIGURATION")
        rospy.loginfo(f"Output file: {self.output_file}")
        rospy.loginfo(f"Using moving average filter with window size: {self.filter_window_size}")
        rospy.loginfo(f"Marker 0 frame: {self.marker0_frame} (mounted on back of camera 2)")
        rospy.loginfo(f"Marker 1 cam1 frame: {self.marker1_cam1_frame}")
        rospy.loginfo(f"Marker 1 cam2 frame: {self.marker1_cam2_frame}")
        rospy.loginfo(f"Cam1 optical frame: {self.cam1_optical_frame}")
        rospy.loginfo(f"Cam2 optical frame: {self.cam2_optical_frame}")

    def get_transform(self, parent_frame, child_frame):
        """Get transform between two frames as a 4x4 homogeneous matrix"""
        try:
            self.tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(2))
            (trans, rot) = self.tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            
            # Convert to 4x4 homogeneous transform matrix
            matrix = tf.transformations.quaternion_matrix(rot)
            matrix[0:3, 3] = trans
            
            return matrix, True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Failed to get transform {parent_frame} -> {child_frame}: {e}")
            return np.eye(4), False

    def calculate_and_save_transform(self, event=None):
        """Calculate the marker0 to cam2 transform, apply filter, and save it to a file"""
        try:
            # Check if all markers are visible by attempting to get transforms
            # 1. Cam1 to Marker0 transform
            T_cam1_marker0, found_0 = self.get_transform(self.cam1_optical_frame, self.marker0_frame)
            if not found_0:
                rospy.logwarn_throttle(3.0, f"Marker 0 not detected by Camera 1. Ensure it's visible.")
                return
                
            # 2. Cam1 to Marker1 transform
            T_cam1_marker1, found_1_cam1 = self.get_transform(self.cam1_optical_frame, self.marker1_cam1_frame)
            if not found_1_cam1:
                rospy.logwarn_throttle(3.0, f"Marker 1 not detected by Camera 1. Ensure it's visible.")
                return
                
            # 3. Cam2 to Marker1 transform
            T_cam2_marker1, found_1_cam2 = self.get_transform(self.cam2_optical_frame, self.marker1_cam2_frame)
            if not found_1_cam2:
                rospy.logwarn_throttle(3.0, f"Marker 1 not detected by Camera 2. Ensure it's visible.")
                return
            
            # Calculate transforms
            T_marker0_cam1 = np.linalg.inv(T_cam1_marker0)
            T_marker1_cam1 = np.linalg.inv(T_cam1_marker1)
            T_marker1_cam2 = np.linalg.inv(T_cam2_marker1)
            
            # Compute marker0 to marker1 transform via cam1
            T_marker0_marker1 = T_marker0_cam1 @ T_cam1_marker1
            
            # Calculate marker0 to cam2 transform via marker1
            T_marker0_cam2 = T_marker0_marker1 @ T_marker1_cam2
            
            # Create 180 degree rotation around Y axis to account for marker being on the back
            rot_y_180 = tf.transformations.quaternion_matrix(
                tf.transformations.quaternion_about_axis(np.pi, (0, 1, 0))
            )
            
            # Apply the rotation to our calculated transform
            T_marker0_cam2_corrected = T_marker0_cam2 @ rot_y_180
            
            # --- Moving Average Filter --- 
            # Extract raw translation and quaternion from this calculation
            raw_trans = T_marker0_cam2_corrected[0:3, 3]
            raw_quat = tf.transformations.quaternion_from_matrix(T_marker0_cam2_corrected)

            # Add to buffers
            self.trans_buffer.append(raw_trans)
            self.quat_buffer.append(raw_quat)

            # Only proceed if buffer is full
            if len(self.trans_buffer) < self.filter_window_size:
                rospy.loginfo_throttle(5.0, f"Filling filter buffer ({len(self.trans_buffer)}/{self.filter_window_size})...")
                return

            # Calculate average translation
            avg_trans = np.mean(np.array(self.trans_buffer), axis=0)

            # Calculate average quaternion (component average + normalization)
            avg_quat_raw = np.mean(np.array(self.quat_buffer), axis=0)
            avg_quat_norm = np.linalg.norm(avg_quat_raw)
            if avg_quat_norm > 1e-6:
                avg_quat = avg_quat_raw / avg_quat_norm
            else:
                avg_quat = np.array([0.0, 0.0, 0.0, 1.0])  # Default identity
            # --- End Filter --- 

            # Use averaged values for output
            trans_list = avg_trans.tolist()
            quat_list = avg_quat.tolist()
            
            # Calculate distance and Euler angles from averaged values
            distance = np.sqrt(np.sum(avg_trans**2))
            euler_rad = tf.transformations.euler_from_quaternion(avg_quat)
            euler_deg = [np.degrees(angle) for angle in euler_rad]
            
            # Create dictionary for YAML output using averaged data
            current_avg_transform_data = {
                'parent_frame': self.marker0_frame,
                'child_frame': self.cam2_optical_frame,
                'transform': {
                    'translation': {'x': trans_list[0], 'y': trans_list[1], 'z': trans_list[2]},
                    'rotation': {'x': quat_list[0], 'y': quat_list[1], 'z': quat_list[2], 'w': quat_list[3]}
                },
                'euler_degrees': {'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw': euler_deg[2]},
                'distance_meters': distance,
                'timestamp': time.time(),
                'timestamp_ros': rospy.Time.now().to_sec(),
                'note': f'Averaged over {self.filter_window_size} samples. Calculated with marker0 on back of camera2, Y-axis rotation applied'
            }

            # Check if the *averaged* transform has changed significantly since last save
            if self.last_saved_avg_transform_data is not None:
                last_avg_trans_array = np.array([
                    self.last_saved_avg_transform_data['transform']['translation']['x'],
                    self.last_saved_avg_transform_data['transform']['translation']['y'],
                    self.last_saved_avg_transform_data['transform']['translation']['z']
                ])
                trans_diff = np.linalg.norm(avg_trans - last_avg_trans_array)
                
                # More lenient threshold for averaged data
                if trans_diff < 0.0005:  # Less than 0.5mm difference in average
                    rospy.logdebug("Averaged transform hasn't changed significantly. Skipping save.")
                    return
            
            # Save the averaged transform to YAML file
            try:
                with open(self.output_file, 'w') as f:
                    yaml.dump(current_avg_transform_data, f, default_flow_style=False)
                
                # Keep track of this saved average transform
                self.last_saved_avg_transform_data = current_avg_transform_data
                self.transform_count += 1
                
                # Log details periodically
                if self.transform_count % 5 == 0:  # Every 5 saves
                    rospy.loginfo("=== Averaged Transform (%s to %s) [BACK MOUNT] ===", self.marker0_frame, self.cam2_optical_frame)
                    rospy.loginfo("Avg Translation [x, y, z]: [%.6f, %.6f, %.6f] meters", trans_list[0], trans_list[1], trans_list[2])
                    rospy.loginfo("Avg Rotation [x, y, z, w]: [%.6f, %.6f, %.6f, %.6f]", quat_list[0], quat_list[1], quat_list[2], quat_list[3])
                    rospy.loginfo("Avg Euler angles [roll, pitch, yaw]: [%.2f, %.2f, %.2f] degrees", euler_deg[0], euler_deg[1], euler_deg[2])
                    rospy.loginfo("Avg Distance: %.6f meters", distance)
                    rospy.loginfo("Saved averaged transform to %s", self.output_file)
                else:
                    rospy.loginfo("Averaged transform saved #%d", self.transform_count)
                    
            except Exception as e:
                rospy.logerr("Failed to save averaged transform to %s: %s", self.output_file, str(e))
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # Clear buffer if we fail to get TF, to avoid stale data when it recovers
            self.trans_buffer.clear()
            self.quat_buffer.clear()
            rospy.logwarn_throttle(5.0, f"TF Exception, clearing filter buffer: {e}")
        except Exception as e:
            rospy.logerr("Error calculating/filtering transform: %s", str(e))
            import traceback
            traceback.print_exc()

def main():
    try:
        calculator = Marker0ToCam2Calculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()