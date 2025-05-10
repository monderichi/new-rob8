#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import tf.transformations
import yaml # To save output as YAML
import os

try:
    import yaml # Check import early
except ImportError:
    rospy.logfatal("PyYAML library not found. Please install it (e.g., 'sudo apt install python3-yaml' or 'pip install PyYAML').")
    exit()

class CalibrationCalculator:
    """
    Calculates the static transform from aruco_marker_0 (seen by cam1)
    to cam2_color_optical_frame using aruco_marker_1 (seen by both cams)
    as a bridge. Saves the result to a YAML file.
    """

    def __init__(self):
        rospy.init_node('calibration_calculator', anonymous=True)

        # --- Parameters ---
        self.marker0_frame = rospy.get_param('~marker0_frame', 'aruco_marker_0')
        # Construct marker 1 frame names based on convention and ID
        marker1_id = rospy.get_param('~marker1_id', 1) # Get the ID used for marker 1
        self.marker1_cam1_frame = rospy.get_param('~marker1_cam1_frame', f'aruco_marker_1_cam1_{marker1_id}')
        self.marker1_cam2_frame = rospy.get_param('~marker1_cam2_frame', f'aruco_marker_1_cam2_{marker1_id}')
        self.cam1_optical_frame = rospy.get_param('~cam1_optical_frame', '/cam1/cam1_color_optical_frame')
        self.cam2_optical_frame = rospy.get_param('~cam2_optical_frame', '/cam2/cam2_color_optical_frame')
        self.output_file = rospy.get_param('~output_file', os.path.join(os.path.expanduser("~"), "catkin_ws", "marker0_to_cam2_transform.yaml"))

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Ensure output directory exists
        output_dir = os.path.dirname(self.output_file)
        if not os.path.exists(output_dir):
            try:
                os.makedirs(output_dir)
                rospy.loginfo(f"Created output directory: {output_dir}")
            except OSError as e:
                rospy.logerr(f"Failed to create output directory {output_dir}: {e}")
                return # Cannot proceed without output directory

        rospy.loginfo("Calibration Calculator Initialized.")
        rospy.loginfo(f"  Cam1 Optical Frame: {self.cam1_optical_frame}")
        rospy.loginfo(f"  Cam2 Optical Frame: {self.cam2_optical_frame}")
        rospy.loginfo(f"  Marker 0 Frame (Cam1): {self.marker0_frame}")
        rospy.loginfo(f"  Marker 1 Frame (Cam1): {self.marker1_cam1_frame}")
        rospy.loginfo(f"  Marker 1 Frame (Cam2): {self.marker1_cam2_frame}")
        rospy.loginfo(f"  Output File: {self.output_file}")
        rospy.loginfo("Waiting for required TF transforms...")

        # Timer for periodic calculation attempts
        self.timer = rospy.Timer(rospy.Duration(1.0), self.calculate_and_save_transform)
        self.last_successful_transform = None

    def get_transform_as_matrix(self, target_frame, source_frame):
        """Try to get a transform and convert to a 4x4 homogeneous matrix"""
        try:
            self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.5))
            (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            matrix = tf.transformations.quaternion_matrix(rot)
            matrix[0:3, 3] = trans
            rospy.logdebug(f"Successfully got transform: {source_frame} -> {target_frame}")
            return matrix, True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"TF Error getting {source_frame} -> {target_frame}: {e}. Ensure markers are visible and detectors running.")
            return np.eye(4), False

    def matrix_to_transform_dict(self, matrix, parent_frame, child_frame):
        """Convert a 4x4 matrix to a dictionary suitable for YAML output"""
        trans = matrix[0:3, 3].tolist() # Convert numpy array to list
        rot = tf.transformations.quaternion_from_matrix(matrix).tolist() # Convert numpy array to list
        euler_deg = [np.degrees(e) for e in tf.transformations.euler_from_quaternion(rot)]
        distance = float(np.sqrt(np.sum(np.array(trans)**2))) # Ensure float

        return {
            'parent_frame': parent_frame,
            'child_frame': child_frame,
            'transform': {
                'translation': {'x': trans[0], 'y': trans[1], 'z': trans[2]},
                'rotation': {'x': rot[0], 'y': rot[1], 'z': rot[2], 'w': rot[3]},
            },
            'euler_degrees': {'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw': euler_deg[2]},
            'distance_meters': distance
        }

    def calculate_and_save_transform(self, event=None):
        """Calculate the transform chain and save if successful"""
        try:
            # Get individual transforms (PARENT -> CHILD)
            T_cam1opt_marker0, found0 = self.get_transform_as_matrix(self.cam1_optical_frame, self.marker0_frame)
            T_cam1opt_marker1cam1, found1 = self.get_transform_as_matrix(self.cam1_optical_frame, self.marker1_cam1_frame)
            T_cam2opt_marker1cam2, found2 = self.get_transform_as_matrix(self.cam2_optical_frame, self.marker1_cam2_frame)

            if not (found0 and found1 and found2):
                # Add specific logging about which transform failed
                missing_tfs = []
                if not found0: missing_tfs.append(f"{self.marker0_frame} -> {self.cam1_optical_frame}")
                if not found1: missing_tfs.append(f"{self.marker1_cam1_frame} -> {self.cam1_optical_frame}")
                if not found2: missing_tfs.append(f"{self.marker1_cam2_frame} -> {self.cam2_optical_frame}")
                rospy.logwarn_throttle(5.0, f"Required TF transforms not found this cycle. Missing: {', '.join(missing_tfs)}")
                return # Need all transforms to proceed

            # Calculate inverses (CHILD -> PARENT)
            T_marker0_cam1opt = np.linalg.inv(T_cam1opt_marker0)
            T_marker1cam1_cam1opt = np.linalg.inv(T_cam1opt_marker1cam1)
            # T_marker1cam2_cam2opt = np.linalg.inv(T_cam2opt_marker1cam2) # Not needed directly

            # Calculate the chain: T_marker0_cam2opt = T_marker0_cam1opt * T_cam1opt_marker1cam1 * T_marker1cam2_cam2opt
            # Assuming T_marker1cam1 == T_marker1cam2 (same physical marker)
            # T_marker0_cam2opt = T_marker0_cam1opt * T_cam1opt_marker1cam1 * inv(T_cam2opt_marker1cam2)
            T_marker0_cam2opt = T_marker0_cam1opt @ T_cam1opt_marker1cam1 @ np.linalg.inv(T_cam2opt_marker1cam2)

            # Convert the final transform to a dictionary
            transform_data = self.matrix_to_transform_dict(T_marker0_cam2opt, self.marker0_frame, self.cam2_optical_frame)

            # Check if the transform is significantly different from the last saved one or if it's the first
            if self.last_successful_transform is None or not np.allclose(self.last_successful_transform, T_marker0_cam2opt, atol=1e-4):
                rospy.loginfo("Successfully calculated transform chain: %s -> %s", self.marker0_frame, self.cam2_optical_frame)
                rospy.loginfo("  Translation: [%.4f, %.4f, %.4f]",
                              transform_data['transform']['translation']['x'],
                              transform_data['transform']['translation']['y'],
                              transform_data['transform']['translation']['z'])
                rospy.loginfo("  Rotation (quat): [%.4f, %.4f, %.4f, %.4f]",
                              transform_data['transform']['rotation']['x'],
                              transform_data['transform']['rotation']['y'],
                              transform_data['transform']['rotation']['z'],
                              transform_data['transform']['rotation']['w'])
                rospy.loginfo("  Distance: %.4f m", transform_data['distance_meters'])

                # Save to YAML file
                try:
                    with open(self.output_file, 'w') as f:
                        yaml.dump(transform_data, f, default_flow_style=False, sort_keys=False)
                    rospy.loginfo(f"Transform saved to {self.output_file}")
                    self.last_successful_transform = T_marker0_cam2opt # Update last saved transform
                except IOError as e:
                    rospy.logerr(f"Failed to write transform to file {self.output_file}: {e}")
                except yaml.YAMLError as e:
                     rospy.logerr(f"Failed to dump YAML to file {self.output_file}: {e}")

            else:
                rospy.logdebug("Calculated transform is close to the last saved one. Skipping save.")


        except Exception as e:
            rospy.logerr(f"Error in calibration calculation loop: {e}")
            import traceback
            traceback.print_exc()

def main():
    try:
        calculator = CalibrationCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
         rospy.logfatal(f"Unhandled exception in main: {e}")

if __name__ == "__main__":
    main()