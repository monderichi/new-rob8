#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import tf.transformations
from geometry_msgs.msg import TransformStamped
import os # Import os module

class DirectTransformCalculator:
    """
    Calculates and saves the transform directly from ArUco marker ID 0
    (published as 'aruco_marker_cam1_m0') to cam2_link by listening to TF.
    Assumes the TF tree is correctly published by other nodes.
    """

    def __init__(self):
        rospy.init_node('marker0_to_cam2_direct_calculator', anonymous=True)

        # Use TF1 listener
        self.tf_listener = tf.TransformListener()

        # Frame IDs (Match the launch file and standard frames)
        # Use ROS param for marker frame, fallback to the name defined in the launch file
        self.marker0_frame = rospy.get_param('~marker0_frame', 'aruco_marker_0') # Changed back to aruco_marker_0
        self.target_frame = rospy.get_param('~target_frame', 'cam2_link')
        # Output file path
        self.output_file = os.path.join(os.path.expanduser("~"), "catkin_ws", "marker0_to_cam2_transform.txt")

        # Sleep to wait for TF system to initialize
        rospy.sleep(2.0)
        rospy.loginfo("Starting direct %s to %s transform calculator...", self.marker0_frame, self.target_frame)
        rospy.loginfo("Will save transform to: %s", self.output_file)

        # Timer for periodic calculations
        self.timer = rospy.Timer(rospy.Duration(1.0), self.calculate_and_save_transform)

        # Flag to track if transform has been found and printed first time
        self.first_transform_found = False

    def get_transform_as_matrix(self, target_frame, source_frame):
        """Try to get a transform and convert to a 4x4 homogeneous matrix"""
        try:
            # Wait for the transform to become available
            self.tf_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(0.5)
            )

            # Lookup the transform
            (trans, rot) = self.tf_listener.lookupTransform(
                target_frame, source_frame, rospy.Time(0)
            )

            # Create homogeneous transformation matrix
            matrix = tf.transformations.quaternion_matrix(rot)
            matrix[0:3, 3] = trans

            rospy.logdebug("Got transform: %s -> %s", source_frame, target_frame)
            return matrix, True

        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            # Log warning less frequently if transform is consistently unavailable
            # Use specific error message for unconnected trees
            if "unconnected trees" in str(e):
                 rospy.logwarn_throttle(5.0, "TF Error: Could not find connection between '%s' and '%s'. Check if static_transform_publisher and camera nodes are running and publishing correctly.", target_frame, source_frame)
            else:
                 rospy.logwarn_throttle(10.0, "TF Error getting %s -> %s: %s. Make sure aruco_detector and camera nodes are running and publishing TF.",
                                 source_frame, target_frame, e)
            return np.eye(4), False

    def matrix_to_transform_data(self, matrix):
        """Convert a 4x4 matrix to translation and quaternion"""
        trans = matrix[0:3, 3]
        rot = tf.transformations.quaternion_from_matrix(matrix)
        euler = tf.transformations.euler_from_quaternion(rot)
        euler_deg = [np.degrees(e) for e in euler]

        return trans, rot, euler_deg

    def calculate_and_save_transform(self, event=None):
        """Calculate transform directly from marker0 to the target frame"""
        try:
            # Get the direct transform: target -> source
            T_target_marker0, found = self.get_transform_as_matrix(
                self.target_frame, self.marker0_frame
            )

            if not found:
                # If transform not found, do nothing this cycle
                return

            # If found, calculate the inverse: source -> target (marker0 -> target_frame)
            T_marker0_target = np.linalg.inv(T_target_marker0)

            # Extract translation and rotation
            trans, rot, euler_deg = self.matrix_to_transform_data(T_marker0_target)
            distance = np.sqrt(np.sum(trans**2))

            # Print header only the first time
            if not self.first_transform_found:
                rospy.loginfo("\n================================")
                rospy.loginfo("TRANSFORM FOUND: %s -> %s", self.marker0_frame, self.target_frame)
                rospy.loginfo("================================")
                self.first_transform_found = True

            # Log the latest transform details
            rospy.loginfo("\nLatest Transform from %s to %s:",
                         self.marker0_frame, self.target_frame)
            rospy.loginfo("  Translation [x, y, z]: [%.6f, %.6f, %.6f] meters",
                         trans[0], trans[1], trans[2])
            rospy.loginfo("  Rotation [x, y, z, w]: [%.6f, %.6f, %.6f, %.6f]",
                         rot[0], rot[1], rot[2], rot[3])
            rospy.loginfo("  Euler angles [roll, pitch, yaw]: [%.2f, %.2f, %.2f] degrees",
                         euler_deg[0], euler_deg[1], euler_deg[2])
            rospy.loginfo("  Distance: %.6f meters", distance)
            rospy.loginfo("--------------------------------")

            # Save the latest transform to the file
            try:
                with open(self.output_file, "w") as f:
                    f.write("# Transform from %s to %s\n" % (self.marker0_frame, self.target_frame))
                    f.write("Translation [x, y, z]: [%.6f, %.6f, %.6f] meters\n" %
                          (trans[0], trans[1], trans[2]))
                    f.write("Rotation [x, y, z, w]: [%.6f, %.6f, %.6f, %.6f]\n" %
                          (rot[0], rot[1], rot[2], rot[3]))
                    f.write("Euler angles [roll, pitch, yaw]: [%.2f, %.2f, %.2f] degrees\n" %
                          (euler_deg[0], euler_deg[1], euler_deg[2]))
                    f.write("Distance: %.6f meters\n" % distance)
                rospy.logdebug("Transform saved to %s", self.output_file)
            except IOError as e:
                rospy.logerr("Failed to write transform to file %s: %s", self.output_file, e)

        except Exception as e:
            rospy.logerr("Error in transform calculation loop: %s", e)
            import traceback
            traceback.print_exc()

def main():
    calculator = DirectTransformCalculator()
    rospy.spin()

if __name__ == "__main__":
    main()