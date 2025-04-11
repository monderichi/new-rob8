#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix, euler_from_matrix
import sys

def print_transform_info(transform):
    """Print detailed information about a transform"""
    print("\nTransform from {} to {}:".format(transform.header.frame_id, transform.child_frame_id))
    print("Translation: x={:.4f}, y={:.4f}, z={:.4f} meters".format(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z))
    
    # Convert quaternion to rotation matrix and then to Euler angles
    quat = [transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w]
    
    rot_matrix = quaternion_matrix(quat)
    roll, pitch, yaw = euler_from_matrix(rot_matrix)
    
    print("Rotation (Euler angles): roll={:.4f}, pitch={:.4f}, yaw={:.4f} radians".format(
        roll, pitch, yaw))
    print("                         roll={:.4f}, pitch={:.4f}, yaw={:.4f} degrees".format(
        np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)))
    
    print("\nRotation Matrix:")
    for i in range(3):
        print("    [{:.4f}, {:.4f}, {:.4f}]".format(
            rot_matrix[i, 0], rot_matrix[i, 1], rot_matrix[i, 2]))

def main():
    rospy.init_node('visualization_helper')
    
    if len(sys.argv) < 3:
        print("Usage: visualize_calibration.py <parent_frame> <child_frame>")
        print("Example: visualize_calibration.py camera1_link camera2_link")
        return
    
    parent_frame = sys.argv[1]
    child_frame = sys.argv[2]
    
    # Create a TF2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # Wait for the transform to be available
    rate = rospy.Rate(1.0)
    transform_found = False
    
    print("Waiting for transform from {} to {}...".format(parent_frame, child_frame))
    
    while not rospy.is_shutdown() and not transform_found:
        try:
            # Look up the transform
            transform = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(1.0))
            transform_found = True
            
            # Print the transform information
            print_transform_info(transform)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Error looking up transform: {}".format(e))
            print("Retrying...")
        
        rate.sleep()
        
    if transform_found:
        print("\nTransform successfully visualized.")
    else:
        print("\nFailed to visualize transform.")

if __name__ == '__main__':
    main()