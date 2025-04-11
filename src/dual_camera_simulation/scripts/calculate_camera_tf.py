#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import tf.transformations as tft
import numpy as np

def main():
    rospy.init_node('dual_camera_tf_calculator')

    # Get the delay parameter
    delay = rospy.get_param("~delay", 5.0)  # Default to 5 seconds if not specified
    rospy.sleep(delay)

    listener = tf.TransformListener()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    cam1_frame = "camera1_depth_optical_frame"
    cam2_frame = "camera2_depth_optical_frame"
    marker_frame = "aruco_marker_0"

    rate = rospy.Rate(1.0) # Check for transforms once per second
    tf_calculated = False

    rospy.loginfo("Waiting for transforms from cameras to marker...")

    while not rospy.is_shutdown() and not tf_calculated:
        try:
            # Get transform from Camera 1 to Marker
            listener.waitForTransform(cam1_frame, marker_frame, rospy.Time(0), rospy.Duration(4.0))
            (trans1, rot1) = listener.lookupTransform(cam1_frame, marker_frame, rospy.Time(0))
            T_cam1_marker = listener.fromTranslationRotation(trans1, rot1) # Get 4x4 matrix
            rospy.loginfo("Got transform from %s to %s", cam1_frame, marker_frame)

            # Get transform from Camera 2 to Marker
            listener.waitForTransform(cam2_frame, marker_frame, rospy.Time(0), rospy.Duration(4.0))
            (trans2, rot2) = listener.lookupTransform(cam2_frame, marker_frame, rospy.Time(0))
            T_cam2_marker = listener.fromTranslationRotation(trans2, rot2) # Get 4x4 matrix
            rospy.loginfo("Got transform from %s to %s", cam2_frame, marker_frame)

            # Calculate transform from Camera 1 to Camera 2
            # T_cam1_cam2 = T_cam1_marker * inv(T_cam2_marker)
            T_marker_cam2 = tft.inverse_matrix(T_cam2_marker)
            T_cam1_cam2 = np.dot(T_cam1_marker, T_marker_cam2)

            # Extract translation and rotation for publishing
            trans_cam1_cam2 = tft.translation_from_matrix(T_cam1_cam2)
            rot_cam1_cam2 = tft.quaternion_from_matrix(T_cam1_cam2)

            # Prepare the static transform message
            static_transform_stamped = geometry_msgs.msg.TransformStamped()
            static_transform_stamped.header.stamp = rospy.Time.now()
            static_transform_stamped.header.frame_id = cam1_frame
            static_transform_stamped.child_frame_id = cam2_frame
            static_transform_stamped.transform.translation.x = trans_cam1_cam2[0]
            static_transform_stamped.transform.translation.y = trans_cam1_cam2[1]
            static_transform_stamped.transform.translation.z = trans_cam1_cam2[2]
            static_transform_stamped.transform.rotation.x = rot_cam1_cam2[0]
            static_transform_stamped.transform.rotation.y = rot_cam1_cam2[1]
            static_transform_stamped.transform.rotation.z = rot_cam1_cam2[2]
            static_transform_stamped.transform.rotation.w = rot_cam1_cam2[3]

            # Publish the static transform
            static_broadcaster.sendTransform(static_transform_stamped)
            rospy.loginfo("Published static transform from %s to %s.", cam1_frame, cam2_frame)
            tf_calculated = True # Exit after publishing once

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Could not get required transforms yet: %s. Retrying...", e)
            rate.sleep()
        except Exception as e:
            rospy.logerr("An unexpected error occurred: %s", e)
            break # Exit on other errors

    if tf_calculated:
        rospy.loginfo("Static transform calculation complete. Node will exit.")
    else:
         rospy.logwarn("Could not calculate static transform before shutdown.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
