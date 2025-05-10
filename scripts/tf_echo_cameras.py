#!/usr/bin/env python
import rospy
import tf2_ros
import math
import geometry_msgs.msg

def main():
    rospy.init_node('camera_tf_echoer', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # --- Get frame names from parameters (defaults provided) ---
    source_frame = rospy.get_param('~source_frame', 'cam1_color_optical_frame')
    target_frame = rospy.get_param('~target_frame', 'cam2_color_optical_frame')
    # --- ---

    rate = rospy.Rate(1.0) # Print once per second

    rospy.loginfo("Camera TF Echoer started. Looking for transform from '%s' to '%s'", source_frame, target_frame)

    while not rospy.is_shutdown():
        try:
            # Use lookup_transform(target, source, time)
            # We want the pose of target_frame relative to source_frame
            trans = tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(1.0))

            t = trans.transform.translation
            # q = trans.transform.rotation # Quaternion not needed for this specific output format

            distance = math.sqrt(t.x**2 + t.y**2 + t.z**2)

            # --- Print in the requested format ---
            rospy.loginfo("Published TF: %s -> %s, translation=[%.4f, %.4f, %.4f]m, distance=%.4f m",
                          source_frame, target_frame, t.x, t.y, t.z, distance)
            # --- ---

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn_throttle(5.0, "Could not get transform from '%s' to '%s': %s", source_frame, target_frame, ex)
            # Continue trying
        except Exception as e:
            rospy.logerr("An unexpected error occurred: %s", e)
            break # Exit on unexpected errors

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down camera_tf_echoer.")
            break

if __name__ == '__main__':
    main()
