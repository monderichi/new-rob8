#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int32  # Add this import
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped

from intent_prediction_calc import IntentPredictCalc

class IntentPredictor:
    def __init__(self):
        rospy.init_node('intent_predictor')

        # Initializing TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Defining frame names
        self.hand_depth_frame = "hand_camera_depth_optical_frame"
        self.head_depth_frame = "head_camera_depth_optical_frame"

        # initialize the intent predict algorithm class
        self.intent_predict_calc = IntentPredictCalc()
        
        # Subscribe to the centroid markers from PCLProcessor
        self.centroid_sub = rospy.Subscriber('/centroid_markers', MarkerArray, self.centroid_callback)
        
        # Publish the selected object ID
        self.intent_pub = rospy.Publisher('/object_id_to_grasp', Int32, queue_size=1)

        # Waiting for transforms to become available
        rospy.loginfo("Waiting for transforms to become available...")
        self._wait_for_transforms()

        # Add debug timer to periodically report transform status
        rospy.Timer(rospy.Duration(10.0), self._debug_transforms)
        
    def centroid_callback(self, marker_array):
        num_markers = len(marker_array.markers)

        if num_markers == 0:
            rospy.logwarn("Received empty marker array")
            return
        
        # get transform from hand depth frame to head depth frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.head_depth_frame, # target frame
                self.hand_depth_frame, # source frame
                rospy.Time(0), # get latest available transform
                rospy.Duration(1.0) # wait up to 1 second for transform
            )
            rospy.logdebug_throttle(5.0, "Got transform from hand to head depth frame")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to lookup transform: {e}")
            return
        
        # Pre-allocate arrays
        centroids = np.zeros((num_markers, 3), dtype=np.float32)
        labels = np.zeros(num_markers, dtype=np.int32)
        transformed_centroids = np.zeros((num_markers, 3), dtype=np.float32)  # New array for transformed points
        
        # Fill arrays (normal + transformed centroids)
        for i, marker in enumerate(marker_array.markers):
            # rriginal position in hand camera frame
            position = marker.pose.position
            centroids[i] = [position.x, position.y, position.z]
            labels[i] = marker.id

            # transform the centroid to the head camera depth frame
            point = PointStamped()
            point.header.frame_id = self.hand_depth_frame
            point.point.x = position.x
            point.point.y = position.y
            point.point.z = position.z

            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)

            transformed_centroids[i] = [
                transformed_point.point.x,
                transformed_point.point.y,
                transformed_point.point.z
            ]
        
        # Run the intent prediction algorithm
        intended_idx = self.intent_predict_calc.find_grasp_obj(centroids, transformed_centroids, labels)

        # check if the index is valid
        if 0 <= intended_idx < len(labels):
            self.intent_pub.publish(int(labels[intended_idx]))
            rospy.loginfo(f"Predicted grasp object ID: {labels[intended_idx]}")
        else:
            rospy.logwarn(f"Invalid prediction index: {intended_idx}")

    def _wait_for_transforms(self):
        """Wait for the required transforms to be available"""
        frames_ready = False
        while not frames_ready and not rospy.is_shutdown():
            try:
                # Check if we can transform between the frames
                self.tf_buffer.lookup_transform(
                    self.head_depth_frame,
                    self.hand_depth_frame,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                frames_ready = True
                rospy.loginfo("All required transforms are available.")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(5.0, "Waiting for transforms between camera frames...")
                rospy.sleep(0.5)

    def _debug_transforms(self, event=None):
        """Print debug info about the TF tree"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.head_depth_frame,
                self.hand_depth_frame,
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            rospy.loginfo(f"Transform hand→head: [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}], quat=[{r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f}]")
        except Exception as e:
            rospy.logwarn(f"Debug transform check failed: {e}")

if __name__ == '__main__':
    predictor = IntentPredictor()
    rospy.spin()