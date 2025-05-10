#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf.transformations
from std_msgs.msg import Float64MultiArray, String

class AlignmentDebugger:
    def __init__(self):
        rospy.init_node('alignment_debugger', anonymous=True)
        
        # Get parameters
        self.cam1_frame = rospy.get_param('~cam1_frame', 'cam1_color_optical_frame')
        self.cam2_frame = rospy.get_param('~cam2_frame', 'cam2_color_optical_frame')
        self.world_frame = rospy.get_param('~world_frame', 'world')
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        
        # Publishers for debugging information
        self.angles_pub = rospy.Publisher('/alignment_debug/angles', Float64MultiArray, queue_size=5)
        self.status_pub = rospy.Publisher('/alignment_debug/status', String, queue_size=5)
        
        # Timer to update transform analysis
        self.update_timer = rospy.Timer(rospy.Duration(0.2), self.analyze_transform)
        
        self.prev_angles = None
        self.angle_stability_threshold = 0.5  # degrees
        
        rospy.loginfo("Alignment Debugger initialized")
        rospy.loginfo(f"Monitoring transform between {self.cam1_frame} and {self.cam2_frame}")
        
    def analyze_transform(self, event):
        """Analyze the transformation between cameras and publish debug data"""
        try:
            # Wait for the transform to become available
            if not self.tf_listener.canTransform(self.cam1_frame, self.cam2_frame, rospy.Time(0)):
                status_msg = String()
                status_msg.data = "Waiting for transform between cameras"
                self.status_pub.publish(status_msg)
                return
                
            # Get the transform
            (trans, rot) = self.tf_listener.lookupTransform(self.cam1_frame, self.cam2_frame, rospy.Time(0))
            
            # Convert quaternion to Euler angles (roll, pitch, yaw) in degrees
            euler_rad = tf.transformations.euler_from_quaternion(rot)
            euler_deg = [np.degrees(angle) for angle in euler_rad]
            
            # Calculate distance between cameras
            distance = np.sqrt(trans[0]**2 + trans[1]**2 + trans[2]**2)
            
            # Normalize angles to -180 to 180 range
            normalized_angles = [(angle + 180) % 360 - 180 for angle in euler_deg]
            
            # Check angle stability
            if self.prev_angles is not None:
                angle_diffs = [abs(a - b) for a, b in zip(normalized_angles, self.prev_angles)]
                max_diff = max(angle_diffs)
                
                # Prepare status message
                if max_diff > self.angle_stability_threshold:
                    stability = f"UNSTABLE (max diff: {max_diff:.2f}°)"
                else:
                    stability = f"Stable (max diff: {max_diff:.2f}°)"
            else:
                stability = "First reading"
                
            self.prev_angles = normalized_angles
            
            # Create message with angles and distance
            angles_msg = Float64MultiArray()
            angles_msg.data = normalized_angles + [distance]  # [roll, pitch, yaw, distance]
            self.angles_pub.publish(angles_msg)
            
            # Create status message
            status_msg = String()
            status_msg.data = (
                f"Camera Alignment:\n"
                f"Roll: {normalized_angles[0]:.2f}°, Pitch: {normalized_angles[1]:.2f}°, Yaw: {normalized_angles[2]:.2f}°\n"
                f"Distance: {distance:.4f}m\n"
                f"Stability: {stability}"
            )
            self.status_pub.publish(status_msg)
            
            # Log less frequently to console
            rospy.loginfo_throttle(3.0, 
                f"Camera Alignment - Roll: {normalized_angles[0]:.2f}°, Pitch: {normalized_angles[1]:.2f}°, "
                f"Yaw: {normalized_angles[2]:.2f}°, Distance: {distance:.4f}m, {stability}"
            )
            
            # Provide alignment suggestions
            self.provide_alignment_suggestions(normalized_angles, distance)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            status_msg = String()
            status_msg.data = f"Transform error: {str(e)}"
            self.status_pub.publish(status_msg)
            rospy.logwarn_throttle(5.0, f"Transform error: {str(e)}")
    
    def provide_alignment_suggestions(self, angles, distance):
        """Provide suggestions for better camera alignment"""
        # Define threshold for "good" alignment (in degrees)
        angle_threshold = 5.0
        
        roll, pitch, yaw = angles
        suggestions = []
        
        # Check roll alignment
        if abs(roll) > angle_threshold:
            direction = "clockwise" if roll > 0 else "counter-clockwise"
            suggestions.append(f"Roll misalignment: Rotate camera 2 {direction} by ~{abs(roll):.1f}°")
            
        # Check pitch alignment
        if abs(pitch) > angle_threshold:
            direction = "down" if pitch > 0 else "up"
            suggestions.append(f"Pitch misalignment: Tilt camera 2 {direction} by ~{abs(pitch):.1f}°")
            
        # Check yaw alignment
        if abs(yaw) > angle_threshold:
            direction = "right" if yaw > 0 else "left"
            suggestions.append(f"Yaw misalignment: Turn camera 2 to the {direction} by ~{abs(yaw):.1f}°")
        
        # If we have suggestions, publish them
        if suggestions:
            suggestion_msg = String()
            suggestion_msg.data = "ALIGNMENT SUGGESTIONS:\n" + "\n".join(suggestions)
            self.status_pub.publish(suggestion_msg)
            rospy.logwarn_throttle(10.0, "ALIGNMENT SUGGESTIONS:\n" + "\n".join(suggestions))
        elif sum(abs(angle) for angle in angles) < angle_threshold * 3:
            # All angles are good
            good_msg = String()
            good_msg.data = f"✓ Good alignment (within {angle_threshold}° tolerance)"
            self.status_pub.publish(good_msg)

def main():
    debugger = AlignmentDebugger()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass