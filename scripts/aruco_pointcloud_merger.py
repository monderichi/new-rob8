#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
import message_filters

class ArucoPointCloudMerger:
    def __init__(self):
        rospy.init_node('aruco_pointcloud_merger', anonymous=True)
        
        # Get parameters
        self.cam1_frame = rospy.get_param('~cam1_frame', 'cam1_link')
        self.cam2_frame = rospy.get_param('~cam2_frame', 'cam2_link')
        self.target_frame = rospy.get_param('~target_frame', 'world')
        self.cam1_topic = rospy.get_param('~cam1_topic', '/cam1/cam1/depth/color/points')
        self.cam2_topic = rospy.get_param('~cam2_topic', '/cam2/cam2/depth/color/points')
        self.merged_topic = rospy.get_param('~merged_topic', '/merged_pointcloud')
        
        # TF buffer and listener for getting transformations
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher for merged pointcloud
        self.merged_pub = rospy.Publisher(self.merged_topic, PointCloud2, queue_size=1)
        
        # Setup subscribers for point clouds
        self.cam1_sub = message_filters.Subscriber(self.cam1_topic, PointCloud2)
        self.cam2_sub = message_filters.Subscriber(self.cam2_topic, PointCloud2)
        
        # Synchronize the two point cloud feeds with a time tolerance
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.cam1_sub, self.cam2_sub], 
            queue_size=5, 
            slop=0.1  # Allow up to 0.1 second time difference
        )
        self.ts.registerCallback(self.pointcloud_callback)
        
        rospy.loginfo("ArUco point cloud merger initialized")
        rospy.loginfo(f"Merging: {self.cam1_topic} and {self.cam2_topic}")
        rospy.loginfo(f"Publishing to: {self.merged_topic}")
        rospy.loginfo(f"Target frame: {self.target_frame}")
        
    def get_transform(self, target_frame, source_frame):
        """Get transformation from source frame to target frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),  # Get latest available transform
                rospy.Duration(1.0)  # Wait up to 1 second for transform
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get transform from {source_frame} to {target_frame}: {e}")
            return None
            
    def pointcloud_callback(self, cloud1_msg, cloud2_msg):
        """Process incoming point clouds and merge them"""
        # Validate that we have both point clouds
        if not cloud1_msg or not cloud2_msg:
            rospy.logwarn("Received empty point cloud message. Skipping merge.")
            return
        
        try:
            # Get transform from cam1 to target frame
            transform1 = self.get_transform(self.target_frame, cloud1_msg.header.frame_id)
            if transform1 is None:
                rospy.logwarn(f"No transform from {cloud1_msg.header.frame_id} to {self.target_frame}. Skipping merge.")
                return
                
            # Get transform from cam2 to target frame
            transform2 = self.get_transform(self.target_frame, cloud2_msg.header.frame_id)
            if transform2 is None:
                rospy.logwarn(f"No transform from {cloud2_msg.header.frame_id} to {self.target_frame}. Skipping merge.")
                return
            
            # Transform both point clouds to target frame
            cloud1_transformed = do_transform_cloud(cloud1_msg, transform1)
            cloud2_transformed = do_transform_cloud(cloud2_msg, transform2)
            
            # Merge point clouds
            points1 = list(pc2.read_points(cloud1_transformed))
            points2 = list(pc2.read_points(cloud2_transformed))
            
            if not points1 or not points2:
                rospy.logwarn("One or both transformed point clouds are empty. Skipping merge.")
                return
            
            # Create merged point cloud
            merged_points = points1 + points2
            
            # Create output point cloud
            merged_cloud = pc2.create_cloud(
                cloud1_transformed.header,
                cloud1_transformed.fields,  # Use the field structure from the first cloud
                merged_points
            )
            
            # Set header for the merged cloud
            merged_cloud.header.stamp = rospy.Time.now()
            merged_cloud.header.frame_id = self.target_frame
            
            # Publish the merged point cloud
            self.merged_pub.publish(merged_cloud)
            
            # Log every 5 seconds to avoid flooding the console
            rospy.loginfo_throttle(5.0, f"Merged cloud with {len(merged_points)} points")
            
        except Exception as e:
            rospy.logerr(f"Error merging point clouds: {e}")

def main():
    merger = ArucoPointCloudMerger()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass