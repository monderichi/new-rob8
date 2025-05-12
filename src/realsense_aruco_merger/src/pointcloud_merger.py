#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import message_filters
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped

class PointCloudMerger:
    def __init__(self):
        rospy.init_node('pointcloud_merger', anonymous=True)
        
        # Get parameters
        self.cam1_frame = rospy.get_param('~cam1_frame', 'cam1_link')
        self.cam2_frame = rospy.get_param('~cam2_frame', 'cam2_link')
        self.target_frame = rospy.get_param('~target_frame', 'cam1_link')  # Reference frame for merged cloud
        self.cam1_topic = rospy.get_param('~cam1_topic', '/cam1/depth/color/points')
        self.cam2_topic = rospy.get_param('~cam2_topic', '/cam2/depth/color/points')
        self.merged_topic = rospy.get_param('~merged_topic', '/merged_pointcloud')
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher for merged pointcloud
        self.merged_pub = rospy.Publisher(self.merged_topic, PointCloud2, queue_size=1)
        
        # Setup synchronization for receiving point clouds
        self.cam1_sub = message_filters.Subscriber(self.cam1_topic, PointCloud2)
        self.cam2_sub = message_filters.Subscriber(self.cam2_topic, PointCloud2)
        
        # Time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.cam1_sub, self.cam2_sub], 
            queue_size=5, 
            slop=0.1  # Allow up to 0.1 second time difference
        )
        self.sync.registerCallback(self.pointcloud_callback)
        
        rospy.loginfo("PointCloud merger initialized.")
        rospy.loginfo(f"Merging: {self.cam1_topic} and {self.cam2_topic}")
        rospy.loginfo(f"Publishing to: {self.merged_topic}")
        rospy.loginfo(f"Reference frame: {self.target_frame}")
        
    def get_transform(self, target_frame, source_frame):
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
        # Get the transforms
        try:
            # If cam1 is the target frame, we don't need to transform it
            if self.cam1_frame != self.target_frame:
                transform1 = self.get_transform(self.target_frame, cloud1_msg.header.frame_id)
                if transform1 is None:
                    return
                cloud1_transformed = do_transform_cloud(cloud1_msg, transform1)
            else:
                cloud1_transformed = cloud1_msg  # No transform needed
                
            # Get transform from cam2 to target frame
            transform2 = self.get_transform(self.target_frame, cloud2_msg.header.frame_id)
            if transform2 is None:
                return
            cloud2_transformed = do_transform_cloud(cloud2_msg, transform2)
            
            # Merge pointclouds by concatenating them
            # In a real-world scenario, we might want to do more complex processing,
            # like filtering overlapping points or applying weights
            points1 = list(pc2.read_points(cloud1_transformed))
            points2 = list(pc2.read_points(cloud2_transformed))
            
            # Create merged pointcloud
            merged_cloud = PointCloud2()
            merged_cloud.header = cloud1_msg.header  # Use header from first cloud
            merged_cloud.header.frame_id = self.target_frame
            
            # Combine the point clouds
            # Use the same field structure as the input clouds
            if len(points1) > 0 and len(points2) > 0:
                fields = cloud1_transformed.fields
                merged_points = points1 + points2
                
                # Create and publish the merged cloud
                merged_cloud = pc2.create_cloud(
                    cloud1_transformed.header,
                    fields,
                    merged_points
                )
                self.merged_pub.publish(merged_cloud)
                rospy.logdebug(f"Published merged cloud with {len(merged_points)} points")
            else:
                rospy.logwarn("One of the point clouds was empty. Skipping merge operation.")
                
        except Exception as e:
            rospy.logerr(f"Error in point cloud merger: {e}")

if __name__ == '__main__':
    try:
        merger = PointCloudMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
