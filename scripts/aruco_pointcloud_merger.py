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
import tf.transformations

class ArucoPointCloudMerger:
    def __init__(self):
        rospy.init_node('aruco_pointcloud_merger', anonymous=True)
        
        # Get parameters
        self.cam1_frame = rospy.get_param('~cam1_frame', 'cam1_color_optical_frame')  # Updated default frames
        self.cam2_frame = rospy.get_param('~cam2_frame', 'cam2_color_optical_frame')  # Updated default frames
        self.target_frame = rospy.get_param('~target_frame', 'world')
        self.cam1_topic = rospy.get_param('~cam1_topic', '/cam1/cam1/depth/color/points')
        self.cam2_topic = rospy.get_param('~cam2_topic', '/cam2/cam2/depth/color/points')
        self.merged_topic = rospy.get_param('~merged_topic', '/merged_pointcloud')
        
        # TF buffer and listener for getting transformations
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))  # Increased buffer time
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher for merged pointcloud
        self.merged_pub = rospy.Publisher(self.merged_topic, PointCloud2, queue_size=10)  # Increased queue size
        
        # Debug publishers - optional, publish individual transformed clouds for comparison
        self.debug_cloud1_pub = rospy.Publisher('/debug_cloud1_transformed', PointCloud2, queue_size=10)
        self.debug_cloud2_pub = rospy.Publisher('/debug_cloud2_transformed', PointCloud2, queue_size=10)
        
        # Setup subscribers for point clouds
        self.cam1_sub = message_filters.Subscriber(self.cam1_topic, PointCloud2)
        self.cam2_sub = message_filters.Subscriber(self.cam2_topic, PointCloud2)
        
        # Synchronize the two point cloud feeds with a time tolerance
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.cam1_sub, self.cam2_sub], 
            queue_size=10,   # Increased queue size for better matching
            slop=0.5         # Increased time tolerance to 0.5 seconds
        )
        self.ts.registerCallback(self.pointcloud_callback)
        
        # For diagnostics
        self.last_published = rospy.Time(0)
        self.tf_check_timer = rospy.Timer(rospy.Duration(1.0), self.check_transforms)
        
        rospy.loginfo("ArUco point cloud merger initialized")
        rospy.loginfo(f"Merging: {self.cam1_topic} and {self.cam2_topic}")
        rospy.loginfo(f"Publishing to: {self.merged_topic}")
        rospy.loginfo(f"Target frame: {self.target_frame}")
        rospy.loginfo(f"Camera 1 frame: {self.cam1_frame}")
        rospy.loginfo(f"Camera 2 frame: {self.cam2_frame}")
        
    def check_transforms(self, event):
        """Periodically check if transforms are available"""
        # Check for transform from world to cam1
        try:
            trans1 = self.tf_buffer.lookup_transform(
                self.target_frame, 
                self.cam1_frame, 
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            t = trans1.transform.translation
            r = trans1.transform.rotation
            rospy.loginfo_throttle(5.0, f"Transform from {self.cam1_frame} to {self.target_frame} is available: [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Transform from {self.cam1_frame} to {self.target_frame} is NOT available: {e}")
            
        # Check for transform from world to cam2
        try:
            trans2 = self.tf_buffer.lookup_transform(
                self.target_frame, 
                self.cam2_frame, 
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            t = trans2.transform.translation
            r = trans2.transform.rotation
            rospy.loginfo_throttle(5.0, f"Transform from {self.cam2_frame} to {self.target_frame} is available: [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Transform from {self.cam2_frame} to {self.target_frame} is NOT available: {e}")
            
        # Check if anyone is subscribed to our merged pointcloud
        if self.merged_pub.get_num_connections() == 0:
            rospy.logwarn_throttle(10.0, "No subscribers to the merged pointcloud topic. Is RViz configured to display it?")
        else:
            rospy.loginfo_throttle(10.0, f"Number of subscribers to merged pointcloud: {self.merged_pub.get_num_connections()}")
        
        # Check if we've published recently
        if (rospy.Time.now() - self.last_published).to_sec() > 5.0:
            rospy.logwarn_throttle(5.0, "No merged pointcloud has been published in the last 5 seconds")
        
    def get_transform(self, target_frame, source_frame):
        """Get transformation from source frame to target frame"""
        try:
            # Use a small timeout to avoid blocking
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(1.0)  # Increased timeout
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get transform from {source_frame} to {self.target_frame}: {e}")
            return None
            
    def pointcloud_callback(self, cloud1_msg, cloud2_msg):
        """Process incoming point clouds and merge them"""
        # Debug info about received point clouds
        rospy.loginfo_throttle(3.0, f"Received pointcloud from cam1, frame: {cloud1_msg.header.frame_id}, points: {cloud1_msg.width * cloud1_msg.height}")
        rospy.loginfo_throttle(3.0, f"Received pointcloud from cam2, frame: {cloud2_msg.header.frame_id}, points: {cloud2_msg.width * cloud2_msg.height}")
        
        # Validate that we have both point clouds
        if not cloud1_msg or not cloud2_msg:
            rospy.logwarn_throttle(3.0, "Received empty point cloud message. Skipping merge.")
            return
        
        try:
            # Get actual frame IDs from the messages
            frame1 = cloud1_msg.header.frame_id
            frame2 = cloud2_msg.header.frame_id
            
            # Get transform from point cloud frames to target frame
            transform1 = self.get_transform(self.target_frame, frame1)
            if transform1 is None:
                rospy.logwarn_throttle(3.0, f"No transform from {frame1} to {self.target_frame}. Skipping merge.")
                return
                
            transform2 = self.get_transform(self.target_frame, frame2)
            if transform2 is None:
                rospy.logwarn_throttle(3.0, f"No transform from {frame2} to {self.target_frame}. Skipping merge.")
                return
            
            # Log transform details (for debugging)
            t1 = transform1.transform.translation
            r1 = transform1.transform.rotation
            t2 = transform2.transform.translation
            r2 = transform2.transform.rotation
            rospy.loginfo_throttle(5.0, f"Transform1: pos=[{t1.x:.3f}, {t1.y:.3f}, {t1.z:.3f}], rot=[{r1.x:.3f}, {r1.y:.3f}, {r1.z:.3f}, {r1.w:.3f}]")
            rospy.loginfo_throttle(5.0, f"Transform2: pos=[{t2.x:.3f}, {t2.y:.3f}, {t2.z:.3f}], rot=[{r2.x:.3f}, {r2.y:.3f}, {r2.z:.3f}, {r2.w:.3f}]")
            
            # Transform both point clouds to target frame
            cloud1_transformed = do_transform_cloud(cloud1_msg, transform1)
            cloud2_transformed = do_transform_cloud(cloud2_msg, transform2)
            
            # Publish individual transformed clouds for debugging
            self.debug_cloud1_pub.publish(cloud1_transformed)
            self.debug_cloud2_pub.publish(cloud2_transformed)
            
            # Convert to list of points
            points1 = list(pc2.read_points(cloud1_transformed))
            points2 = list(pc2.read_points(cloud2_transformed))
            
            rospy.loginfo_throttle(3.0, f"Points in cloud1: {len(points1)}, Points in cloud2: {len(points2)}")
            
            if not points1:
                rospy.logwarn_throttle(3.0, "Transformed point cloud 1 is empty. Skipping merge.")
                return
                
            if not points2:
                rospy.logwarn_throttle(3.0, "Transformed point cloud 2 is empty. Skipping merge.")
                return
            
            # Create merged point cloud
            merged_points = points1 + points2
            
            # Create output point cloud - use cloud1's structure
            merged_cloud = pc2.create_cloud(
                cloud1_transformed.header,
                cloud1_transformed.fields,
                merged_points
            )
            
            # Set header for the merged cloud
            merged_cloud.header.stamp = rospy.Time.now()
            merged_cloud.header.frame_id = self.target_frame
            
            # Publish the merged point cloud
            self.merged_pub.publish(merged_cloud)
            self.last_published = rospy.Time.now()
            
            # Log success
            rospy.loginfo_throttle(3.0, f"Published merged cloud with {len(merged_points)} points in frame {self.target_frame}")
            
        except Exception as e:
            rospy.logerr(f"Error merging point clouds: {e}")
            import traceback
            traceback.print_exc()

def main():
    merger = ArucoPointCloudMerger()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass