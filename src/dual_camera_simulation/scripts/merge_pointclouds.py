#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf
import numpy as np
import pcl

def transform_pointcloud(cloud, transform):
    """Transforms a point cloud using a 4x4 transformation matrix."""
    points_list = []
    for p in pc2.read_points(cloud, field_names = ("x", "y", "z", "rgb"), skip_nans = True):
        # Convert to homogeneous coordinates
        point = np.array([p[0], p[1], p[2], 1.0])
        # Apply the transformation
        point_transformed = np.dot(transform, point)
        # Append the transformed point with the original color
        points_list.append([point_transformed[0], point_transformed[1], point_transformed[2], p[3]])

    # Create a new point cloud
    cloud_out = pc2.create_cloud(cloud.header, cloud.fields, points_list)
    return cloud_out

class PointCloudMerger:
    def __init__(self):
        rospy.init_node('pointcloud_merger')

        self.tf_buffer = tf2_ros.BufferClient(rospy.get_namespace())
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cam1_frame = "camera1_depth_optical_frame"
        self.cam2_frame = "camera2_depth_optical_frame"

        self.cloud1 = None  # Store the point cloud from camera 1

        self.cloud1_sub = rospy.Subscriber("/camera1/points", PointCloud2, self.cloud1_callback)
        self.cloud2_sub = rospy.Subscriber("/camera2/points", PointCloud2, self.cloud2_callback)

        self.merged_cloud_pub = rospy.Publisher("/merged_points", PointCloud2, queue_size=1)

    def cloud1_callback(self, cloud):
        self.cloud1 = cloud

    def cloud2_callback(self, cloud):
        if self.cloud1 is None:
            rospy.logwarn("Received cloud from camera2 before cloud from camera1.  Waiting...")
            return

        try:
            # Get the transform from camera2_depth_optical_frame to camera1_depth_optical_frame
            transform = self.tf_buffer.lookup_transform(self.cam1_frame, self.cam2_frame, rospy.Time(), rospy.Duration(1.0))

            # Convert the transform to a 4x4 homogeneous transformation matrix
            translation = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
            rotation = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
            transform_matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(translation), tf.transformations.quaternion_matrix(rotation))

            # Transform the point cloud from camera 2
            transformed_cloud = transform_pointcloud(cloud, transform_matrix)

            # Merge the point clouds (concatenate the point data)
            points1 = list(pc2.read_points(self.cloud1, field_names = ("x", "y", "z", "rgb"), skip_nans=True))
            points2 = list(pc2.read_points(transformed_cloud, field_names = ("x", "y", "z", "rgb"), skip_nans=True))
            merged_points = points1 + points2

            # Create a new point cloud message
            merged_cloud = pc2.create_cloud(self.cloud1.header, self.cloud1.fields, merged_points)

            # Publish the merged cloud
            self.merged_cloud_pub.publish(merged_cloud)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF error: %s", e)
            return
        except Exception as e:
            rospy.logerr("An unexpected error occurred: %s", e)
            return

if __name__ == '__main__':
    try:
        merger = PointCloudMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
