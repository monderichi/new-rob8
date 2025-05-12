#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // Required for tf2::doTransform
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h> // Required for pcl_ros::transformPointCloud
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h> // For VoxelGrid filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>

// Define the point type used throughout
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

class PointCloudMerger
{
public:
    PointCloudMerger() : 
        nh_("~"), // Use private node handle for parameters
        tf_listener_(tf_buffer_), // Initialize TF listener with buffer
        sync_(SyncPolicy(10), sub_cloud1_, sub_cloud2_) // Initialize synchronizer
    {
        // Get parameters
        nh_.param<std::string>("cam1_topic", cam1_topic_, "/cam1/depth/color/points");
        nh_.param<std::string>("cam2_topic", cam2_topic_, "/cam2/depth/color/points");
        nh_.param<std::string>("merged_topic", merged_topic_, "/merged_pointcloud");
        nh_.param<std::string>("target_frame", target_frame_, "world"); // Default to 'world' frame
        nh_.param<bool>("enable_voxel_filter", enable_voxel_filter_, true); // Enable filtering by default
        nh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.03); // Increased to 3cm to avoid overflow

        ROS_INFO("PointCloudMerger Node Initializing:");
        ROS_INFO("  Cam1 Topic: %s", cam1_topic_.c_str());
        ROS_INFO("  Cam2 Topic: %s", cam2_topic_.c_str());
        ROS_INFO("  Merged Topic: %s", merged_topic_.c_str());
        ROS_INFO("  Target Frame: %s", target_frame_.c_str());
        ROS_INFO("  Voxel Filter Enabled: %s", enable_voxel_filter_ ? "true" : "false");
        if (enable_voxel_filter_) {
            ROS_INFO("  Voxel Leaf Size: %f meters", voxel_leaf_size_);
        }
        ROS_INFO("  Using CPU processing");

        // Initialize subscribers using message_filters
        sub_cloud1_.subscribe(nh_global_, cam1_topic_, 1);
        sub_cloud2_.subscribe(nh_global_, cam2_topic_, 1);

        // Register callback for synchronized messages
        sync_.registerCallback(boost::bind(&PointCloudMerger::cloudsCallback, this, _1, _2));

        // Initialize publisher
        merged_cloud_pub_ = nh_global_.advertise<sensor_msgs::PointCloud2>(merged_topic_, 1);

        ROS_INFO("PointCloudMerger node ready.");
    }

private:
    ros::NodeHandle nh_; // Private NodeHandle for parameters
    ros::NodeHandle nh_global_; // Global NodeHandle for topics/tf
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud1_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud2_;

    // Define synchronization policy (ApproximateTime)
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync_;

    ros::Publisher merged_cloud_pub_;

    std::string cam1_topic_, cam2_topic_, merged_topic_, target_frame_;
    bool enable_voxel_filter_;
    double voxel_leaf_size_;

    void cloudsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg, const sensor_msgs::PointCloud2ConstPtr& cloud2_msg)
    {
        ROS_DEBUG("Received synchronized point clouds.");
        pcl::ScopeTime timer("Total processing time");

        // 1. Transform Cloud 2 to the target frame
        sensor_msgs::PointCloud2 cloud2_transformed_msg;
        try
        {
            // Wait for transform availability (optional, with timeout)
            if (!tf_buffer_.canTransform(target_frame_, cloud2_msg->header.frame_id, cloud2_msg->header.stamp, ros::Duration(0.1))) {
                ROS_WARN_THROTTLE(2.0, "Cannot transform cloud2 from %s to %s. Waiting for transform.", 
                                  cloud2_msg->header.frame_id.c_str(), target_frame_.c_str());
                return;
            }
            
            // Lookup the transform
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                target_frame_, cloud2_msg->header.frame_id, cloud2_msg->header.stamp);

            // Apply the transform
            tf2::doTransform(*cloud2_msg, cloud2_transformed_msg, transform_stamped);
            cloud2_transformed_msg.header.frame_id = target_frame_; // Update frame_id after transform
            ROS_DEBUG("Successfully transformed cloud2 to target frame '%s'", target_frame_.c_str());
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2.0, "TF Exception transforming cloud2: %s", ex.what());
            return; // Skip this pair if transform fails
        }

        // 2. Process clouds
        pcl::ScopeTime cloud_processing_timer("Cloud processing");
        
        // Convert ROS messages to PCL PointClouds
        PointCloudXYZRGB::Ptr cloud1_pcl(new PointCloudXYZRGB);
        PointCloudXYZRGB::Ptr cloud2_pcl(new PointCloudXYZRGB);
        
        pcl::fromROSMsg(*cloud1_msg, *cloud1_pcl);
        pcl::fromROSMsg(cloud2_transformed_msg, *cloud2_pcl); // Use the transformed cloud 2
        
        ROS_DEBUG("Cloud1 size: %lu, Cloud2 size: %lu", 
                  cloud1_pcl->size(), cloud2_pcl->size());

        // 3. Apply downsampling using CPU VoxelGrid
        PointCloudXYZRGB::Ptr cloud1_filtered(new PointCloudXYZRGB);
        PointCloudXYZRGB::Ptr cloud2_filtered(new PointCloudXYZRGB);

        if (enable_voxel_filter_)
        {
            // Use CPU voxel filter with a safe leaf size
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
            voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

            // Filter cloud 1
            voxel_filter.setInputCloud(cloud1_pcl);
            voxel_filter.filter(*cloud1_filtered);

            // Filter cloud 2
            voxel_filter.setInputCloud(cloud2_pcl);
            voxel_filter.filter(*cloud2_filtered);
            
            ROS_DEBUG("Applied CPU voxel grid filter");
            
            ROS_DEBUG("Filtered Cloud1 size: %lu, Cloud2 size: %lu", 
                      cloud1_filtered->size(), cloud2_filtered->size());
        } else {
            // If not filtering, just pass the original pointers
            cloud1_filtered = cloud1_pcl;
            cloud2_filtered = cloud2_pcl;
        }

        // 4. Concatenate the two clouds
        PointCloudXYZRGB merged_cloud_pcl = *cloud1_filtered + *cloud2_filtered;
        ROS_DEBUG("Merged cloud size: %lu points", merged_cloud_pcl.size());

        // 5. Convert merged PCL cloud back to ROS message
        sensor_msgs::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(merged_cloud_pcl, merged_cloud_msg);

        // Set header information for the merged cloud
        merged_cloud_msg.header.stamp = ros::Time::now(); 
        merged_cloud_msg.header.frame_id = target_frame_;

        // 6. Publish the merged cloud
        merged_cloud_pub_.publish(merged_cloud_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_pointcloud_merger_node");
    PointCloudMerger merger;
    ros::spin();
    return 0;
}
