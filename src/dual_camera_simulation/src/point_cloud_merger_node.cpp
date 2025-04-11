#include "dual_camera_simulation/point_cloud_merger.h"

namespace dual_camera_simulation {

PointCloudMerger::PointCloudMerger() 
    : nh_()
    , pnh_("~")
    , tf_listener_(tf_buffer_) {
    
    // Get parameters
    pnh_.param<std::string>("input_cloud1", input_cloud1_topic_, "input_cloud1");
    pnh_.param<std::string>("input_cloud2", input_cloud2_topic_, "input_cloud2");
    pnh_.param<std::string>("output_cloud", output_cloud_topic_, "merged_cloud");
    pnh_.param<std::string>("target_frame", target_frame_, "base_link");
    pnh_.param<double>("publish_rate", publish_rate_, 10.0);
    
    // Set up publishers and subscribers
    cloud1_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(input_cloud1_topic_, 1, 
                                                        &PointCloudMerger::cloud1Callback, this);
    cloud2_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(input_cloud2_topic_, 1, 
                                                        &PointCloudMerger::cloud2Callback, this);
    merged_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
    
    // Set up timer for publishing merged cloud at fixed rate
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                  &PointCloudMerger::publishTimerCallback, this);
    
    ROS_INFO("Point Cloud Merger initialized");
    ROS_INFO_STREAM("Input cloud 1 topic: " << input_cloud1_topic_);
    ROS_INFO_STREAM("Input cloud 2 topic: " << input_cloud2_topic_);
    ROS_INFO_STREAM("Output merged cloud topic: " << output_cloud_topic_);
    ROS_INFO_STREAM("Target frame: " << target_frame_);
    ROS_INFO_STREAM("Publish rate: " << publish_rate_ << " Hz");
}

void PointCloudMerger::cloud1Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    last_cloud1_ = cloud_msg;
}

void PointCloudMerger::cloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    last_cloud2_ = cloud_msg;
}

void PointCloudMerger::publishTimerCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    
    // Check if we have received both point clouds
    if (!last_cloud1_ || !last_cloud2_) {
        return;
    }
    
    try {
        // Transform both point clouds to the target frame
        sensor_msgs::PointCloud2 transformed_cloud1;
        sensor_msgs::PointCloud2 transformed_cloud2;
        
        // Use tf2 to transform the point clouds
        tf_buffer_.transform(*last_cloud1_, transformed_cloud1, target_frame_);
        tf_buffer_.transform(*last_cloud2_, transformed_cloud2, target_frame_);
        
        // Convert to PCL point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        pcl::fromROSMsg(transformed_cloud1, *pcl_cloud1);
        pcl::fromROSMsg(transformed_cloud2, *pcl_cloud2);
        
        // Merge the point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *merged_cloud = *pcl_cloud1;
        *merged_cloud += *pcl_cloud2;
        
        // Apply voxel grid filter to downsample the merged cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setInputCloud(merged_cloud);
        voxel.setLeafSize(0.01, 0.01, 0.01); // 1cm voxel size
        voxel.filter(*filtered_merged_cloud);
        
        // Convert back to ROS message
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_merged_cloud, output_msg);
        output_msg.header.stamp = ros::Time::now();
        output_msg.header.frame_id = target_frame_;
        
        // Publish the merged point cloud
        merged_cloud_pub_.publish(output_msg);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(5.0, "Could not transform point clouds: %s", ex.what());
    }
}

} // namespace dual_camera_simulation

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_merger_node");
    dual_camera_simulation::PointCloudMerger merger;
    ros::spin();
    return 0;
}