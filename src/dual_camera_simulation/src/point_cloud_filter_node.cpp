#include "dual_camera_simulation/point_cloud_filter.h"

namespace dual_camera_simulation {

PointCloudFilter::PointCloudFilter() : nh_(), pnh_("~") {
    // Get parameters
    pnh_.param<std::string>("input_cloud", input_cloud_topic_, "input_cloud");
    pnh_.param<std::string>("output_cloud", output_cloud_topic_, "output_cloud");
    pnh_.param<std::string>("camera_name", camera_name_, "camera");
    
    // Load filter parameters from parameter server
    std::string param_ns = "/" + camera_name_ + "/filter/";
    
    // Default filter parameters if not found on parameter server
    pnh_.param(param_ns + "passthrough/min_depth", pass_min_depth_, 0.1);
    pnh_.param(param_ns + "passthrough/max_depth", pass_max_depth_, 3.0);
    pnh_.param(param_ns + "voxel_grid/leaf_size", voxel_leaf_size_, 0.01);
    pnh_.param(param_ns + "statistical_outlier/mean_k", sor_mean_k_, 50);
    pnh_.param(param_ns + "statistical_outlier/std_dev", sor_std_dev_, 1.0);
    
    // Set up publisher and subscriber
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(input_cloud_topic_, 1, &PointCloudFilter::cloudCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
    
    ROS_INFO_STREAM("Point Cloud Filter initialized for camera: " << camera_name_);
    ROS_INFO_STREAM("Input topic: " << input_cloud_topic_);
    ROS_INFO_STREAM("Output topic: " << output_cloud_topic_);
    ROS_INFO_STREAM("PassThrough filter range: [" << pass_min_depth_ << ", " << pass_max_depth_ << "]");
    ROS_INFO_STREAM("VoxelGrid leaf size: " << voxel_leaf_size_);
    ROS_INFO_STREAM("StatisticalOutlierRemoval params: mean_k=" << sor_mean_k_ << ", std_dev=" << sor_std_dev_);
}

void PointCloudFilter::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // Apply PassThrough filter to remove points outside specified depth range
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_pass(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(pass_min_depth_, pass_max_depth_);
    pass.filter(*cloud_filtered_pass);
    
    // Apply VoxelGrid filter to downsample the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud_filtered_pass);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(*cloud_filtered_voxel);
    
    // Apply Statistical Outlier Removal filter to remove noise
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_sor(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_filtered_voxel);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_std_dev_);
    sor.filter(*cloud_filtered_sor);
    
    // Convert back to ROS message
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_filtered_sor, output_msg);
    output_msg.header = cloud_msg->header;
    
    // Publish the filtered point cloud
    cloud_pub_.publish(output_msg);
}

} // namespace dual_camera_simulation

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_filter_node");
    dual_camera_simulation::PointCloudFilter filter;
    ros::spin();
    return 0;
}