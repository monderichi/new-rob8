#ifndef DUAL_CAMERA_SIMULATION_POINT_CLOUD_FILTER_H
#define DUAL_CAMERA_SIMULATION_POINT_CLOUD_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace dual_camera_simulation {

class PointCloudFilter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string camera_name_;
    
    // Filter parameters
    double pass_min_depth_;
    double pass_max_depth_;
    double voxel_leaf_size_;
    int sor_mean_k_;
    double sor_std_dev_;

public:
    PointCloudFilter();
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};

} // namespace dual_camera_simulation

#endif // DUAL_CAMERA_SIMULATION_POINT_CLOUD_FILTER_H