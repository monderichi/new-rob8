#ifndef DUAL_CAMERA_SIMULATION_POINT_CLOUD_MERGER_H
#define DUAL_CAMERA_SIMULATION_POINT_CLOUD_MERGER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex>

namespace dual_camera_simulation {

class PointCloudMerger {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // Standard subscribers for the point clouds
    ros::Subscriber cloud1_sub_;
    ros::Subscriber cloud2_sub_;
    ros::Publisher merged_cloud_pub_;
    ros::Timer publish_timer_;
    
    // TF listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    std::string input_cloud1_topic_;
    std::string input_cloud2_topic_;
    std::string output_cloud_topic_;
    std::string target_frame_;
    double publish_rate_;
    
    // Last received point clouds
    sensor_msgs::PointCloud2::ConstPtr last_cloud1_;
    sensor_msgs::PointCloud2::ConstPtr last_cloud2_;
    
    // Mutex for thread safety
    std::mutex cloud_mutex_;

public:
    PointCloudMerger();
    void cloud1Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void cloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void publishTimerCallback(const ros::TimerEvent& event);
};

} // namespace dual_camera_simulation

#endif // DUAL_CAMERA_SIMULATION_POINT_CLOUD_MERGER_H