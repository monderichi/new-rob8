#ifndef DUAL_CAMERA_SIMULATION_CAMERA_CALIBRATION_H
#define DUAL_CAMERA_SIMULATION_CAMERA_CALIBRATION_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

namespace dual_camera_simulation {

class CameraCalibration {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    
    std::string marker_frame_;
    std::string camera1_frame_;
    std::string camera2_frame_;
    std::string base_frame_;
    
    bool calibration_complete_;
    geometry_msgs::TransformStamped camera2_to_marker_;
    
    double publish_rate_; // Rate at which to publish transforms (Hz)

public:
    CameraCalibration();
    void timerCallback(const ros::TimerEvent& event);
    void calibrateCameras();
    void publishCalibrationTransform();
};

} // namespace dual_camera_simulation

#endif // DUAL_CAMERA_SIMULATION_CAMERA_CALIBRATION_H