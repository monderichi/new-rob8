#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "dual_camera_simulation/camera_calibration.h"

namespace dual_camera_simulation {

CameraCalibration::CameraCalibration() 
    : nh_()
    , pnh_("~")
    , tf_listener_(tf_buffer_)
    , calibration_complete_(false) {
    
    // Get parameters
    pnh_.param<std::string>("marker_frame", marker_frame_, "aruco_marker_23");
    pnh_.param<std::string>("camera1_frame", camera1_frame_, "camera1_link");
    pnh_.param<std::string>("camera2_frame", camera2_frame_, "camera2_link");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<double>("publish_rate", publish_rate_, 10.0);
    
    // Start timer for periodic processing
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &CameraCalibration::timerCallback, this);
    
    ROS_INFO("Camera calibration node initialized");
    ROS_INFO_STREAM("Marker frame: " << marker_frame_);
    ROS_INFO_STREAM("Camera 1 frame: " << camera1_frame_);
    ROS_INFO_STREAM("Camera 2 frame: " << camera2_frame_);
    ROS_INFO_STREAM("Base frame: " << base_frame_);
    ROS_INFO_STREAM("Publish rate: " << publish_rate_ << " Hz");
}

void CameraCalibration::timerCallback(const ros::TimerEvent& event) {
    if (!calibration_complete_) {
        // Try to calibrate cameras using the marker
        calibrateCameras();
    } else {
        // Just publish the calibrated transform
        publishCalibrationTransform();
    }
}

void CameraCalibration::calibrateCameras() {
    geometry_msgs::TransformStamped camera1_to_marker;
    
    try {
        // Get the transform from camera 1 to marker
        camera1_to_marker = tf_buffer_.lookupTransform(
            camera1_frame_, marker_frame_, ros::Time(0), ros::Duration(1.0));
            
        ROS_INFO_ONCE("Found transform from camera1 to marker");
        
        // In a real system, we'd get the transform from marker to camera2
        // directly, but in our simulation we know the marker is attached to camera2
        
        // Since the marker is attached to the back of camera2, we know its position
        // relative to camera2: small offset on the x-axis (usually the depth direction)
        camera2_to_marker_.header.stamp = ros::Time::now();
        camera2_to_marker_.header.frame_id = camera2_frame_;
        camera2_to_marker_.child_frame_id = marker_frame_;
        camera2_to_marker_.transform.translation.x = -0.013; // From the URDF
        camera2_to_marker_.transform.translation.y = 0.0;
        camera2_to_marker_.transform.translation.z = 0.0;
        camera2_to_marker_.transform.rotation.w = 1.0;
        camera2_to_marker_.transform.rotation.x = 0.0;
        camera2_to_marker_.transform.rotation.y = 0.0;
        camera2_to_marker_.transform.rotation.z = 0.0;
        
        calibration_complete_ = true;
        
        ROS_INFO("Camera calibration completed successfully!");
        
        // Publish the calibrated transform for the first time
        publishCalibrationTransform();
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(5.0, "Could not get transform from camera1 to marker: %s", ex.what());
    }
}

void CameraCalibration::publishCalibrationTransform() {
    if (!calibration_complete_) {
        return;
    }
    
    try {
        // Get the current transform from camera 1 to marker
        geometry_msgs::TransformStamped camera1_to_marker = tf_buffer_.lookupTransform(
            camera1_frame_, marker_frame_, ros::Time(0), ros::Duration(0.1));
        
        // Compute the transform from camera1 to camera2
        // T_camera1_camera2 = T_camera1_marker * T_marker_camera2
        // T_marker_camera2 = inverse(T_camera2_marker)
        
        // Convert to tf2::Transform objects for easier manipulation
        tf2::Transform tf_camera1_marker;
        tf2::fromMsg(camera1_to_marker.transform, tf_camera1_marker);
        
        tf2::Transform tf_camera2_marker;
        tf2::fromMsg(camera2_to_marker_.transform, tf_camera2_marker);
        
        // Get inverse (marker to camera2)
        tf2::Transform tf_marker_camera2 = tf_camera2_marker.inverse();
        
        // Compute the transform from camera1 to camera2
        tf2::Transform tf_camera1_camera2 = tf_camera1_marker * tf_marker_camera2;
        
        // Convert back to geometry_msgs::TransformStamped
        geometry_msgs::TransformStamped camera1_to_camera2;
        camera1_to_camera2.header.stamp = ros::Time::now();
        camera1_to_camera2.header.frame_id = camera1_frame_;
        camera1_to_camera2.child_frame_id = camera2_frame_;
        camera1_to_camera2.transform = tf2::toMsg(tf_camera1_camera2);
        
        // Broadcast the transform
        tf_broadcaster_.sendTransform(camera1_to_camera2);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to publish calibration transform: %s", ex.what());
    }
}

} // namespace dual_camera_simulation

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_calibration_node");
    dual_camera_simulation::CameraCalibration calibration;
    ros::spin();
    return 0;
}