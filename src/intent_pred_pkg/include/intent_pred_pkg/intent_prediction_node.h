#ifndef INTENT_PREDICTION_NODE_H
#define INTENT_PREDICTION_NODE_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

#include "intent_prediction_calc.h"

class IntentPredictor
{
public:
    IntentPredictor();
    virtual ~IntentPredictor() = default;

private:
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Subscribers and publishers
    ros::Subscriber centroid_sub_;
    ros::Publisher intent_pub_;
    
    // Timers
    ros::Timer debug_timer_;
    
    // TF2 objects
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Frame names
    std::string hand_depth_frame_;
    std::string head_depth_frame_;
    
    // Intent prediction algorithm
    std::unique_ptr<IntentPredictCalc> intent_predict_calc_;
    
    // Callback functions
    void centroidCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_array);
    void debugTransformsCallback(const ros::TimerEvent& event);
    
    // Helper functions
    bool waitForTransforms();
};

#endif // INTENT_PREDICTION_NODE_H