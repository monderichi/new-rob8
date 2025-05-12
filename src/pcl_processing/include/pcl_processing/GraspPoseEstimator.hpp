#pragma once

#include <ros/ros.h>
#include <vector>
#include <deque>
#include <string>
#include <memory> // For unique_ptr if needed, or just direct members

#include "pcl_processing/GeometricPrimitive.h"
#include "pcl_processing/GraspReference.h"
#include "pcl_processing/GraspPoseEstimatorConfig.hpp"
// Include headers for the new logic classes
#include "pcl_processing/SphereGraspLogic.hpp"
#include "pcl_processing/CylinderGraspLogic.hpp"
#include "pcl_processing/BoxGraspLogic.hpp"

namespace pcl_processing {

class GraspPoseEstimator {
public:
    GraspPoseEstimator(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber primitive_sub_;
    ros::Publisher grasp_pub_;

    GraspPoseEstimatorConfig config_;

    // Instances of the shape-specific logic handlers
    SphereGraspLogic sphere_logic_;
    CylinderGraspLogic cylinder_logic_;
    BoxGraspLogic box_logic_;

    // History for smoothing
    std::deque<int8_t> grasp_type_history_;
    std::deque<float> grasp_size_history_;
    std::deque<float> wrist_orientation_history_;

    // --- Main Callback ---
    void primitiveCallback(const pcl_processing::GeometricPrimitive::ConstPtr& msg);

    // --- Shape Agnostic Calculation Steps (Orchestration) ---
    // These delegate to the appropriate logic class instance
    int8_t selectGraspType(const pcl_processing::GeometricPrimitive::ConstPtr& msg);
    float computeWristOrientation(const pcl_processing::GeometricPrimitive::ConstPtr& msg, int8_t grasp_type);
    float computeGraspSize(const pcl_processing::GeometricPrimitive::ConstPtr& msg);

    // --- Smoothing Logic ---
    int8_t getMajorityGraspType();
    float getMovingAverage(const std::deque<float>& values);

    // --- Helper Functions ---
    void updateHistory(int8_t type, float size, float orientation);
    pcl_processing::GraspReference smoothGrasp();
    float clamp(float value, float min_val, float max_val);
};

} // namespace pcl_processing