#pragma once

#include "pcl_processing/GraspPoseEstimatorConfig.hpp"
#include "pcl_processing/GraspReference.h" // For grasp type constants
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h> // Needed for approach angle calculation if view vector is involved
#include <cmath>

namespace pcl_processing {

class CylinderGraspLogic {
private:
    // Store relevant config parameters
    double approach_angle_threshold_;
    double length_threshold_;
    double diameter_threshold_;
    double padding_;
    double lateral_grasp_offset_;
    double min_wrist_angle_;
    double max_wrist_angle_;

public:
    // Constructor takes the config struct
    explicit CylinderGraspLogic(const GraspPoseEstimatorConfig& config);

    // Public methods for calculations specific to cylinders
    // Note: Approach angle calculation might need the camera view vector,
    // which isn't directly in the primitive message. This might need adjustment
    // or be based solely on orientation relative to a world frame.
    // Assuming for now it's based on axis orientation relative to some frame.
    int8_t selectGraspType(float radius, float length, const geometry_msgs::Vector3& axis);
    float computeWristOrientation(const geometry_msgs::Vector3& axis, int8_t grasp_type);
    float computeGraspSize(float radius);

private:
    // Private helper methods
    float clamp(float value, float min_val, float max_val);
    // Potentially add vector math helpers if needed
    double calculateApproachAngle(const geometry_msgs::Vector3& axis);
};

} // namespace pcl_processing