#pragma once

#include "pcl_processing/GraspPoseEstimatorConfig.hpp"
#include "pcl_processing/GraspReference.h" // For grasp type constants
#include <cmath>

namespace pcl_processing {

class SphereGraspLogic {
private:
    // Store relevant config parameters needed for sphere calculations
    double diameter_threshold_;
    double padding_;
    double lateral_grasp_offset_; // Might be needed if orientation depends on type
    double min_wrist_angle_;
    double max_wrist_angle_;


public:
    // Constructor takes the config struct to initialize its parameters
    explicit SphereGraspLogic(const GraspPoseEstimatorConfig& config);

    // Public methods for calculations specific to spheres
    int8_t selectGraspType(float radius);
    float computeWristOrientation(int8_t grasp_type, const geometry_msgs::Point& approach_direction);
    float computeGraspSize(float radius);

private:
    // Private helper methods if needed for sphere logic
    float clamp(float value, float min_val, float max_val); // Could be duplicated or put in a common utility header
};

} // namespace pcl_processing