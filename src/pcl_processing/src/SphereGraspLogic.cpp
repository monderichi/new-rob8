#include "pcl_processing/SphereGraspLogic.hpp"
#include <limits> // Required for std::numeric_limits if using NaN

namespace pcl_processing {

// Constructor: Store relevant config values
SphereGraspLogic::SphereGraspLogic(const GraspPoseEstimatorConfig& config) :
    diameter_threshold_(config.sphere_diameter_threshold),
    padding_(config.grasp_size_padding),
    lateral_grasp_offset_(config.lateral_grasp_orientation_offset),
    min_wrist_angle_(config.wrist_min_angle),
    max_wrist_angle_(config.wrist_max_angle)
{}

// Select grasp type based on diameter threshold
int8_t SphereGraspLogic::selectGraspType(float radius) {
    double diameter = 2.0 * static_cast<double>(radius);
    ROS_DEBUG("Sphere: Radius=%.3f, Diameter=%.3f, Threshold=%.3f", radius, diameter, diameter_threshold_);

    if (diameter >= diameter_threshold_) {
        ROS_DEBUG("Sphere: Diameter >= Threshold -> PALMAR grasp");
        return GraspReference::GRASP_PALMAR;
    } else {
        ROS_DEBUG("Sphere: Diameter < Threshold -> LATERAL grasp");
        // Changed from GRASP_NONE in stub to LATERAL as per requirement
        return GraspReference::GRASP_LATERAL;
    }
}

float SphereGraspLogic::computeWristOrientation(int8_t grasp_type, const geometry_msgs::Point& approach_direction) {
    // Normalize approach direction in XY plane
    double approach_xy_norm = std::sqrt(approach_direction.x * approach_direction.x + 
                                       approach_direction.y * approach_direction.y);
    
    // Default orientation in case of invalid approach
    float orientation = 0.0f;
    
    if (approach_xy_norm < 1e-6) {
        // Approach is almost perfectly vertical, use default orientation
        ROS_DEBUG("Sphere: approach is vertical, using default orientation");
        if (grasp_type == GraspReference::GRASP_LATERAL) {
            orientation = lateral_grasp_offset_;
        } else {
            orientation = 0.0f;
        }
    } else {
        // Calculate base angle in XY plane
        float base_angle = std::atan2(approach_direction.y, approach_direction.x);
        
        if (grasp_type == GraspReference::GRASP_LATERAL) {
            // For lateral grasp on sphere, orient perpendicular to approach vector
            // This positions fingers naturally around the edge of the sphere
            orientation = base_angle + M_PI/2.0f + lateral_grasp_offset_;
            ROS_DEBUG("Sphere: lateral grasp, orientation = %.3f (base + π/2 + offset)", orientation);
        } else { // GRASP_PALMAR
            // For palmar grasp, align wrist with approach direction
            // This positions the palm directly facing the sphere
            orientation = base_angle;
            ROS_DEBUG("Sphere: palmar grasp, orientation = %.3f (aligned with approach)", orientation);
        }
    }
    
    // Normalize angle to [-π, π] range
    while (orientation > M_PI) orientation -= 2.0f * M_PI;
    while (orientation < -M_PI) orientation += 2.0f * M_PI;
    
    // Clamp to allowed wrist angle limits
    orientation = clamp(orientation, min_wrist_angle_, max_wrist_angle_);
    
    ROS_DEBUG("Sphere: final wrist orientation = %.3f (after clamping)", orientation);
    return orientation;
}
// Stub implementation: Return diameter + padding
float SphereGraspLogic::computeGraspSize(float radius) {
    return (2.0f * radius) + static_cast<float>(padding_);
}

// Helper function (if needed, could be in a common utility)
float SphereGraspLogic::clamp(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}

} // namespace pcl_processing