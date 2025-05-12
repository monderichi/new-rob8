#include "pcl_processing/CylinderGraspLogic.hpp"
#include <limits> // Required for std::numeric_limits if using NaN
#include <ros/ros.h>
#include <cmath>

namespace pcl_processing {

// Constructor: Store relevant config values
CylinderGraspLogic::CylinderGraspLogic(const GraspPoseEstimatorConfig& config) :
    approach_angle_threshold_(config.cylinder_approach_angle_threshold),
    length_threshold_(config.cylinder_length_threshold),
    diameter_threshold_(config.cylinder_diameter_threshold),
    padding_(config.grasp_size_padding),
    lateral_grasp_offset_(config.lateral_grasp_orientation_offset),
    min_wrist_angle_(config.wrist_min_angle),
    max_wrist_angle_(config.wrist_max_angle)
{}

// Select grasp type based on dimensions and approach angle
int8_t CylinderGraspLogic::selectGraspType(float radius, float length, const geometry_msgs::Vector3& axis) {
    double diameter = 2.0 * static_cast<double>(radius);
    ROS_DEBUG("Cylinder: Radius=%.3f, Length=%.3f, Diameter=%.3f", radius, length, diameter);
    ROS_DEBUG("Cylinder Thresholds: Length=%.3f, Diameter=%.3f, Angle=%.3f rad",
              length_threshold_, diameter_threshold_, approach_angle_threshold_);

    if (static_cast<double>(length) <= length_threshold_) {
        ROS_DEBUG("Cylinder: Length <= Threshold -> PALMAR grasp");
        return GraspReference::GRASP_PALMAR;
    } else {
        ROS_DEBUG("Cylinder: Length > Threshold");
        if (diameter < diameter_threshold_) {
            ROS_DEBUG("Cylinder: Diameter < Threshold -> LATERAL grasp");
            return GraspReference::GRASP_LATERAL;
        } else {
            ROS_DEBUG("Cylinder: Diameter >= Threshold -> Check approach angle");

            // Call the helper function to get the angle
            double angle = calculateApproachAngle(axis);

            // Check if the angle calculation was valid
            if (std::isnan(angle)) {
                ROS_WARN("Invalid approach angle calculated, defaulting grasp type.");
                // Decide on a default behavior: maybe PALMAR is safer? Or NONE?
                return GraspReference::GRASP_NONE; // Or GRASP_PALMAR
            }

            // Compare angle directly with the threshold
            if (angle < approach_angle_threshold_) {
                ROS_DEBUG("Cylinder: Approach angle < Threshold -> LATERAL grasp");
                return GraspReference::GRASP_LATERAL;
            } else {
                ROS_DEBUG("Cylinder: Approach angle >= Threshold -> PALMAR grasp");
                return GraspReference::GRASP_PALMAR;
            }
        }
    }
}

// --- Private Helper: Calculate Approach Angle ---
double CylinderGraspLogic::calculateApproachAngle(const geometry_msgs::Vector3& axis) {
    // Calculate angle between cylinder axis and camera Z-axis [0,0,1]
    double axis_norm = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (axis_norm < 1e-6) { // Avoid division by zero / handle invalid axis
        ROS_WARN("Cylinder axis norm is near zero. Cannot compute approach angle.");
        // Return an invalid angle (e.g., NaN or a negative value) to indicate failure
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Cosine of the angle between axis and (0,0,1) is axis.z / axis_norm
    double cos_angle = axis.z / axis_norm;
    // Clamp cos_angle to [-1, 1] due to potential floating point inaccuracies
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    double angle = std::acos(cos_angle); // Angle in radians [0, PI]

    ROS_DEBUG("Calculated Approach Angle = %.3f rad", angle);
    return angle;
}

float CylinderGraspLogic::computeWristOrientation(const geometry_msgs::Vector3& axis, int8_t grasp_type) {
    // Normalize the axis
    double axis_norm = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (axis_norm < 1e-6) {
        ROS_WARN("Cylinder axis norm is near zero. Using default orientation.");
        return 0.0f;
    }
    
    // Project axis onto XY plane to determine its base orientation
    double axis_xy_norm = std::sqrt(axis.x * axis.x + axis.y * axis.y);
    float orientation = 0.0f;
    
    // Special case: Cylinder is almost perfectly vertical
    if (axis_xy_norm < 1e-6) {
        ROS_DEBUG("Cylinder is vertical (along Z-axis)");
        // For vertical cylinders, use default or offset orientation
        return (grasp_type == GraspReference::GRASP_LATERAL) ? lateral_grasp_offset_ : 0.0f;
    }
    
    // Calculate base angle in XY plane
    float base_angle = std::atan2(axis.y, axis.x);
    
    // Apply grasp-specific orientation
    if (grasp_type == GraspReference::GRASP_LATERAL) {
        // For lateral: perpendicular to cylinder axis + offset
        orientation = base_angle + M_PI/2.0f + lateral_grasp_offset_;
        ROS_DEBUG("LATERAL grasp: orienting perpendicular to cylinder (%.3f + π/2 + %.3f)",
                 base_angle, lateral_grasp_offset_);
    } 
    else { // PALMAR or other
        // For palmar: align with cylinder axis
        orientation = base_angle;
        ROS_DEBUG("PALMAR grasp: orienting along cylinder axis (%.3f)", base_angle);
    }
    
    // Normalize angle to [-π, π] range
    while (orientation > M_PI) orientation -= 2.0f * M_PI;
    while (orientation < -M_PI) orientation += 2.0f * M_PI;
    
    // Clamp to allowed wrist angle limits
    orientation = clamp(orientation, min_wrist_angle_, max_wrist_angle_);
    
    ROS_DEBUG("Final wrist orientation: %.3f rad (clamped to [%.3f, %.3f])", 
             orientation, min_wrist_angle_, max_wrist_angle_);
             
    return orientation;
}

// Stub implementation: Return diameter + padding
float CylinderGraspLogic::computeGraspSize(float radius) {
    return (2.0f * radius) + static_cast<float>(padding_);
}

// Helper function
float CylinderGraspLogic::clamp(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}

} // namespace pcl_processing