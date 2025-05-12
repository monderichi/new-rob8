#pragma once

#include "pcl_processing/GraspPoseEstimatorConfig.hpp"
#include "pcl_processing/GraspReference.h" // For grasp type constants
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>
#include <numeric>   // For std::iota
#include <algorithm> // For std::sort, std::max_element
#include <limits>

namespace pcl_processing {

class BoxGraspLogic {
private:
    // Store relevant config parameters
    double length_threshold_;
    double width_threshold_;
    double padding_;
    double lateral_grasp_offset_;
    double min_wrist_angle_;
    double max_wrist_angle_;

    struct TargetFaceInfo {
        bool valid = false;          // Was a valid face found?
        int axis_index = -1;         // Index (0, 1, or 2) of the normal axis
        float face_length = 0.0f;    // Longer dimension of the target face
        float face_width = 0.0f;     // Shorter dimension of the target face
        // geometry_msgs::Vector3 normal; // Could store the normal vector too
    };

    // --- Private Helper Struct for Candidate Faces ---
    struct CandidateFace {
        int axis_index; // Index of the normal axis (0, 1, or 2)
        int side; // Side relative to box center (+1 or -1) along normal axis
        geometry_msgs::Point center; // p0
        geometry_msgs::Vector3 normal; // n (outward pointing)
        geometry_msgs::Vector3 u_axis; // Tangent axis 1
        geometry_msgs::Vector3 v_axis; // Tangent axis 2
        double dim_u; // Dimension along u_axis
        double dim_v; // Dimension along v_axis
        double distance_sq; // Squared distance from center to Z-intersection
        bool rejected = false; // Flag for rejection based on step 4
        bool processed = false; // Flag to indicate if steps 2,3 were done successfully
        geometry_msgs::Point intersection_point; // Store intersection point for step 4
    };


public:
    // Constructor takes the config struct
    explicit BoxGraspLogic(const GraspPoseEstimatorConfig& config);

    // Public methods for calculations specific to boxes
    // Assumes dimensions correspond to axes in the same order
    int8_t selectGraspType(const geometry_msgs::Point& center, const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions);
    float computeWristOrientation(const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions, int8_t grasp_type);
    float computeGraspSize(const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions);

private:
    // Private helper methods
    float clamp(float value, float min_val, float max_val);
    // Helper to determine the face being grasped and its properties
    // BoxFaceAxis getGraspingFaceAxis(const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions);
    // Potentially add vector math helpers if needed

    // Selects the face aimed at by the camera's Z-axis (assumed to be 0,0,t)
    TargetFaceInfo selectTargetFace(const geometry_msgs::Point& center, const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions);

    // Helper for dot product (optional, but can make code cleaner)
    static double dot(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b);
    static double dot(const geometry_msgs::Vector3& v, const geometry_msgs::Point& p); // For n · p0
    
};

} // namespace pcl_processing