#include "pcl_processing/BoxGraspLogic.hpp"
#include <limits> // Required for std::numeric_limits if using NaN

namespace pcl_processing {

// --- Helper Functions for Vector/Point Math ---
// (These could be moved to a separate utility header if used elsewhere)
inline geometry_msgs::Point add(const geometry_msgs::Point& p, const geometry_msgs::Vector3& v) {
    geometry_msgs::Point result;
    result.x = p.x + v.x;
    result.y = p.y + v.y;
    result.z = p.z + v.z;
    return result;
}

inline geometry_msgs::Vector3 subtract(const geometry_msgs::Point& p1, const geometry_msgs::Point& p0) {
    geometry_msgs::Vector3 result;
    result.x = p1.x - p0.x;
    result.y = p1.y - p0.y;
    result.z = p1.z - p0.z;
    return result;
}

inline geometry_msgs::Vector3 scale(const geometry_msgs::Vector3& v, double s) {
    geometry_msgs::Vector3 result;
    result.x = v.x * s;
    result.y = v.y * s;
    result.z = v.z * s;
    return result;
}

inline double magnitudeSq(const geometry_msgs::Vector3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

// Constructor: Store relevant config values
BoxGraspLogic::BoxGraspLogic(const GraspPoseEstimatorConfig& config) :
    length_threshold_(config.box_length_threshold),
    width_threshold_(config.box_width_threshold),
    padding_(config.grasp_size_padding),
    lateral_grasp_offset_(config.lateral_grasp_orientation_offset),
    min_wrist_angle_(config.wrist_min_angle),
    max_wrist_angle_(config.wrist_max_angle)
{}

// --- Public Method: Select Grasp Type ---
// Now takes center and axes as input
int8_t BoxGraspLogic::selectGraspType(const geometry_msgs::Point& center, const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions) {

    // 1. Select the target face
    TargetFaceInfo face_info = selectTargetFace(center, axes, dimensions);

    // 2. Check if a valid face was found
    if (!face_info.valid) {
        ROS_WARN("Box Grasp Type: No valid target face found. Returning GRASP_NONE.");
        return GraspReference::GRASP_NONE;
    }

    // 3. Decide on the grasp type based on face dimensions
    ROS_DEBUG("Box Grasp Type: Using face L=%.3f, W=%.3f. Thresholds L=%.3f, W=%.3f",
              face_info.face_length, face_info.face_width, length_threshold_, width_threshold_);

    // Replace this with your actual logic based on l, w, and thresholds
    if (face_info.face_length >= length_threshold_) {
        ROS_DEBUG(" -> Decision: LATERAL");
        return GraspReference::GRASP_PALMAR;
    } else {
        if (face_info.face_width >= width_threshold_) {
            ROS_DEBUG(" -> Decision: PALMAR");
            return GraspReference::GRASP_PALMAR;
        } else {
            ROS_DEBUG(" -> Decision: GRASP_NONE");
            return GraspReference::GRASP_LATERAL;
        }
    }
}

float BoxGraspLogic::computeWristOrientation(const std::vector<geometry_msgs::Vector3>& axes, 
                                            const std::vector<float>& dimensions, 
                                            int8_t grasp_type) {
    // Get target face information
    geometry_msgs::Point center;
    
    // Get target face information
    TargetFaceInfo face_info = selectTargetFace(center, axes, dimensions);
    
    if (!face_info.valid) {
        ROS_WARN("Cannot compute wrist orientation: No valid target face found");
        return 0.0f;
    }
    
    // Get face normal axis and the in-plane axes
    int normal_idx = face_info.axis_index;
    int u_idx = (normal_idx + 1) % 3;
    int v_idx = (normal_idx + 2) % 3;
    
    // Calculate the orientation based on grasp type
    float orientation = 0.0f;
    
    if (grasp_type == GraspReference::GRASP_PALMAR) {
        // For palmar grasp: Align with the longer dimension of the face
        if (dimensions[u_idx] > dimensions[v_idx]) {
            // Calculate angle between the u axis projection onto XY plane and X axis
            float u_x = axes[u_idx].x;
            float u_y = axes[u_idx].y;
            orientation = atan2(u_y, u_x);
        } else {
            // Calculate angle between the v axis projection onto XY plane and X axis
            float v_x = axes[v_idx].x;
            float v_y = axes[v_idx].y;
            orientation = atan2(v_y, v_x);
        }
    } 
    else if (grasp_type == GraspReference::GRASP_LATERAL) {
        // For lateral grasp: Align with the shorter dimension + offset
        if (dimensions[u_idx] <= dimensions[v_idx]) {
            // Calculate angle between the u axis projection onto XY plane and X axis
            float u_x = axes[u_idx].x;
            float u_y = axes[u_idx].y;
            orientation = atan2(u_y, u_x) + lateral_grasp_offset_;
        } else {
            // Calculate angle between the v axis projection onto XY plane and X axis
            float v_x = axes[v_idx].x;
            float v_y = axes[v_idx].y;
            orientation = atan2(v_y, v_x) + lateral_grasp_offset_;
        }
    }
    
    // Normalize the angle to be within min/max limits
    while (orientation > M_PI) orientation -= 2.0f * M_PI;
    while (orientation < -M_PI) orientation += 2.0f * M_PI;
    
    // Clamp the orientation within the allowed wrist angle range
    orientation = clamp(orientation, min_wrist_angle_, max_wrist_angle_);
    
    ROS_DEBUG("Computed wrist orientation: %.3f rad (%.1f deg) for grasp type %d",
              orientation, orientation * 180.0f / M_PI, grasp_type);
              
    return orientation;
}


float BoxGraspLogic::computeGraspSize(const std::vector<geometry_msgs::Vector3>& axes, const std::vector<float>& dimensions) {
    
    geometry_msgs::Point center;
    
    // Use the target face width + padding for a more accurate grasp size
    TargetFaceInfo face_info = selectTargetFace(center, axes, dimensions);
    
    if (face_info.valid) {
        // Use the face width (smaller dimension of the face) for grasping
        ROS_DEBUG("Computing grasp size using target face width %.3f + padding %.3f", 
                 face_info.face_width, static_cast<float>(padding_));
        return face_info.face_width + static_cast<float>(padding_);
    }
    
    // Fallback to previous method if no valid face was found
    if (dimensions.size() >= 3) {
        float min_dim = dimensions[0];
        if (dimensions[1] < min_dim) min_dim = dimensions[1];
        if (dimensions[2] < min_dim) min_dim = dimensions[2];
        
        ROS_DEBUG("No valid target face found, using minimum dimension %.3f + padding %.3f", 
                 min_dim, static_cast<float>(padding_));
        return min_dim + static_cast<float>(padding_);
    }

    // If dimensions vector is too small, return a default
    return static_cast<float>(padding_);

}

// Helper function
float BoxGraspLogic::clamp(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}

BoxGraspLogic::TargetFaceInfo BoxGraspLogic::selectTargetFace(
    const geometry_msgs::Point& center,
    const std::vector<geometry_msgs::Vector3>& axes,
    const std::vector<float>& dimensions)
{
    TargetFaceInfo best_face_info; // Final result, starts invalid
    std::vector<CandidateFace> candidates(3); // Size 3 directly
    const double ZERO_THRESHOLD = 1e-6; // Threshold for checking near-zero values

    if (axes.size() != 3 || dimensions.size() != 3) {
        ROS_WARN("Box Logic: Invalid axes or dimensions size for face selection.");
        return best_face_info; // Return invalid result
    }

    ROS_DEBUG("Box Face Selection: Center(%.2f, %.2f, %.2f)", center.x, center.y, center.z);

    // --- Step 1: Find the 3 closest faces and their properties ---
    for (int i = 0; i < 3; ++i) {
        const geometry_msgs::Vector3& n_axis = axes[i];
        const double dim_n = static_cast<double>(dimensions[i]);

        geometry_msgs::Point p0_pos = add(center, scale(n_axis, +dim_n / 2.0));
        geometry_msgs::Point p0_neg = add(center, scale(n_axis, -dim_n / 2.0));

        CandidateFace& face = candidates[i]; // Use reference
        face.axis_index = i;
        face.u_axis = axes[(i + 1) % 3];
        face.v_axis = axes[(i + 2) % 3];
        face.dim_u = static_cast<double>(dimensions[(i + 1) % 3]);
        face.dim_v = static_cast<double>(dimensions[(i + 2) % 3]);

        // Assuming camera near origin, smaller Z is closer
        if (p0_pos.z < p0_neg.z) {
            face.center = p0_pos;
            face.normal = n_axis;
            face.side = +1; // Store the side
        } else {
            face.center = p0_neg;
            face.normal = scale(n_axis, -1.0);
            face.side = -1; // Store the side
        }
        ROS_DEBUG(" Found candidate face %d: Side=%d, Center(%.2f, %.2f, %.2f)",
                  i, face.side, face.center.x, face.center.y, face.center.z);
    }

    // --- Steps 2, 3: Intersection, Distance for each candidate ---
    for (CandidateFace& face : candidates) {
        ROS_DEBUG(" Processing Candidate Face %d:", face.axis_index);

        // Check if the normal is parallel to the Z-axis
        if (std::abs(face.normal.z) < ZERO_THRESHOLD) {
            ROS_DEBUG("  Normal is parallel to Z-axis (n_z=%.4f). Rejecting.", face.normal.z);
            face.rejected = true;
            face.processed = false; // Mark as not successfully processed
            continue;
        }

        // Step 2: Find intersection point 'I' with Z-axis
        double t = dot(face.normal, face.center) / face.normal.z;
        face.intersection_point.x = 0.0;
        face.intersection_point.y = 0.0;
        face.intersection_point.z = t;
        ROS_DEBUG("  Intersection Point I(0, 0, %.3f)", t);

        // Step 3: Calculate distance vector and squared distance
        geometry_msgs::Vector3 dist_vec = subtract(face.intersection_point, face.center);
        face.distance_sq = magnitudeSq(dist_vec);
        ROS_DEBUG("  Distance^2 = %.4f", face.distance_sq);
        face.processed = true; // Mark as processed for steps 2, 3
    }

    // --- Step 4: Rejection logic based on adjacent face centers ---
    for (int i = 0; i < 3; ++i) {
        CandidateFace& current_face = candidates[i];
        if (!current_face.processed) continue; // Skip if failed processing earlier

        ROS_DEBUG(" Applying Rejection Logic to Face %d:", current_face.axis_index);

        // Find the other two candidates (adjacent faces)
        const CandidateFace& face_j = candidates[(i + 1) % 3]; // Normal axis is current_face.u_axis
        const CandidateFace& face_k = candidates[(i + 2) % 3]; // Normal axis is current_face.v_axis

        // Projected coordinates of adjacent centers in current_face's frame
        double adj_center_u = face_j.side * current_face.dim_u / 2.0;
        double adj_center_v = face_k.side * current_face.dim_v / 2.0;
        ROS_DEBUG("  Adjacent centers projected: U-axis at %.3f, V-axis at %.3f", adj_center_u, adj_center_v);

        // Calculate distance vector from current face center to intersection point
        geometry_msgs::Vector3 dist_vec = subtract(current_face.intersection_point, current_face.center);

        // Calculate projections of intersection point in current_face's frame
        double proj_u = dot(dist_vec, current_face.u_axis);
        double proj_v = dot(dist_vec, current_face.v_axis);
        ROS_DEBUG("  Intersection Projections: proj_u=%.3f, proj_v=%.3f", proj_u, proj_v);

        // Check rejection condition: Is intersection outside AND beyond adjacent center?
        bool reject_u = (std::abs(proj_u) > current_face.dim_u/2.0 + ZERO_THRESHOLD &&
                         proj_u * adj_center_u > std::pow(current_face.dim_u/2.0, 2) + ZERO_THRESHOLD); // Added threshold for comparison robustness

        bool reject_v = (std::abs(proj_v) > current_face.dim_v/2.0 + ZERO_THRESHOLD &&
                         proj_v * adj_center_v > std::pow(current_face.dim_v/2.0, 2) + ZERO_THRESHOLD); // Added threshold

        if (reject_u || reject_v) {
            ROS_DEBUG("  Face %d REJECTED. Reason: %s%s", current_face.axis_index,
                      (reject_u ? "Beyond U-adjacent " : ""), (reject_v ? "Beyond V-adjacent" : ""));
            current_face.rejected = true;
        } else {
            ROS_DEBUG("  Face %d Kept (Intersection OK relative to adjacent centers).", current_face.axis_index);
            current_face.rejected = false; // Ensure it's false if not rejected
        }
    }

    // --- Step 5: Select the non-rejected face with the shortest distance vector ---
    double min_valid_distance_sq = std::numeric_limits<double>::max();
    int best_candidate_index = -1;

    for (int i = 0; i < candidates.size(); ++i) {
        const auto& face = candidates[i];
        if (face.processed && !face.rejected) { // Must be processed and not rejected
            ROS_DEBUG(" Considering valid face %d, dist^2=%.4f", face.axis_index, face.distance_sq);
            if (face.distance_sq < min_valid_distance_sq) {
                min_valid_distance_sq = face.distance_sq;
                best_candidate_index = i;
                ROS_DEBUG("  -> New best candidate.");
            }
        } else {
             ROS_DEBUG(" Skipping face %d (Processed: %d, Rejected: %d).",
                       face.axis_index, face.processed, face.rejected);
        }
    }

    // Populate the final result
    if (best_candidate_index != -1) {
        const auto& best_candidate = candidates[best_candidate_index];
        best_face_info.valid = true;
        best_face_info.axis_index = best_candidate.axis_index;
        best_face_info.face_length = static_cast<float>(std::max(best_candidate.dim_u, best_candidate.dim_v));
        best_face_info.face_width = static_cast<float>(std::min(best_candidate.dim_u, best_candidate.dim_v));

        ROS_INFO("Box Logic: Selected target face with normal axis index %d (L=%.3f, W=%.3f). Min valid dist^2=%.4f",
                 best_face_info.axis_index, best_face_info.face_length, best_face_info.face_width, min_valid_distance_sq);
    } else {
        ROS_WARN("Box Logic: No valid target face found after applying rejection criteria.");
        best_face_info.valid = false;
    }

    return best_face_info;
}

// --- Dot Product Helpers ---
/* static */ double BoxGraspLogic::dot(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/* static */ double BoxGraspLogic::dot(const geometry_msgs::Vector3& v, const geometry_msgs::Point& p) {
    // Treat point p as a vector from origin for dot product calculation n · p0
    return v.x * p.x + v.y * p.y + v.z * p.z;
}


} // namespace pcl_processing