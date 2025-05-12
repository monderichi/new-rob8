#include "pcl_processing/GraspPoseEstimator.hpp"
#include <std_msgs/String.h> // For logging config override if needed
#include <numeric> // For std::accumulate in moving average
#include <map>     // For std::map in majority voting
#include <algorithm> // For std::max_element

namespace pcl_processing {

// Constructor
GraspPoseEstimator::GraspPoseEstimator(ros::NodeHandle& nh, ros::NodeHandle& private_nh) :
    nh_(nh),
    private_nh_(private_nh),
    // Load configuration first using the private node handle
    config_(), // Default construct config_
    // Initialize logic classes AFTER config is loaded, passing the config
    sphere_logic_(config_),
    cylinder_logic_(config_),
    box_logic_(config_)
{
    // Load parameters into the config_ object
    config_.load(private_nh_);

    // Now that config_ is loaded, initialize members that depend on it
    // Initialize history deques with appropriate sizes (optional, can grow dynamically)
    // grasp_type_history_.resize(config_.majority_voting_window_size); // Or let them grow
    // grasp_size_history_.resize(config_.moving_average_window_size);
    // wrist_orientation_history_.resize(config_.moving_average_window_size);


    // Setup Subscriber
    // Use queue size 1 to always process the latest primitive
    primitive_sub_ = nh_.subscribe(
        config_.input_topic, 1, &GraspPoseEstimator::primitiveCallback, this);

    // Setup Publisher
    grasp_pub_ = nh_.advertise<pcl_processing::GraspReference>(
        config_.output_topic, 10); // Queue size 10

    ROS_INFO("GraspPoseEstimator initialized. Subscribed to '%s', Publishing to '%s'",
             config_.input_topic.c_str(), config_.output_topic.c_str());
    ROS_INFO("Filter windows: Type=%d, Size/Orientation=%d",
             config_.majority_voting_window_size, config_.moving_average_window_size);
}

// Main Callback
void GraspPoseEstimator::primitiveCallback(const pcl_processing::GeometricPrimitive::ConstPtr& msg) {
    if (!msg) {
        ROS_WARN("Received null message pointer in primitiveCallback.");
        return;
    }

    ROS_DEBUG("Received geometric primitive: type %d", msg->shape_type);

    // 1. Select Grasp Type for current frame (delegates based on shape)
    int8_t current_type = selectGraspType(msg);

    // 2. Compute Wrist Orientation for current frame (delegates based on shape)
    float current_orientation = computeWristOrientation(msg, current_type);

    // 3. Compute Grasp Size for current frame (delegates based on shape)
    float current_size = computeGraspSize(msg);

    // 4. Update history buffers
    updateHistory(current_type, current_size, current_orientation);

    // 5. Compute smoothed/filtered output
    pcl_processing::GraspReference final_grasp = smoothGrasp();

    // 6. Publish the final result
    final_grasp.header = msg->header; // Use timestamp from input message
    grasp_pub_.publish(final_grasp);

    ROS_DEBUG("Published GraspReference: Type=%d, Size=%.3f, Orientation=%.3f",
             final_grasp.grasp_type, final_grasp.grasp_size, final_grasp.wrist_orientation);
}

// --- Orchestration Methods ---

int8_t GraspPoseEstimator::selectGraspType(const pcl_processing::GeometricPrimitive::ConstPtr& msg) {
    switch (msg->shape_type) {
        case GeometricPrimitive::SHAPE_SPHERE:
            if (msg->dimensions.empty()) {
                 ROS_WARN("Sphere message has no dimensions."); return GraspReference::GRASP_NONE;
            }
            return sphere_logic_.selectGraspType(msg->dimensions[0]); // Pass radius
        case GeometricPrimitive::SHAPE_CYLINDER:
             if (msg->dimensions.size() < 2 || msg->orientation.empty()) {
                 ROS_WARN("Cylinder message has insufficient dimensions/orientation."); return GraspReference::GRASP_NONE;
            }
            // Assuming dimensions[0]=radius, dimensions[1]=length, orientation[0]=axis
            return cylinder_logic_.selectGraspType(msg->dimensions[0], msg->dimensions[1], msg->orientation[0]);
        case GeometricPrimitive::SHAPE_BOX:
            if (msg->dimensions.size() < 3 || msg->orientation.size() < 3) { // Check axes size too
                ROS_WARN("Box message has insufficient dimensions or orientation."); return GraspReference::GRASP_NONE;
           }
           // Pass center, axes, and dimensions
           return box_logic_.selectGraspType(msg->center, msg->orientation, msg->dimensions);
        case GeometricPrimitive::SHAPE_UNKNOWN:
        default:
            ROS_WARN("Received unknown or invalid shape type: %d", msg->shape_type);
            return GraspReference::GRASP_NONE;
    }
}

float GraspPoseEstimator::computeWristOrientation(const pcl_processing::GeometricPrimitive::ConstPtr& msg, int8_t grasp_type) {
     // If no valid grasp type was selected, orientation is meaningless (return NaN)
    if (grasp_type == GraspReference::GRASP_NONE) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    float raw_orientation = 0.0f;
    // Create a default approach direction (from origin to center)
    geometry_msgs::Point approach_dir;

    switch (msg->shape_type) {
        case GeometricPrimitive::SHAPE_SPHERE:                    
            if (!msg->center.x && !msg->center.y && !msg->center.z) {
                // If no center provided, use default approach from front
                approach_dir.x = 0;
                approach_dir.y = 0; 
                approach_dir.z = 1;
            } else {
                // Use direction from origin to sphere center
                approach_dir = msg->center;
            }
            raw_orientation = sphere_logic_.computeWristOrientation(grasp_type, approach_dir);
            break;
        case GeometricPrimitive::SHAPE_CYLINDER:
             if (msg->orientation.empty()) {
                 ROS_WARN("Cylinder message has no orientation for wrist calculation."); return 0.0f;
            }
            raw_orientation = cylinder_logic_.computeWristOrientation(msg->orientation[0], grasp_type); // Pass axis
            break;
        case GeometricPrimitive::SHAPE_BOX:
             if (msg->dimensions.size() < 3 || msg->orientation.size() < 3) {
                 ROS_WARN("Box message has insufficient dimensions/orientation for wrist calculation."); return 0.0f;
            }
            raw_orientation = box_logic_.computeWristOrientation(msg->orientation, msg->dimensions, grasp_type);
            break;
        case GeometricPrimitive::SHAPE_UNKNOWN:
        default:
            // Already handled by the initial check on grasp_type
            break;
    }
     // Clamp the final orientation within limits (clamping done in smoothGrasp)
    return raw_orientation; // Return raw value, clamping happens during smoothing
}

float GraspPoseEstimator::computeGraspSize(const pcl_processing::GeometricPrimitive::ConstPtr& msg) {
    switch (msg->shape_type) {
        case GeometricPrimitive::SHAPE_SPHERE:
            if (msg->dimensions.empty()) {
                 ROS_WARN("Sphere message has no dimensions for size calculation."); return 0.0f;
            }
            return sphere_logic_.computeGraspSize(msg->dimensions[0]); // Pass radius
        case GeometricPrimitive::SHAPE_CYLINDER:
             if (msg->dimensions.empty()) {
                 ROS_WARN("Cylinder message has no dimensions for size calculation."); return 0.0f;
            }
            return cylinder_logic_.computeGraspSize(msg->dimensions[0]); // Pass radius
        case GeometricPrimitive::SHAPE_BOX:
             if (msg->dimensions.size() < 3 || msg->orientation.size() < 3) {
                 ROS_WARN("Box message has insufficient dimensions/orientation for size calculation."); return 0.0f;
            }
             // Box size calculation might need axes too, depending on implementation
            return box_logic_.computeGraspSize(msg->orientation, msg->dimensions);
        case GeometricPrimitive::SHAPE_UNKNOWN:
        default:
            ROS_WARN("Received unknown or invalid shape type for size calculation: %d", msg->shape_type);
            return 0.0f; // Return 0 size for unknown shapes
    }
}


// --- Smoothing Logic ---

int8_t GraspPoseEstimator::getMajorityGraspType() {
    if (grasp_type_history_.empty()) {
        return GraspReference::GRASP_NONE;
    }

    std::map<int8_t, int> counts;
    for (int8_t type : grasp_type_history_) {
        // Only count valid grasp types for majority voting
        if (type != GraspReference::GRASP_NONE) {
            counts[type]++;
        }
    }

    if (counts.empty()) {
         // If history only contained NONE, return NONE
        return GraspReference::GRASP_NONE;
    }

    // Find the type with the maximum count
    auto max_it = std::max_element(counts.begin(), counts.end(),
        [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

    // Optional: Add a threshold? Only return majority if count > N?
    // For now, just return the most frequent valid type.
    return max_it->first;
}

float GraspPoseEstimator::getMovingAverage(const std::deque<float>& values) {
    if (values.empty()) {
        return 0.0f; // Or NaN?
    }
    // Use std::accumulate for sum
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return static_cast<float>(sum / values.size());
}

// --- Helper Functions ---

void GraspPoseEstimator::updateHistory(int8_t type, float size, float orientation) {
    // Add current values to the front (or back, consistency matters)
    grasp_type_history_.push_front(type);
    grasp_size_history_.push_front(size);
    wrist_orientation_history_.push_front(orientation);

    // Remove old values if deque exceeds window size
    while (grasp_type_history_.size() > config_.majority_voting_window_size) {
        grasp_type_history_.pop_back();
    }
    while (grasp_size_history_.size() > config_.moving_average_window_size) {
        grasp_size_history_.pop_back();
    }
    while (wrist_orientation_history_.size() > config_.moving_average_window_size) {
        wrist_orientation_history_.pop_back();
    }
}

pcl_processing::GraspReference GraspPoseEstimator::smoothGrasp() {
    pcl_processing::GraspReference smoothed_grasp;

    smoothed_grasp.grasp_type = getMajorityGraspType();
    smoothed_grasp.grasp_size = getMovingAverage(grasp_size_history_);

    // Calculate moving average for orientation
    float avg_orientation = getMovingAverage(wrist_orientation_history_);
    // Clamp the smoothed orientation
    smoothed_grasp.wrist_orientation = clamp(avg_orientation, config_.wrist_min_angle, config_.wrist_max_angle);

    return smoothed_grasp;
}

float GraspPoseEstimator::clamp(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}


} // namespace pcl_processing