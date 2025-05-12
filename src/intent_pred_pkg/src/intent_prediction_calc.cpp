#include "intent_pred_pkg/intent_prediction_calc.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

IntentPredictCalc::IntentPredictCalc() {
    // Default positions and directions
    hand_position_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);   // in hand camera frame
    hand_direction_ = Eigen::Vector3f(0.0f, 0.0f, 1.0f);  // in hand camera frame
    gaze_direction_ = Eigen::Vector3f(0.0f, 0.0f, 1.0f);  // in head camera frame
    gaze_position_ = Eigen::Vector3f(477.0f, 233.0f, 0.0f); // in head camera frame
    
    // Initialize history
    prev_prediction_ = -1;
}

int IntentPredictCalc::findGraspObj(
    const std::vector<Eigen::Vector3f>& centroids,
    const std::vector<Eigen::Vector3f>& transformed_centroids,
    const std::vector<int>& labels) {
    
    // Check for empty inputs
    if (centroids.empty() || transformed_centroids.empty() || labels.empty()) {
        return -1;
    }
    
    // Find the closest object to the hand pointing vector
    std::vector<float> hand_line_distances = findClosestObject3dLineOptimized(
        centroids, hand_position_, hand_direction_);
        
    // Find closest object to the hand in 3D space
    std::vector<float> hand_3D_distances = findClosestObject3DDepth(
        centroids, hand_position_);
        
    // Find closest object to the head gaze vector
    std::vector<float> head_gaze_distances = findClosestObject3dLineOptimized(
        transformed_centroids, gaze_position_, gaze_direction_);
        
    // Combine the results and weight them
    int closest_idx_final = weightResults(
        hand_line_distances, hand_3D_distances, head_gaze_distances);
        
    return closest_idx_final;
}

std::vector<float> IntentPredictCalc::findClosestObject3dLineOptimized(
    const std::vector<Eigen::Vector3f>& objects_3d,
    const Eigen::Vector3f& head_hand_position,
    const Eigen::Vector3f& pointing_direction) {
        
    // Normalize direction vector
    Eigen::Vector3f direction = pointing_direction;
    float direction_norm = direction.norm();
    if (direction_norm > 0) {
        direction.normalize();
    }
    
    // Initialize distances array with infinite values
    std::vector<float> distances(objects_3d.size(), std::numeric_limits<float>::infinity());
    
    // Process each object
    for (size_t i = 0; i < objects_3d.size(); i++) {
        // Vector from hand/head to object
        Eigen::Vector3f vector = objects_3d[i] - head_hand_position;
        
        // Scalar projection onto direction
        float proj_length = vector.dot(direction);
        
        // Only consider objects in front (positive projection)
        if (proj_length > 0) {
            // Calculate closest point on ray
            Eigen::Vector3f closest_point = head_hand_position + proj_length * direction;
            
            // Calculate perpendicular distance
            float perp_distance = (objects_3d[i] - closest_point).norm();
            
            // Store distance
            distances[i] = perp_distance;
        }
    }
    
    return distances;
}

std::vector<float> IntentPredictCalc::findClosestObject3DDepth(
    const std::vector<Eigen::Vector3f>& centroids,
    const Eigen::Vector3f& object_pos) {
    
    // Calculate Euclidean distances for each centroid
    std::vector<float> distances;
    distances.reserve(centroids.size());
    
    for (const auto& centroid : centroids) {
        float dist = (centroid - object_pos).norm();
        distances.push_back(dist);
    }
    
    return distances;
}

std::vector<float> IntentPredictCalc::distanceToScore(
    const std::vector<float>& distances,
    float max_distance,
    const std::pair<float, float>& score_range,
    const std::string& falloff) {
    
    std::vector<float> scores;
    scores.reserve(distances.size());
    
    float min_score = score_range.first;
    float max_score = score_range.second;
    
    for (float distance : distances) {
        // Clip distance to max_distance
        float clipped_distance = std::min(distance, max_distance);
        
        // Apply falloff function
        float normalized_score;
        
        if (falloff == "linear") {
            normalized_score = 1.0f - (clipped_distance / max_distance);
        }
        else if (falloff == "inverse") {
            normalized_score = 1.0f / (1.0f + (clipped_distance / max_distance));
        }
        else if (falloff == "inverse_square") {
            normalized_score = 1.0f / (1.0f + std::pow(clipped_distance / max_distance, 2));
        }
        else {
            // Default to linear
            normalized_score = 1.0f - (clipped_distance / max_distance);
        }
        
        // Scale to requested score range
        float score = min_score + normalized_score * (max_score - min_score);
        scores.push_back(score);
    }
    
    return scores;
}

int IntentPredictCalc::weightResults(
    const std::vector<float>& hand_line_distances,
    const std::vector<float>& hand_depth_distances,
    const std::vector<float>& gaze_line_distances) {
    
    // 1. Convert each distance metric to a normalized score (higher = better)
    std::vector<float> hand_line_scores = distanceToScore(
        hand_line_distances, 0.5f, {0.0f, 100.0f}, "inverse_square");
        
    std::vector<float> hand_depth_scores = distanceToScore(
        hand_depth_distances, 1.0f, {0.0f, 100.0f}, "linear");
        
    std::vector<float> gaze_line_scores = distanceToScore(
        gaze_line_distances, 0.5f, {0.0f, 100.0f}, "inverse_square");
        
    // 2. Dynamically adjust weights based on hand proximity
    float min_hand_dist = std::numeric_limits<float>::infinity();
    if (!hand_depth_distances.empty()) {
        min_hand_dist = *std::min_element(hand_depth_distances.begin(), hand_depth_distances.end());
    }
    
    float weight_line, weight_hand, weight_gaze;
    
    if (min_hand_dist < 0.15f) {  // Close to object (15cm)
        weight_line = 0.2f;
        weight_hand = 0.7f;
        weight_gaze = 0.1f;
    }
    else if (min_hand_dist < 0.4f) {  // Medium distance (40cm)
        weight_line = 0.4f;
        weight_hand = 0.3f;
        weight_gaze = 0.3f;
    }
    else {  // Far from objects
        weight_line = 0.45f;
        weight_hand = 0.1f;
        weight_gaze = 0.45f;
    }
    
    // 3. Calculate combined scores using weights
    std::vector<float> combined_scores(hand_line_scores.size());
    for (size_t i = 0; i < combined_scores.size(); i++) {
        combined_scores[i] = (weight_line * hand_line_scores[i] +
                              weight_hand * hand_depth_scores[i] +
                              weight_gaze * gaze_line_scores[i]);
    }
    
    // 4. Find index of object with highest score
    auto max_it = std::max_element(combined_scores.begin(), combined_scores.end());
    int closest_idx = std::distance(combined_scores.begin(), max_it);
    
    return closest_idx;
}