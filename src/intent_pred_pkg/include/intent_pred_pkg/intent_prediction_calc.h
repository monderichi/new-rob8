#ifndef INTENT_PREDICTION_CALC_H
#define INTENT_PREDICTION_CALC_H

#include <Eigen/Dense>
#include <vector>
#include <string>

class IntentPredictCalc
{
public:
    IntentPredictCalc();
    virtual ~IntentPredictCalc() = default;
    
    /**
     * Predict the object the user intends to grasp
     * 
     * @param centroids Object centroids in hand camera frame
     * @param transformed_centroids Object centroids transformed to head camera frame
     * @param labels Object IDs
     * @return Index of the predicted grasp object
     */
    int findGraspObj(const std::vector<Eigen::Vector3f>& centroids,
                      const std::vector<Eigen::Vector3f>& transformed_centroids,
                      const std::vector<int>& labels);
                      
private:
    // Default positions and directions
    Eigen::Vector3f hand_position_;    // in hand camera frame
    Eigen::Vector3f hand_direction_;   // in hand camera frame
    Eigen::Vector3f gaze_position_;    // in head camera frame
    Eigen::Vector3f gaze_direction_;   // in head camera frame
    
    // History tracking for stabilization
    int prev_prediction_;
    std::vector<int> prediction_history_;
    const size_t history_size_ = 3;
    
    /**
     * Find object closest to the pointing line in 3D space - optimized implementation
     */
    std::vector<float> findClosestObject3dLineOptimized(
        const std::vector<Eigen::Vector3f>& objects_3d,
        const Eigen::Vector3f& position,
        const Eigen::Vector3f& direction);
        
    /**
     * Find the closest object based on 3D euclidean distance
     */
    std::vector<float> findClosestObject3DDepth(
        const std::vector<Eigen::Vector3f>& objects_3d,
        const Eigen::Vector3f& position);
        
    /**
     * Convert distances to scores - closer objects get higher scores
     */
    std::vector<float> distanceToScore(
        const std::vector<float>& distances,
        float max_distance = 1.0f,
        const std::pair<float, float>& score_range = {0.0f, 100.0f},
        const std::string& falloff = "inverse_square");
        
    /**
     * Combine the distance metrics with dynamic weighting
     */
    int weightResults(
        const std::vector<float>& hand_line_distances,
        const std::vector<float>& hand_depth_distances,
        const std::vector<float>& gaze_line_distances);
};

#endif // INTENT_PREDICTION_CALC_H