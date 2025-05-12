#pragma once
#include <string>
#include <ros/ros.h> // Include the ROS header for NodeHandle

struct PCLProcessorConfig {
    std::string input_topic;
    std::string output_topic;

    bool debug_mode;
    int queue_size;
    bool publish_supervoxel_cloud;
    std::string supervoxel_cloud_topic;
    std::string centroid_marker_topic;

    float voxel_leaf_size;
    float min_depth;
    float max_depth;

    float voxel_resolution;
    float seed_resolution;
    float color_importance;
    float spatial_importance;
    float normal_importance;

    float concavity_tolerance_threshold;
    bool use_smoothness_check;
    float smoothness_threshold;
    float k_factor;
    int min_segment_size;
    bool use_supervoxel_refinement;
    int supervoxel_refinement_iterations;
    float small_segment_threshold_percent;
    bool use_single_camera_transform;

    bool publish_centroid_markers;

    // Planar filtering parameters
    bool filter_planar_segments;
    float planar_distance_threshold;
    float min_planar_inlier_percentage;
    int max_planar_segment_size; // Max size (points) to be considered potentially planar

    // Filtered cloud visualization
    bool publish_filtered_cloud;
    std::string filtered_cloud_topic;

    // Primitive Fitting parameters
    bool publish_primitive_marker;
    bool publish_unselected_primitive_markers;
    std::string primitive_marker_topic;
    float primitive_distance_threshold;
    float min_primitive_inlier_percentage;
    float cylinder_normal_distance_weight; // Weight for normals in cylinder fitting
    float cylinder_min_radius;
    float cylinder_max_radius;
    float sphere_max_radius;
    float box_max_size; // Maximum size for a single side of the box
    std::string geometric_primitive_topic;

};

void loadConfig(ros::NodeHandle& nh, PCLProcessorConfig& config);