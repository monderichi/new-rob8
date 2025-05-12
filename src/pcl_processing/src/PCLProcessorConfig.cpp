#include "stdafx.h"
#include "pcl_processing/PCLProcessorConfig.hpp"

void loadConfig(ros::NodeHandle& nh, PCLProcessorConfig& config) {
    nh.param<std::string>("input_topic", config.input_topic, "/input_cloud");
    nh.param<std::string>("output_topic", config.output_topic, "/segmented_cloud");
    nh.param<bool>("debug_mode", config.debug_mode, false);
    nh.param<int>("queue_size", config.queue_size, 1);
    nh.param<bool>("publish_supervoxel_cloud", config.publish_supervoxel_cloud, false);
    nh.param<std::string>("supervoxel_cloud_topic", config.supervoxel_cloud_topic, "/supervoxel_cloud");
    nh.param<std::string>("centroid_marker_topic", config.centroid_marker_topic, "/centroid_markers");
    nh.param<float>("voxel_leaf_size", config.voxel_leaf_size, 0.01f);
    nh.param<float>("min_depth", config.min_depth, 1.1f);
    nh.param<float>("max_depth", config.max_depth, 5.0f);
    nh.param<float>("voxel_resolution", config.voxel_resolution, 0.03f);
    nh.param<float>("seed_resolution", config.seed_resolution, 0.15f);
    nh.param<float>("color_importance", config.color_importance, 0.1f);
    nh.param<float>("spatial_importance", config.spatial_importance, 1.0f);
    nh.param<float>("normal_importance", config.normal_importance, 5.0f);
    nh.param<float>("concavity_tolerance_threshold", config.concavity_tolerance_threshold, 15.0f);
    nh.param<bool>("use_smoothness_check", config.use_smoothness_check, true);
    nh.param<float>("smoothness_threshold", config.smoothness_threshold, 0.15f);
    nh.param<float>("k_factor", config.k_factor, 0.5f);
    nh.param<int>("min_segment_size", config.min_segment_size, 500);
    nh.param<bool>("use_supervoxel_refinement", config.use_supervoxel_refinement, true);
    nh.param<float>("small_segment_threshold_percent", config.small_segment_threshold_percent, 1.0f);
    nh.param<bool>("use_single_camera_transform", config.use_single_camera_transform, false);
    nh.param<int>("supervoxel_refinement_iterations", config.supervoxel_refinement_iterations, 2);
    nh.param<bool>("publish_centroid_markers", config.publish_centroid_markers, true);
    // Load planar filtering parameters
    nh.param<bool>("filter_planar_segments", config.filter_planar_segments, true);
    nh.param<float>("planar_distance_threshold", config.planar_distance_threshold, 0.015f); // e.g., 1.5 cm
    nh.param<float>("min_planar_inlier_percentage", config.min_planar_inlier_percentage, 0.85f); // e.g., 85%
    nh.param<int>("max_planar_segment_size", config.max_planar_segment_size, 5000); // e.g., segments larger than 5000 points are checked
    // Load filtered cloud visualization parameters
    nh.param<bool>("publish_filtered_cloud", config.publish_filtered_cloud, true);
    nh.param<std::string>("filtered_cloud_topic", config.filtered_cloud_topic, "/filtered_segments_cloud");
    // Load primitive fitting parameters
    nh.param<bool>("publish_primitive_marker", config.publish_primitive_marker, true);
    nh.param<bool>("publish_unselected_primitive_markers", config.publish_unselected_primitive_markers, true);
    nh.param<std::string>("primitive_marker_topic", config.primitive_marker_topic, "/primitive_marker");
    nh.param<float>("primitive_distance_threshold", config.primitive_distance_threshold, 0.01f); // e.g., 1 cm
    nh.param<float>("min_primitive_inlier_percentage", config.min_primitive_inlier_percentage, 0.75f); // e.g., 75%
    nh.param<float>("cylinder_normal_distance_weight", config.cylinder_normal_distance_weight, 0.1f);
    nh.param<float>("cylinder_min_radius", config.cylinder_min_radius, 0.01f); // e.g., 1cm min radius
    nh.param<float>("cylinder_max_radius", config.cylinder_max_radius, 0.2f); // e.g., 20cm max radius
    nh.param<float>("sphere_max_radius", config.sphere_max_radius, 0.2f); // e.g., 20cm max radius
    nh.param<float>("box_max_size", config.box_max_size, 0.2f); // e.g., 20cm max size for a single side of the box
    nh.param<std::string>("geometric_primitive_topic", config.geometric_primitive_topic, "/geometric_primitive");
}