#pragma once

#include <string>
#include <ros/ros.h>

namespace pcl_processing {

struct GraspPoseEstimatorConfig {
    std::string input_topic = "/geometric_primitive"; // Default topic
    std::string output_topic = "/grasp_reference";   // Default topic

    // Cylinder parameters
    double cylinder_approach_angle_threshold = 0.785; // Radians (45 degrees)
    double cylinder_length_threshold = 0.1;          // Meters
    double cylinder_diameter_threshold = 0.05;       // Meters

    // Sphere parameters
    double sphere_diameter_threshold = 0.08;         // Meters

    // Box parameters
    double box_length_threshold = 0.1;               // Meters
    double box_width_threshold = 0.05;               // Meters

    // General grasp parameters
    double grasp_size_padding = 0.01;                // Meters
    double lateral_grasp_orientation_offset = 1.57;  // Radians (90 degrees)
    double wrist_min_angle = -1.57;                  // Radians (-90 degrees)
    double wrist_max_angle = 1.57;                   // Radians (90 degrees)

    // Filter parameters
    int moving_average_window_size = 5;
    int majority_voting_window_size = 5;

    // Method to load parameters from ROS Parameter Server
    void load(ros::NodeHandle& nh) {
        nh.param<std::string>("grasp/input_topic", input_topic, input_topic);
        nh.param<std::string>("grasp/output_topic", output_topic, output_topic);
        nh.param<double>("grasp/cylinder/approach_angle_threshold", cylinder_approach_angle_threshold, cylinder_approach_angle_threshold);
        nh.param<double>("grasp/cylinder/length_threshold", cylinder_length_threshold, cylinder_length_threshold);
        nh.param<double>("grasp/cylinder/diameter_threshold", cylinder_diameter_threshold, cylinder_diameter_threshold);
        nh.param<double>("grasp/sphere/diameter_threshold", sphere_diameter_threshold, sphere_diameter_threshold);
        nh.param<double>("grasp/box/length_threshold", box_length_threshold, box_length_threshold);
        nh.param<double>("grasp/box/width_threshold", box_width_threshold, box_width_threshold);
        nh.param<double>("grasp/padding", grasp_size_padding, grasp_size_padding);
        nh.param<double>("grasp/lateral_grasp_orientation_offset", lateral_grasp_orientation_offset, lateral_grasp_orientation_offset);
        nh.param<double>("grasp/wrist/min_angle", wrist_min_angle, wrist_min_angle);
        nh.param<double>("grasp/wrist/max_angle", wrist_max_angle, wrist_max_angle);
        nh.param<int>("grasp/filter/moving_average_window_size", moving_average_window_size, moving_average_window_size);
        nh.param<int>("grasp/filter/majority_voting_window_size", majority_voting_window_size, majority_voting_window_size);
    }
};

} // namespace pcl_processing