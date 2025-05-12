#include <ros/ros.h>
#include "pcl_processing/GraspPoseEstimator.hpp" // Include your main class header

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "grasp_pose_estimation_node"); // Node name defined here

    // Create the public node handle (for topics, services, etc.)
    ros::NodeHandle nh;

    // Create the private node handle (for parameters, typically "~")
    ros::NodeHandle private_nh("~");

    // Instantiate the main processing class
    // Pass both node handles to the constructor
    try {
        pcl_processing::GraspPoseEstimator estimator(nh, private_nh);

        // Spin allows ROS to process callbacks (like subscriptions)
        // It will exit when ROS shuts down (e.g., Ctrl+C)
        ROS_INFO("Grasp Pose Estimation Node started.");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("An exception occurred in GraspPoseEstimator: %s", e.what());
        return 1; // Indicate failure
    } catch (...) {
        ROS_ERROR("An unknown exception occurred in GraspPoseEstimator.");
        return 1; // Indicate failure
    }


    return 0; // Indicate successful completion (though spin usually prevents this)
}