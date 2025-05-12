#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>  // For loading .pcd files
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_lccp");
    ros::NodeHandle nh;

    // Publisher for the input point cloud topic
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 1);

    // Ensure the file path is provided
    if (argc < 2) {
        ROS_ERROR("Please provide the path to a .pcd file as an argument.");
        return -1;
    }

    std::string pcd_file = argv[1];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Load the PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud) == -1) {
        ROS_ERROR("Failed to load PCD file: %s", pcd_file.c_str());
        return -1;
    }

    ROS_INFO("Loaded PCD file with %zu points.", cloud->points.size());

    // Convert to ROS PointCloud2 message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";  // Set the coordinate frame

    // Set up the rate for publishing
    ros::Rate loop_rate(0.5); // 0.5 Hz = publish every 2 seconds
    
    ROS_INFO("Starting to publish point cloud to topic 'input_cloud'. Press Ctrl+C to stop.");
    
    // Continuously publish the point cloud
    while (ros::ok()) {
        // Update timestamp to current time
        output.header.stamp = ros::Time::now();
        
        // Publish the point cloud
        pub.publish(output);
        
        ROS_INFO("Published point cloud to topic 'input_cloud'.");
        
        // Sleep to maintain the desired rate
        loop_rate.sleep();
        
        // Process any callbacks
        ros::spinOnce();
    }

    return 0;
}
