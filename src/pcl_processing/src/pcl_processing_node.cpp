#include <ros/ros.h>
#include "pcl_processing/PCLProcessor.hpp"
#include "pcl_processing/PCLProcessorConfig.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_processing_node");
    ros::NodeHandle nh, private_nh("~");
    PCLProcessorConfig config;
    loadConfig(private_nh, config);

    PCLProcessor processor(nh, config);

    ros::Subscriber sub = nh.subscribe(
        config.input_topic,
        // in normal mode, always process the most recent message
        config.debug_mode 
            ? config.queue_size
            : 1,
        &PCLProcessor::processCloud,
        &processor
    );

    ros::spin();
    return 0;
}