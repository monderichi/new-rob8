#include "intent_pred_pkg/intent_prediction_node.h"

IntentPredictor::IntentPredictor() 
    : nh_("~"),
      tf_listener_(tf_buffer_)
{
    // Get parameters (with defaults)
    nh_.param<std::string>("hand_depth_frame", hand_depth_frame_, "cam1_depth_optical_frame");
    nh_.param<std::string>("head_depth_frame", head_depth_frame_, "cam2_depth_optical_frame");
    
    // Initialize the intent prediction algorithm
    intent_predict_calc_ = std::make_unique<IntentPredictCalc>();
    
    ROS_INFO("Waiting for transforms to become available...");
    if (!waitForTransforms()) {
        ROS_ERROR("Failed to get required transforms. Exiting.");
        ros::shutdown();
        return;
    }
    
    // Initialize subscribers
    centroid_sub_ = nh_.subscribe("/centroid_markers", 10, 
        &IntentPredictor::centroidCallback, this);
    
    // Initialize publishers
    intent_pub_ = nh_.advertise<std_msgs::Int32>("/object_id_to_grasp", 10);
    
    // Initialize debug timer (10 second period)
    debug_timer_ = nh_.createTimer(ros::Duration(10.0), 
        &IntentPredictor::debugTransformsCallback, this);
    
    ROS_INFO("Intent predictor node initialized successfully");
}

bool IntentPredictor::waitForTransforms()
{
    // Poll until transform is available or timeout
    const int max_attempts = 20;
    const float poll_interval = 0.5;
    
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        try {
            tf_buffer_.lookupTransform(
                head_depth_frame_,
                hand_depth_frame_,
                ros::Time(0),
                ros::Duration(1.0)
            );
            ROS_INFO("All required transforms are available.");
            return true;
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_STREAM_THROTTLE(5.0, "Waiting for transforms: " << ex.what());
            ros::Duration(poll_interval).sleep();
        }
    }
    
    ROS_ERROR("Timed out waiting for transforms.");
    return false;
}

void IntentPredictor::debugTransformsCallback(const ros::TimerEvent& event)
{
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            head_depth_frame_,
            hand_depth_frame_,
            ros::Time(0),
            ros::Duration(0.1)
        );
        
        const auto& t = transform.transform.translation;
        const auto& r = transform.transform.rotation;
        
        ROS_INFO_STREAM("Transform hand→head: [" 
            << t.x << ", " << t.y << ", " << t.z << "], quat=["
            << r.x << ", " << r.y << ", " << r.z << ", " << r.w << "]");
            
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_STREAM("Debug transform check failed: " << ex.what());
    }
}

void IntentPredictor::centroidCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_array)
{
    const size_t num_markers = marker_array->markers.size();
    
    if (num_markers == 0) {
        ROS_WARN("Received empty marker array");
        return;
    }
    
    // Get transform from hand depth frame to head depth frame
    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(
            head_depth_frame_,  // target frame
            hand_depth_frame_,  // source frame
            ros::Time(0),       // latest available transform
            ros::Duration(1.0)  // wait up to 1 second
        );
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_STREAM("Failed to lookup transform: " << ex.what());
        return;
    }
    
    // Pre-allocate data structures
    std::vector<Eigen::Vector3f> centroids(num_markers);
    std::vector<Eigen::Vector3f> transformed_centroids(num_markers);
    std::vector<int> labels(num_markers);
    
    // Process each marker
    for (size_t i = 0; i < num_markers; ++i) {
        const auto& marker = marker_array->markers[i];
        
        // Original position in hand camera frame
        centroids[i] = Eigen::Vector3f(
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z
        );
        
        labels[i] = marker.id;
        
        // Transform point to head camera frame
        geometry_msgs::PointStamped point_in, point_out;
        point_in.header.frame_id = hand_depth_frame_;
        point_in.point.x = marker.pose.position.x;
        point_in.point.y = marker.pose.position.y;
        point_in.point.z = marker.pose.position.z;
        
        tf2::doTransform(point_in, point_out, transform);
        
        transformed_centroids[i] = Eigen::Vector3f(
            point_out.point.x,
            point_out.point.y,
            point_out.point.z
        );
    }
    
    // Run the intent prediction algorithm
    int intended_idx = intent_predict_calc_->findGraspObj(centroids, transformed_centroids, labels);
    
    // Check if the index is valid
    if (intended_idx >= 0 && intended_idx < static_cast<int>(labels.size())) {
        std_msgs::Int32 msg;
        msg.data = labels[intended_idx];
        intent_pub_.publish(msg);
        ROS_INFO_STREAM("Predicted grasp object ID: " << labels[intended_idx]);
    } else {
        ROS_WARN_STREAM("Invalid prediction index: " << intended_idx);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "intent_prediction_node");
    IntentPredictor predictor;
    ros::spin();
    return 0;
}