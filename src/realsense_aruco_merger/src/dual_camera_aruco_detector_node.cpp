// ROS Headers first
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h> // Include cv_bridge with ROS headers

// Eigen Headers
#include <Eigen/Geometry>

// Standard C++ Headers
#include <vector>
#include <string>
#include <map>
#include <iomanip>  // For std::setprecision
#include <ctime>    // For timestamp

// OpenCV Headers
#include <opencv2/opencv.hpp> // Main OpenCV header
#include <opencv2/objdetect.hpp> // Include object detection features
#include <opencv2/aruco.hpp>   // General ArUco header

struct MarkerInfo {
    int id;
    tf2::Transform transform; // Transform from marker frame to camera frame
    cv::Vec3d rvec, tvec; // Raw pose estimation results
    std::vector<cv::Point2f> corners; // Store corners for drawing
};

class DualCameraArucoDetectorCPP {
public:
    // Constructor: Initialize members in the initializer list
    DualCameraArucoDetectorCPP() : 
        nh_("~"), 
        it_(nh_global_), 
        sync_(SyncPolicy(10), sub_img1_, sub_img2_),
        aruco_dict_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)) // Init dictionary
    {
        // --- Parameters ---
        nh_.param<double>("marker_size", marker_size_, 0.02);
        nh_.param<int>("marker_id", target_marker_id_, 1);
        nh_.param<std::string>("cam1_frame", cam1_frame_, "cam1_color_optical_frame");
        nh_.param<std::string>("cam2_frame", cam2_frame_, "cam2_color_optical_frame");
        nh_.param<std::string>("marker_frame_cam1", marker_frame_cam1_, "aruco_marker_cam1");
        nh_.param<std::string>("marker_frame_cam2", marker_frame_cam2_, "aruco_marker_cam2");
        nh_.param<std::string>("world_frame", world_frame_, "world");
        nh_.param<std::string>("cam1_info_topic", cam1_info_topic_, "/cam1/cam1/color/camera_info");
        nh_.param<std::string>("cam2_info_topic", cam2_info_topic_, "/cam2/cam2/color/camera_info");
        nh_.param<std::string>("cam1_image_topic", cam1_image_topic_, "/cam1/cam1/color/image_raw");
        nh_.param<std::string>("cam2_image_topic", cam2_image_topic_, "/cam2/cam2/color/image_raw");
        // Optional: Params to enable/disable debug image/marker publishing
        nh_.param<bool>("publish_debug_images", publish_debug_images_, true);  // Changed default to true
        nh_.param<bool>("publish_tf_marker", publish_tf_marker_, true); 
        nh_.param<bool>("publish_viz_marker", publish_viz_marker_, true);

        // Initialize detector parameters for older OpenCV ArUco API
        detector_params_ = cv::aruco::DetectorParameters::create();

        ROS_INFO("DualCameraArucoDetectorCPP Initializing:");
        ROS_INFO("  Marker Size: %f m", marker_size_);
        ROS_INFO("  Target Marker ID: %d", target_marker_id_);
        ROS_INFO("  Cam1 Frame: %s", cam1_frame_.c_str());
        ROS_INFO("  Cam2 Frame: %s", cam2_frame_.c_str());
        ROS_INFO("  Publishing debug images: %s", publish_debug_images_ ? "YES" : "NO");
        ROS_INFO("  Using CPU processing");

        // --- Subscribers ---
        sub_cam1_info_ = nh_global_.subscribe<sensor_msgs::CameraInfo>(cam1_info_topic_, 1, &DualCameraArucoDetectorCPP::cam1InfoCallback, this);
        sub_cam2_info_ = nh_global_.subscribe<sensor_msgs::CameraInfo>(cam2_info_topic_, 1, &DualCameraArucoDetectorCPP::cam2InfoCallback, this);

        // Use image_transport::SubscriberFilter for synchronized image subscriptions
        sub_img1_.subscribe(it_, cam1_image_topic_, 1, image_transport::TransportHints("raw"));
        sub_img2_.subscribe(it_, cam2_image_topic_, 1, image_transport::TransportHints("raw"));
        sync_.registerCallback(boost::bind(&DualCameraArucoDetectorCPP::imagesCallback, this, _1, _2));

        // --- Publishers ---
        if (publish_debug_images_) {
            pub_img1_result_ = it_.advertise("cam1/aruco_result", 1);
            pub_img2_result_ = it_.advertise("cam2/aruco_result", 1);
        }
        if (publish_viz_marker_) {
            pub_marker1_ = nh_global_.advertise<visualization_msgs::Marker>("cam1/aruco_marker", 1);
            pub_marker2_ = nh_global_.advertise<visualization_msgs::Marker>("cam2/aruco_marker", 1);
        }
        pub_cam_transform_ = nh_global_.advertise<geometry_msgs::TransformStamped>("camera_transformation", 1);

        ROS_INFO("Waiting for camera info...");
    }

private:
    ros::NodeHandle nh_; // Private node handle for parameters
    ros::NodeHandle nh_global_; // Global node handle for topics/tf
    image_transport::ImageTransport it_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // Camera Intrinsics
    cv::Mat cam1_matrix_, cam1_dist_coeffs_;
    cv::Mat cam2_matrix_, cam2_dist_coeffs_;
    bool cam1_info_received_ = false;
    bool cam2_info_received_ = false;
    bool world_transform_published_ = false;

    // ArUco Members
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_; // Parameters object
    double marker_size_;
    int target_marker_id_;

    // Frame Names & Topics
    std::string cam1_frame_, cam2_frame_, marker_frame_cam1_, marker_frame_cam2_, world_frame_;
    std::string cam1_info_topic_, cam2_info_topic_, cam1_image_topic_, cam2_image_topic_;
    bool publish_debug_images_, publish_tf_marker_, publish_viz_marker_;

    // Subscribers
    ros::Subscriber sub_cam1_info_, sub_cam2_info_;
    image_transport::SubscriberFilter sub_img1_, sub_img2_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync_;

    // Publishers
    image_transport::Publisher pub_img1_result_, pub_img2_result_;
    ros::Publisher pub_marker1_, pub_marker2_;
    ros::Publisher pub_cam_transform_;

    // --- Callbacks ---
    void cam1InfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        if (!cam1_info_received_) {
            cam1_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->K.data()).clone();
            cam1_dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F, (void*)msg->D.data()).clone();
            cam1_info_received_ = true;
            ROS_INFO("Camera 1 intrinsics received.");
            checkStartDetection();
        }
    }

    void cam2InfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        if (!cam2_info_received_) {
            cam2_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->K.data()).clone();
            cam2_dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F, (void*)msg->D.data()).clone();
            cam2_info_received_ = true;
            ROS_INFO("Camera 2 intrinsics received.");
            checkStartDetection();
        }
    }

    void checkStartDetection() {
        if (cam1_info_received_ && cam2_info_received_) {
            ROS_INFO("Both camera intrinsics received. Starting detection.");
            // Subscribers are already set up and waiting via message_filters
        }
    }

    void imagesCallback(const sensor_msgs::ImageConstPtr& img1_msg, const sensor_msgs::ImageConstPtr& img2_msg) {
        if (!cam1_info_received_ || !cam2_info_received_) {
            ROS_WARN_THROTTLE(5.0, "Received images but camera info is not ready yet.");
            return;
        }

        try {
            cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(img1_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(img2_msg, sensor_msgs::image_encodings::BGR8);

            std::vector<MarkerInfo> markers1, markers2;

            // CPU-based ArUco detection
            markers1 = detectArucoMarker(cv_ptr1->image, cam1_matrix_, cam1_dist_coeffs_, img1_msg->header);
            markers2 = detectArucoMarker(cv_ptr2->image, cam2_matrix_, cam2_dist_coeffs_, img2_msg->header);

            // Publish marker TFs and visualizations
            publishMarkerInfo(markers1, img1_msg->header.frame_id, marker_frame_cam1_, pub_marker1_);
            publishMarkerInfo(markers2, img2_msg->header.frame_id, marker_frame_cam2_, pub_marker2_);

            calculateCameraTransformation(markers1, markers2, img1_msg->header.stamp); // Use stamp for TF

            if (publish_debug_images_) {
                cv::Mat img1_result = cv_ptr1->image.clone();
                cv::Mat img2_result = cv_ptr2->image.clone();
                displayMarkerInfo(img1_result, markers1, cam1_matrix_, cam1_dist_coeffs_);
                displayMarkerInfo(img2_result, markers2, cam2_matrix_, cam2_dist_coeffs_);
                pub_img1_result_.publish(cv_bridge::CvImage(img1_msg->header, "bgr8", img1_result).toImageMsg());
                pub_img2_result_.publish(cv_bridge::CvImage(img2_msg->header, "bgr8", img2_result).toImageMsg());
            }

        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            ROS_ERROR("Standard exception: %s", e.what());
        }
    }

    // CPU-based ArUco marker detection
    std::vector<MarkerInfo> detectArucoMarker(const cv::Mat& image, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, const std_msgs::Header& header) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, detector_params_); // Use older API

        std::vector<MarkerInfo> detected_markers;

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix, dist_coeffs, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i) {
                if (ids[i] == target_marker_id_) {
                    MarkerInfo info;
                    info.id = ids[i];
                    info.rvec = rvecs[i];
                    info.tvec = tvecs[i];
                    info.corners = corners[i]; // Store corners

                    // Convert OpenCV rvec/tvec to tf2::Transform (marker -> camera)
                    tf2::Vector3 translation(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
                    cv::Mat rot_mat;
                    cv::Rodrigues(rvecs[i], rot_mat); // Convert rvec to rotation matrix
                    tf2::Matrix3x3 tf_rot_mat(
                        rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                        rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                        rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2)
                    );
                    tf2::Quaternion rotation;
                    tf_rot_mat.getRotation(rotation);
                    
                    info.transform.setOrigin(translation);
                    info.transform.setRotation(rotation.normalized()); // Ensure quaternion is normalized

                    detected_markers.push_back(info);
                    break; // Found the target marker, stop searching
                }
            }
        }
        return detected_markers;
    }

    // --- Core Logic ---
    void publishMarkerInfo(const std::vector<MarkerInfo>& markers, const std::string& camera_frame, const std::string& marker_frame_base, const ros::Publisher& viz_pub) {
        if (markers.empty()) return;

        const MarkerInfo& marker = markers[0]; // Assuming only one target marker

        // Publish TF: camera -> marker
        if (publish_tf_marker_) {
            geometry_msgs::TransformStamped tf_stamped;
            tf_stamped.header.stamp = ros::Time::now(); // Use current time for marker TF
            tf_stamped.header.frame_id = camera_frame;
            tf_stamped.child_frame_id = marker_frame_base; // Use the specific marker frame name
            tf_stamped.transform = tf2::toMsg(marker.transform);
            tf_broadcaster_.sendTransform(tf_stamped);
        }

        // Publish Visualization Marker
        if (publish_viz_marker_) {
            visualization_msgs::Marker viz_marker;
            viz_marker.header.frame_id = camera_frame;
            viz_marker.header.stamp = ros::Time::now();
            viz_marker.ns = "aruco_markers";
            viz_marker.id = marker.id;
            viz_marker.type = visualization_msgs::Marker::CUBE;
            viz_marker.action = visualization_msgs::Marker::ADD;
            tf2::toMsg(marker.transform, viz_marker.pose); 
            viz_marker.scale.x = marker_size_;
            viz_marker.scale.y = marker_size_;
            viz_marker.scale.z = 0.001;
            viz_marker.color.r = 0.0;
            viz_marker.color.g = 1.0;
            viz_marker.color.b = 0.0;
            viz_marker.color.a = 0.7;
            viz_marker.lifetime = ros::Duration(0.5); // Make marker disappear if not seen
            viz_pub.publish(viz_marker);
        }
    }

    void calculateCameraTransformation(const std::vector<MarkerInfo>& markers1, const std::vector<MarkerInfo>& markers2, const ros::Time& stamp) {
        if (markers1.empty() || markers2.empty()) {
            ROS_WARN_THROTTLE(5.0, "Marker not detected in both cameras, cannot calculate transformation.");
            return;
        }

        const tf2::Transform& T1 = markers1[0].transform; // Transform: marker -> cam1
        const tf2::Transform& T2 = markers2[0].transform; // Transform: marker -> cam2

        try {
            // Calculate T_cam1_to_cam2 = T1 * T2.inverse()
            tf2::Transform T_cam1_to_cam2 = T1 * T2.inverse();

            // Publish dynamic TF: cam1 -> cam2
            geometry_msgs::TransformStamped tf_stamped;
            tf_stamped.header.stamp = stamp; // Use image timestamp for inter-camera TF
            tf_stamped.header.frame_id = cam1_frame_;
            tf_stamped.child_frame_id = cam2_frame_;
            tf_stamped.transform = tf2::toMsg(T_cam1_to_cam2);
            tf_broadcaster_.sendTransform(tf_stamped);
            pub_cam_transform_.publish(tf_stamped); // Also publish on topic

            // Publish static TF: cam1 -> world (only once)
            if (!world_transform_published_) {
                geometry_msgs::TransformStamped static_tf_stamped;
                static_tf_stamped.header.stamp = ros::Time::now(); // Stamp needed
                static_tf_stamped.header.frame_id = cam1_frame_;
                static_tf_stamped.child_frame_id = world_frame_;
                static_tf_stamped.transform.rotation.w = 1.0; // Identity transform
                static_tf_broadcaster_.sendTransform(static_tf_stamped);
                world_transform_published_ = true;
                ROS_INFO("Published static transform: %s -> %s", cam1_frame_.c_str(), world_frame_.c_str());
            }

            // Optional: Log transform details
            tf2::Vector3 t = T_cam1_to_cam2.getOrigin();
            double dist = t.length();
            ROS_INFO_THROTTLE(5.0, "Published TF: %s -> %s, translation=[%.4f, %.4f, %.4f]m, distance=%.4f m",
                              cam1_frame_.c_str(), cam2_frame_.c_str(), t.x(), t.y(), t.z(), dist);

        } catch (const tf2::TransformException& ex) {
            ROS_ERROR("TF Exception in calculation: %s", ex.what());
        }
    }

    // Optional: Helper to draw info on image (if publish_debug_images_ is true)
    void displayMarkerInfo(cv::Mat& image, const std::vector<MarkerInfo>& markers, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
         if (markers.empty()) {
             cv::putText(image, "No target marker detected", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
             return;
         }
         
         // Draw axes and corners for the first (target) marker found
         const auto& marker = markers[0];
         
         // Draw detected corners with clearer visuals
         std::vector<std::vector<cv::Point2f>> single_marker_corners;
         single_marker_corners.push_back(marker.corners);
         cv::aruco::drawDetectedMarkers(image, single_marker_corners, cv::Mat(1, 1, CV_32SC1, cv::Scalar(marker.id)), cv::Scalar(0, 255, 0)); 

         // Draw 3D axes with larger size for better visibility
         cv::drawFrameAxes(image, camera_matrix, dist_coeffs, marker.rvec, marker.tvec, marker_size_);
         
         // Calculate distance for display
         double distance = sqrt(pow(marker.tvec[0], 2) + pow(marker.tvec[1], 2) + pow(marker.tvec[2], 2));
         
         // Add more detailed information with better formatting
         int baseline = 0;
         int lineHeight = 30;
         int startY = 30;
         int fontFace = cv::FONT_HERSHEY_SIMPLEX;
         double fontScale = 0.7;
         int thickness = 2;
         cv::Scalar textColor(0, 255, 0);
         
         // Add background rectangle for better text visibility
         cv::Size textSize = cv::getTextSize("ID: " + std::to_string(marker.id), fontFace, fontScale, thickness, &baseline);
         cv::Rect bgRect(5, startY - textSize.height - 5, textSize.width + 10, textSize.height + 10);
         cv::rectangle(image, bgRect, cv::Scalar(0, 0, 0, 128), -1);
         
         // Display marker ID with shadow for better visibility
         cv::putText(image, "ID: " + std::to_string(marker.id), cv::Point(10, startY), 
                    fontFace, fontScale, cv::Scalar(0, 0, 0), thickness + 1);
         cv::putText(image, "ID: " + std::to_string(marker.id), cv::Point(10, startY), 
                    fontFace, fontScale, textColor, thickness);
                    
         // Display position information
         std::stringstream pos_ss;
         pos_ss << std::fixed << std::setprecision(3);
         pos_ss << "Pos: (" << marker.tvec[0] << ", " << marker.tvec[1] << ", " << marker.tvec[2] << ") m";
         cv::putText(image, pos_ss.str(), cv::Point(10, startY + lineHeight), 
                    fontFace, fontScale - 0.1, textColor, thickness - 1);
                    
         // Display distance information
         std::stringstream dist_ss;
         dist_ss << std::fixed << std::setprecision(3) << "Dist: " << distance << " m";
         cv::putText(image, dist_ss.str(), cv::Point(10, startY + lineHeight*2), 
                    fontFace, fontScale - 0.1, textColor, thickness - 1);
                    
         // Add timestamp to image
         std::time_t now = std::time(nullptr);
         std::tm *timeinfo = std::localtime(&now);
         char timestamp[20];
         std::strftime(timestamp, sizeof(timestamp), "%H:%M:%S", timeinfo);
         cv::putText(image, timestamp, cv::Point(image.cols - 100, 30), 
                   fontFace, fontScale - 0.2, textColor, 1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dual_camera_aruco_detector_node");
    DualCameraArucoDetectorCPP detector;
    ros::spin();
    return 0;
}
