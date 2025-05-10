#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // Required for tf2::doTransform
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h> // Required for pcl_ros::transformPointCloud
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h> // For CPU VoxelGrid filter
#include <pcl/filters/passthrough.h> // Added for passthrough filter
#include <pcl/filters/statistical_outlier_removal.h> // Added for statistical outlier removal
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
#include <omp.h> // Added for OpenMP multithreading

// --- Added for GPU support ---
#include <pcl/gpu/filters/voxel_grid.h>
#include <pcl/gpu/containers/device_array.h>
// --- End Added ---

// Define the point type used throughout
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
// --- Added for GPU support ---
typedef pcl::gpu::DeviceArray<pcl::PointXYZRGB> PointCloudXYZRGBGpu;
// --- End Added ---

class PointCloudMerger
{
public:
    PointCloudMerger() : 
        nh_("~"), // Use private node handle for parameters
        tf_listener_(tf_buffer_), // Initialize TF listener with buffer
        sync_(SyncPolicy(10), sub_cloud1_, sub_cloud2_) // Initialize synchronizer
    {
        // Get parameters
        nh_.param<std::string>("cam1_topic", cam1_topic_, "/cam1/depth/color/points");
        nh_.param<std::string>("cam2_topic", cam2_topic_, "/cam2/depth/color/points");
        nh_.param<std::string>("merged_topic", merged_topic_, "/merged_pointcloud");
        nh_.param<std::string>("target_frame", target_frame_, "world"); // Default to 'world' frame
        
        // Voxel filter parameters
        nh_.param<bool>("enable_voxel_filter", enable_voxel_filter_, true);
        nh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.03);
        
        // Passthrough filter parameters
        nh_.param<bool>("enable_passthrough_filter", enable_passthrough_filter_, false);
        nh_.param<double>("passthrough_min_x", passthrough_min_x_, -1.0);
        nh_.param<double>("passthrough_max_x", passthrough_max_x_, 1.0);
        nh_.param<double>("passthrough_min_y", passthrough_min_y_, -1.0);
        nh_.param<double>("passthrough_max_y", passthrough_max_y_, 1.0);
        nh_.param<double>("passthrough_min_z", passthrough_min_z_, 0.0);
        nh_.param<double>("passthrough_max_z", passthrough_max_z_, 2.0);
        
        // Statistical outlier removal parameters
        nh_.param<bool>("enable_statistical_outlier_filter", enable_statistical_outlier_filter_, false);
        nh_.param<int>("statistical_outlier_mean_k", statistical_outlier_mean_k_, 30);
        nh_.param<double>("statistical_outlier_std_dev", statistical_outlier_std_dev_, 1.0);
        
        // Sync and thread parameters
        int queue_size;
        nh_.param<int>("queue_size", queue_size, 10);
        nh_.param<bool>("use_approximate_sync", use_approximate_sync_, false);
        nh_.param<double>("sync_slop", sync_slop_, 0.1);
        nh_.param<int>("thread_count", thread_count_, 1);
        
        // Set OpenMP thread count if supported
        #ifdef _OPENMP
        if (thread_count_ > 0) {
            omp_set_num_threads(thread_count_);
            ROS_INFO("OpenMP enabled with %d threads", thread_count_);
        }
        #endif
        
        // --- Added for GPU support ---
        nh_.param<bool>("use_gpu", use_gpu_, false); // Default to CPU processing
        // --- End Added ---

        ROS_INFO("PointCloudMerger Node Initializing:");
        ROS_INFO("  Cam1 Topic: %s", cam1_topic_.c_str());
        ROS_INFO("  Cam2 Topic: %s", cam2_topic_.c_str());
        ROS_INFO("  Merged Topic: %s", merged_topic_.c_str());
        ROS_INFO("  Target Frame: %s", target_frame_.c_str());
        ROS_INFO("  Voxel Filter Enabled: %s", enable_voxel_filter_ ? "true" : "false");
        if (enable_voxel_filter_) {
            ROS_INFO("  Voxel Leaf Size: %f meters", voxel_leaf_size_);
        }
        
        if (enable_passthrough_filter_) {
            ROS_INFO("  Passthrough Filter Enabled");
            ROS_INFO("    X range: [%f, %f]", passthrough_min_x_, passthrough_max_x_);
            ROS_INFO("    Y range: [%f, %f]", passthrough_min_y_, passthrough_max_y_);
            ROS_INFO("    Z range: [%f, %f]", passthrough_min_z_, passthrough_max_z_);
        }
        
        if (enable_statistical_outlier_filter_) {
            ROS_INFO("  Statistical Outlier Removal Enabled");
            ROS_INFO("    Mean K: %d", statistical_outlier_mean_k_);
            ROS_INFO("    StdDev Multiplier: %f", statistical_outlier_std_dev_);
        }
        
        if (use_approximate_sync_) {
            ROS_INFO("  Using approximate time synchronization with slop: %f sec", sync_slop_);
        } else {
            ROS_INFO("  Using exact time synchronization");
        }
        
        // --- Modified for GPU logging ---
        if (use_gpu_) {
            ROS_INFO("  Using GPU for Voxel Grid filtering.");
        } else {
            ROS_INFO("  Using CPU for Voxel Grid filtering.");
        }
        // --- End Modified ---

        // Initialize subscribers using message_filters
        sub_cloud1_.subscribe(nh_global_, cam1_topic_, queue_size);
        sub_cloud2_.subscribe(nh_global_, cam2_topic_, queue_size);

        // Register callback for synchronized messages
        sync_.registerCallback(boost::bind(&PointCloudMerger::cloudsCallback, this, _1, _2));

        // Initialize publisher
        merged_cloud_pub_ = nh_global_.advertise<sensor_msgs::PointCloud2>(merged_topic_, 1);

        ROS_INFO("PointCloudMerger node ready.");
    }

private:
    ros::NodeHandle nh_; // Private NodeHandle for parameters
    ros::NodeHandle nh_global_; // Global NodeHandle for topics/tf
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud1_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud2_;

    // Define synchronization policy (ApproximateTime)
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync_;

    ros::Publisher merged_cloud_pub_;

    std::string cam1_topic_, cam2_topic_, merged_topic_, target_frame_;
    
    // Voxel filter parameters
    bool enable_voxel_filter_;
    double voxel_leaf_size_;
    
    // Passthrough filter parameters 
    bool enable_passthrough_filter_;
    double passthrough_min_x_, passthrough_max_x_;
    double passthrough_min_y_, passthrough_max_y_;
    double passthrough_min_z_, passthrough_max_z_;
    
    // Statistical outlier removal parameters
    bool enable_statistical_outlier_filter_;
    int statistical_outlier_mean_k_;
    double statistical_outlier_std_dev_;
    
    // Sync and threading parameters
    bool use_approximate_sync_;
    double sync_slop_;
    int thread_count_;
    
    // --- Added for GPU support ---
    bool use_gpu_;
    // --- End Added ---

    // Apply passthrough filter to limit point cloud volume
    void applyPassthroughFilter(PointCloudXYZRGB::Ptr& cloud_in, PointCloudXYZRGB::Ptr& cloud_out)
    {
        if (!enable_passthrough_filter_ || cloud_in->empty()) {
            cloud_out = cloud_in;
            return;
        }
        
        pcl::ScopeTime timer("Passthrough filter");
        PointCloudXYZRGB::Ptr temp_cloud(new PointCloudXYZRGB);
        
        // Filter X axis
        pcl::PassThrough<pcl::PointXYZRGB> pass_x;
        pass_x.setInputCloud(cloud_in);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(passthrough_min_x_, passthrough_max_x_);
        pass_x.filter(*temp_cloud);
        
        // Filter Y axis
        pcl::PassThrough<pcl::PointXYZRGB> pass_y;
        pass_y.setInputCloud(temp_cloud);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(passthrough_min_y_, passthrough_max_y_);
        pass_y.filter(*temp_cloud);
        
        // Filter Z axis
        pcl::PassThrough<pcl::PointXYZRGB> pass_z;
        pass_z.setInputCloud(temp_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(passthrough_min_z_, passthrough_max_z_);
        pass_z.filter(*cloud_out);
        
        ROS_DEBUG("Passthrough filter: %lu points -> %lu points", 
                 cloud_in->size(), cloud_out->size());
    }
    
    // Apply statistical outlier removal filter to remove noise
    void applyStatisticalOutlierRemoval(PointCloudXYZRGB::Ptr& cloud_in, PointCloudXYZRGB::Ptr& cloud_out)
    {
        if (!enable_statistical_outlier_filter_ || cloud_in->empty()) {
            cloud_out = cloud_in;
            return;
        }
        
        pcl::ScopeTime timer("Statistical outlier removal");
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(statistical_outlier_mean_k_);
        sor.setStddevMulThresh(statistical_outlier_std_dev_);
        sor.filter(*cloud_out);
        
        ROS_DEBUG("Statistical outlier removal: %lu points -> %lu points", 
                 cloud_in->size(), cloud_out->size());
    }

    void cloudsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg, const sensor_msgs::PointCloud2ConstPtr& cloud2_msg)
    {
        ROS_DEBUG("Received synchronized point clouds.");
        pcl::ScopeTime timer("Total processing time");

        // 1. Transform Cloud 2 to the target frame
        sensor_msgs::PointCloud2 cloud2_transformed_msg;
        try
        {
            // Wait for transform availability (optional, with timeout)
            if (!tf_buffer_.canTransform(target_frame_, cloud2_msg->header.frame_id, cloud2_msg->header.stamp, ros::Duration(0.1))) {
                ROS_WARN_THROTTLE(2.0, "Cannot transform cloud2 from %s to %s. Waiting for transform.", 
                                  cloud2_msg->header.frame_id.c_str(), target_frame_.c_str());
                return;
            }
            
            // Lookup the transform
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                target_frame_, cloud2_msg->header.frame_id, cloud2_msg->header.stamp);

            // Apply the transform
            tf2::doTransform(*cloud2_msg, cloud2_transformed_msg, transform_stamped);
            cloud2_transformed_msg.header.frame_id = target_frame_; // Update frame_id after transform
            ROS_DEBUG("Successfully transformed cloud2 to target frame '%s'", target_frame_.c_str());
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2.0, "TF Exception transforming cloud2: %s", ex.what());
            return; // Skip this pair if transform fails
        }

        // 2. Process clouds
        pcl::ScopeTime cloud_processing_timer("Cloud processing");
        
        // Convert ROS messages to PCL PointClouds
        PointCloudXYZRGB::Ptr cloud1_pcl(new PointCloudXYZRGB);
        PointCloudXYZRGB::Ptr cloud2_pcl(new PointCloudXYZRGB);
        
        pcl::fromROSMsg(*cloud1_msg, *cloud1_pcl);
        pcl::fromROSMsg(cloud2_transformed_msg, *cloud2_pcl); // Use the transformed cloud 2
        
        ROS_DEBUG("Cloud1 size: %lu, Cloud2 size: %lu", 
                  cloud1_pcl->size(), cloud2_pcl->size());
                  
        // 3. Apply passthrough filters first to reduce data quickly
        PointCloudXYZRGB::Ptr cloud1_pass_filtered(new PointCloudXYZRGB);
        PointCloudXYZRGB::Ptr cloud2_pass_filtered(new PointCloudXYZRGB);
        
        applyPassthroughFilter(cloud1_pcl, cloud1_pass_filtered);
        applyPassthroughFilter(cloud2_pcl, cloud2_pass_filtered);

        // 4. Apply voxel grid filter to downsample
        PointCloudXYZRGB::Ptr cloud1_voxel_filtered(new PointCloudXYZRGB);
        PointCloudXYZRGB::Ptr cloud2_voxel_filtered(new PointCloudXYZRGB);

        if (enable_voxel_filter_)
        {
            pcl::ScopeTime filter_timer("Voxel filtering"); // Time the filtering step

            // --- Modified for GPU/CPU conditional filtering ---
            if (use_gpu_)
            {
                // --- GPU Voxel Grid Filtering ---
                try {
                    pcl::gpu::VoxelGrid<pcl::PointXYZRGB> vg_gpu;
                    vg_gpu.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

                    // Filter cloud 1
                    PointCloudXYZRGBGpu::Ptr cloud1_gpu_in(new PointCloudXYZRGBGpu);
                    cloud1_gpu_in->upload(cloud1_pass_filtered->points); // Upload points to GPU
                    PointCloudXYZRGBGpu cloud1_gpu_out;
                    vg_gpu.setInputCloud(cloud1_gpu_in);
                    vg_gpu.filter(cloud1_gpu_out);
                    cloud1_gpu_out.download(cloud1_voxel_filtered->points); // Download filtered points
                    cloud1_voxel_filtered->width = cloud1_voxel_filtered->points.size();
                    cloud1_voxel_filtered->height = 1;
                    cloud1_voxel_filtered->is_dense = false; // Voxel grid might create non-dense clouds

                    // Filter cloud 2
                    PointCloudXYZRGBGpu::Ptr cloud2_gpu_in(new PointCloudXYZRGBGpu);
                    cloud2_gpu_in->upload(cloud2_pass_filtered->points); // Upload points to GPU
                    PointCloudXYZRGBGpu cloud2_gpu_out;
                    vg_gpu.setInputCloud(cloud2_gpu_in);
                    vg_gpu.filter(cloud2_gpu_out);
                    cloud2_gpu_out.download(cloud2_voxel_filtered->points); // Download filtered points
                    cloud2_voxel_filtered->width = cloud2_voxel_filtered->points.size();
                    cloud2_voxel_filtered->height = 1;
                    cloud2_voxel_filtered->is_dense = false;

                    ROS_DEBUG("Applied GPU voxel grid filter");
                } catch (const pcl::PCLException& e) {
                    ROS_ERROR("PCL GPU Exception during filtering: %s. Falling back to CPU.", e.what());
                    use_gpu_ = false; // Disable GPU for subsequent calls if it fails once? (Optional)
                    goto cpu_filtering; // Jump to CPU filtering block
                }
                catch (const std::exception& e) {
                     ROS_ERROR("Standard Exception during GPU filtering: %s. Falling back to CPU.", e.what());
                     use_gpu_ = false; // Optional: disable GPU
                     goto cpu_filtering;
                }
                catch (...) {
                    ROS_ERROR("Unknown exception during GPU filtering. Falling back to CPU.");
                    use_gpu_ = false; // Optional: disable GPU
                    goto cpu_filtering;
                }

            } else {
cpu_filtering: // Label for goto jump from GPU error handling
                // --- CPU Voxel Grid Filtering (Original Code) ---
                pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
                voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

                // Filter cloud 1
                voxel_filter.setInputCloud(cloud1_pass_filtered);
                voxel_filter.filter(*cloud1_voxel_filtered);

                // Filter cloud 2
                voxel_filter.setInputCloud(cloud2_pass_filtered);
                voxel_filter.filter(*cloud2_voxel_filtered);

                ROS_DEBUG("Applied CPU voxel grid filter");
            }
            // --- End Modified ---

            ROS_DEBUG("Filtered Cloud1 size: %lu, Cloud2 size: %lu",
                      cloud1_voxel_filtered->size(), cloud2_voxel_filtered->size());
        } else {
            // If not filtering, just pass the previous pointers
            cloud1_voxel_filtered = cloud1_pass_filtered;
            cloud2_voxel_filtered = cloud2_pass_filtered;
            ROS_DEBUG("Voxel grid filter disabled");
        }
        
        // 5. Apply statistical outlier removal filter
        PointCloudXYZRGB::Ptr cloud1_outlier_filtered(new PointCloudXYZRGB);
        PointCloudXYZRGB::Ptr cloud2_outlier_filtered(new PointCloudXYZRGB);
        
        applyStatisticalOutlierRemoval(cloud1_voxel_filtered, cloud1_outlier_filtered);
        applyStatisticalOutlierRemoval(cloud2_voxel_filtered, cloud2_outlier_filtered);

        // 6. Concatenate the two clouds
        PointCloudXYZRGB merged_cloud_pcl = *cloud1_outlier_filtered + *cloud2_outlier_filtered;
        ROS_DEBUG("Merged cloud size: %lu points", merged_cloud_pcl.size());

        // 7. Convert merged PCL cloud back to ROS message
        sensor_msgs::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(merged_cloud_pcl, merged_cloud_msg);

        // Set header information for the merged cloud
        merged_cloud_msg.header.stamp = ros::Time::now(); 
        merged_cloud_msg.header.frame_id = target_frame_;

        // 8. Publish the merged cloud
        merged_cloud_pub_.publish(merged_cloud_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_pointcloud_merger_node");
    PointCloudMerger merger;
    ros::spin();
    return 0;
}
