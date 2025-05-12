#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <map>
#include <vector>
#include "PCLProcessorConfig.hpp"

class PCLProcessor {
public:
    PCLProcessor(ros::NodeHandle& nh, const PCLProcessorConfig& config);

    void processCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
    PCLProcessorConfig config_;
    ros::Publisher pub_;
    ros::Publisher supervoxel_pub_;
    ros::Publisher centroid_marker_pub_;
    // ros::Publisher primitive_marker_pub_;
    ros::Publisher primitive_marker_array_pub_;
    ros::Publisher filtered_pub_;
    // Header for the last centroid message, needed for clearing previous markers
    std_msgs::Header last_centroid_header_;
    ros::Publisher primitive_pub_;


    // Processing methods
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr preprocessCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterDepth(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> segmentWithLCCP(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);

    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> createSupervoxelClusters(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
        pcl::SupervoxelClustering<pcl::PointXYZRGB>& super);

    void publishSupervoxelVisualization(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& sv_labeled_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorSegments(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr&);

    std::vector<uint32_t> generateColors(size_t count);
        
    pcl::PointCloud<pcl::PointXYZL>::Ptr performLCCPSegmentation(
        const std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr>& supervoxel_clusters,
        const std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& sv_labeled_cloud);

    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> createSegmentMapFromCloud(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud);
        
    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> filterSmallSegments(
        const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map);

    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> filterPlanarSegmentsFromMap(
        const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map);

    std::set<uint32_t> identifyPlanarLabelsToRemove(
        const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map);

    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr> createFilteredMap(
        const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& original_map,
        const std::set<uint32_t>& labels_to_remove);

    void publishSegmentMapVisualization(
        const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map,
        const std_msgs::Header& header);

    pcl::PointCloud<pcl::PointXYZL>::Ptr selectTargetSegment(
        const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map);

    void publishCentroidMarkersIfNeeded(
        const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map,
        const std::map<uint32_t, Eigen::Vector4f>& centroids,
        bool label_selected,
        uint32_t selected_label);

    void publishCentroidMarkers(
        const std::map<uint32_t, Eigen::Vector4f>& centroids,
        bool has_selected_label, // Flag indicating if selected_label is valid
        uint32_t selected_label,   // The selected label (only valid if flag is true)
        const std_msgs::Header& header);

    std::map<uint32_t, Eigen::Vector4f> computeSegmentCentroids(
        const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map);

    bool findClosestCentroidLabel( // Returns bool, takes output param
        const std::map<uint32_t, Eigen::Vector4f>& centroids,
        uint32_t& selected_label_out); // Output parameter 

    pcl::PointCloud<pcl::PointXYZL>::Ptr getSelectedCloudPtr(
        const std::map<uint32_t, pcl::PointCloud<pcl::PointXYZL>::Ptr>& segment_map,
        uint32_t selected_label);

    void fitPrimitiveToTarget(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& segment_cloud,
        const std_msgs::Header& header);

    struct PrimitiveFitResult {
        bool success = false;
        double inlier_percentage = 0.0;
        visualization_msgs::Marker marker;
        int type = -1; // 0: Sphere, 1: Cylinder, 2: Box, -1: None
    };

    PrimitiveFitResult fitSphere(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& segment_cloud,
        const std_msgs::Header& header);
    
    PrimitiveFitResult fitCylinder(
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& segment_cloud,
        const std_msgs::Header& header);
    
    PrimitiveFitResult fitBox( // Using the plane-based method
        const pcl::PointCloud<pcl::PointXYZL>::Ptr& segment_cloud,
        const std_msgs::Header& header);
    
    // Helper for visualization
    void clearPrimitiveMarkers(const std_msgs::Header& header);
    void publishPrimitiveMarkers(
        const PrimitiveFitResult& sphere_result,
        const PrimitiveFitResult& cylinder_result,
        const PrimitiveFitResult& box_result,
        int best_type,
        const std_msgs::Header& header);
};