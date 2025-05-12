#pragma once // Good practice for header files

// Standard Libraries
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <string>
#include <algorithm>
#include <limits>
#include <cmath> // For std::abs

// ROS Headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h> // Often needed with ROS messages
#include "pcl_processing/GeometricPrimitive.h"


// PCL Core & Common
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// PCL Features
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

// PCL Segmentation
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp> // Keep this if needed for explicit instantiation

// PCL Search/KdTree (often needed implicitly by features/segmentation)
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h> // Or just kdtree.h

// Eigen (often included via PCL, but can be explicit)
#include <Eigen/Core>
#include <Eigen/Geometry>

// TF2/Eigen
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Project Headers (Include these last within the PCH)
// Note: If PCLProcessorConfig.hpp changes frequently, it might be better
// to exclude it from the PCH and include it normally in the .cpp files.
// For now, let's include it here.
#include "pcl_processing/PCLProcessorConfig.hpp"