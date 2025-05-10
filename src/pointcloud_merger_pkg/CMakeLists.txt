cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_merger_pkg) # Replace with your actual package name

## Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
  message_filters
)

# --- Modified PCL Find Package ---
# Add 'gpu' to the list of COMPONENTS
find_package(PCL REQUIRED COMPONENTS
  common
  io
  filters
  gpu # <-- Add this component
)
# --- End Modified ---

## Catkin package build information
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs tf2_ros tf2_sensor_msgs pcl_ros message_filters
  DEPENDS PCL # Add PCL dependency here if not already present
)

## Specify sources and executable
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(aruco_pointcloud_merger_node src/aruco_pointcloud_merger_node.cpp)

# --- Modified Target Link Libraries ---
target_link_libraries(aruco_pointcloud_merger_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCL_GPU_LIBRARIES} # <-- Add this line to link PCL GPU libraries
)
# --- End Modified ---

# Add install rules if needed
# install(TARGETS aruco_pointcloud_merger_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
