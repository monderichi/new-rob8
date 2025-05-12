cmake_minimum_required(VERSION 3.0.2)
project(hand_band_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  mia_hand_bringup
  mia_hand_ros_control
  mia_hand_driver
  mia_hand_description
  std_msgs
  hardware_interface
  controller_manager
  roscpp
  rospy
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  rqt_joint_trajectory_controller
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS
    #include

  LIBRARIES ${PROJECT_NAME}

  CATKIN_DEPENDS
    geometry_msgs
    rospy
    sensor_msgs
    std_msgs
    mia_hand_bringup
    mia_hand_ros_control
    mia_hand_driver
    mia_hand_description
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
    rqt_joint_trajectory_controller
)

## Specify additional locations of header files if needed.
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/hand_band_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
