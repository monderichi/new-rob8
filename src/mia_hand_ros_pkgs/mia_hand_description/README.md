mia_hand_description
====================

The following run instruction was tested in ROS noetic to display Mia Hand in rviz without showing the coupling between the index flexion and the thumb opposition joint:

    roslaunch mia_hand_description view_mia_hand_rviz.launch

The following run instruction was tested in ROS noetic to display Mia Hand in rviz showing the coupling between the index flexion and the thumb opposition joint:

    roslaunch mia_hand_description view_mia_hand_rviz.launch remap_MIA_th_opp:=True

Other inputs arguments of the launch file are:

    Rviz_on:                          default="true"                      doc="True to start Rviz and the joint state gui"
    use_joint_gui:                    default="true"                      doc="Enable the joint_state_publisher use a GUI for controlling joint states"
    right_hand:                       default="true"                      doc="True for right hand False otherwise"
    load_default_mia_model:           default="false"                     doc="flag to load URDF with default or calibrated joint parameters eg. joint limits"
    joint_limits_config_filename:     default="joint_limits.yaml"         doc="name of the config yaml file to calibrate joints limit of URDF file"
    transmission_config_filename:     default="transmission_config.yaml"  doc="name of the config yaml file to calibrate transmissions of Mia Hand"
    UR_Flange:                        default = "false"                   doc="True to visualize the UR flange with the Mia Hand".
    remap_MIA_th_opp:                 default="false"                     doc="True to remap the thumb opp joint position following the index fle position.
    hw_interface_thumb_fle:           default="PositionJointInterface"    doc="Type of hardware intereface to load in the mia_hand_gazebo.urdf.xacro file for the index flexion joint.
    hw_interface_index_fle:           default="PositionJointInterface"    doc="Type of hardware intereface to load in the mia_hand_gazebo.urdf.xacro file for the thumb flexion joint.
    hw_interface_mrl_fle:             default="PositionJointInterface"    doc="Type of hardware intereface to load in the mia_hand_gazebo.urdf.xacro file for the mrl flexion joint.
    robotNamespace:                   default="mia_hand"                  doc="Name space of the robot and its releated topics"


Please note that to this launch file loads the Mia Hand joint transmission YAML
configuration file (stored in the mia_hand_description/calibration folder)
to the parameter server.
