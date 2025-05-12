# mia_hand_ros_pkgs

To install and use mia_hand_ros_pkgs:

1) download the repo

2) install **LibSerial**
This can be done by running `sudo apt install libserial-dev`.
If you prefer to install LibSerial from source, please refer to [LibSerial documentation](https://libserial.readthedocs.io/en/latest/install.html).
This package also depends on **mia_hand_msgs** package, so both of them should be cloned in the same catkin workspace.
see README file of the pkg mia_hand_ros_driver for further info.

3) install  ros pkg rqt_joint_trajectory_controller and the pkg joint-trajectory-controller
it may be needed to manually install the ros pkg rqt_joint_trajectory_controller and the pkg joint-trajectory-controller
(sudo apt install ros-noetic-rqt-joint-trajectory-controller and sudo apt install ros-noetic-joint-trajectory-controller).

4) build the repo

5) install setserial: sudo apt install setserial

6) If you have the calibration files of your Mia Hand hardware load them as follow:
In the mia_hand_description pkg, in calibration subpfolder, substitute the files joint_limits.yaml and transmission_config.yaml with the
correspondent homonymous calibration files provided to you.

7) Use the launch files to launch the examples included in this repo.
Please read all the README files of each package for further details about:
- the Mia hand URDF (see pkg mia_hand_description)
- the Mia hand simulator ( see pkg mia_hand_gazebo)
- the Mia hand hardware interface, to control the hand with ROS controller (see pkg mia_hand_ros_control)
- the Mia cpp driver (see pkg mia_hand_driver)

Please note that when you connect the Mia Hand serial port you should set the serial low_latency flag:

      setserial /dev/<tty_name> low_latency
