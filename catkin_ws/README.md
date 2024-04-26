
**All code is designed to run on ROS Noetic on Ubuntu 20.04**


**LIDAR / Navigation** Usage:

BUILD:
1. Must have sick_scan_XD installed to work with LIDAR; follow instruction for "Build on Linux ROS1" here. Make sure to install from source instead of the preinstalled repo! https://github.com/SICKAG/sick_scan_xd/blob/master/INSTALL-ROS1.md#build-on-linux-ros1
2. Install Hector SLAM: sudo apt-get install ros-noetic-hector-slam   (used for visual odometry acquisition)
4. Install Cartographer: https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html  (used for SLAM)
5. Template file for Cartographer located in: Current Scripts/templates/ranger.lua

roslaunch ranger_2dnav ranger_hector_slan.launch 
roslaunch cartographer_ros ranger.launch 

PACKAGE INFO:
- ranger_2dnav:  This package contains all the path-planning contents & the combined all–in-one
launch file called ranger_start.launch. It contains the ROS move_base launch, all required (and tunable) nav parameters, starts Hector SLAM, and the pyserial interface to connect an Arduino

- ranger_actionlib: This package starts the Action Server to send/receive feedback from the navigation path planner. It sends waypoints goals (capable of sending in a GPS coordinate frame as well -- NOT FULLY IMPLEMENTED) and can convert GPS frame coordinates to the local map frame.

- sabertooth_rospy: This package (specifically the CmdVelToMotors node) subscribes to the /cmd_vel topic and converts the velocity commands to differential wheel velocity inputs (based on robot geometry) and publishes this information in a special string format, which is received by a ROS subscriber on an Arduino.

----------
**IMU** Usage:
Install Phidgets drivers: https://github.com/ros-drivers/phidgets_drivers

- Follow instructions in Phidgets Github for "Udev rules setup"
- Start acquiring IMU data:
   roslaunch phidgets_spatial spatial.launch
- Perform necessary frame transforms: 
   rosrun IMU_transform transform.py

Using an Extended Kalman Filter (IMU + LIDAR Hector SLAM must be running)
- Template file located in: Current Scripts/templates/ekf_template.yaml

roslaunch robot_localization ekf_template.launch

----------
**Camera - IN PROGRESS**
- Must have RealSense Camera connected
- Install necessary drivers for ROS 1: https://github.com/IntelRealSense/realsense-ros  

Commands to run Intel RealSense Depth Camera D435 camera (run in separate terminals):

roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

rviz

*Lane Detection (and Pothole detection) using HSV Color Detection*
- The method used to identify lanes from an image of the road uses the following steps:
1.  OpenCV HSV color filtering to detect road
2.  Blur image BEFORE using HSV
3.  Invert results of filter

The result of these transformations produces an image in which the road is colored black, and everything that is not the road (lanes, obstacles and potholes) are colored white.

The range used to identify the gray road color is:
- [0, 0, 50]
- [179, 50, 200]

  After the image is generated, we run a for loop to save a list of all the white pixels in the image. This is output to a file called white_pixels.txt.


