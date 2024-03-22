
**All code is designed to run on ROS Noetic on Ubuntu 20.04**


*LIDAR / Navigation* Usage:

BUILD:
1. Must have sick_scan_XD installed to work with LIDAR; follow instruction for "Build on Linux ROS1" here. Make sure to install from source instead of the preinstalled repo! https://github.com/SICKAG/sick_scan_xd/blob/master/INSTALL-ROS1.md#build-on-linux-ros1
   
3. Install Hector SLAM: sudo apt-get install ros-kinetic-hector-slam
4. Clone this 'catkin_ws' and build with 'catkin_make'

PACKAGE INFO:
- ranger_2dnav:  This package contains all the path-planning contents & the combined all–in-one
launch file called ranger_start.launch. It contains the ROS move_base launch, all required (and tunable) nav parameters, starts Hector SLAM, and the pyserial interface to connect an Arduino

- ranger_actionlib: This package starts the Action Server to send/receive feedback from the navigation path planner. It sends waypoints goals (capable of sending in a GPS coordinate frame as well -- NOT FULLY IMPLEMENTED) and can convert GPS frame coordinates to the local map frame.

- sabertooth_rospy: This package (specifically the CmdVelToMotors node) subscribes to the /cmd_vel topic and converts the velocity commands to differential wheel velocity inputs (based on robot geometry) and publishes this information in a special string format, which is received by a ROS subscriber on an Arduino.


*Camera - IN PROGRESS*

Running the code
(must have camera connected. This is for ROS Noetic)

----------
Lane Detection (and Pothole detection) using HSV Color Detection
- The method used to identify lanes from an image of the road uses the following steps:
1.  OpenCV HSV color filtering to detect road
2.  Blur image BEFORE using HSV
3.  Invert results of filter

The result of these transformations produces an image in which the road is colored black, and everything that is not the road (lanes, obstacles and potholes) are colored white.

The range used to identify the gray road color is:
- [0, 0, 50]
- [179, 50, 200]

  After the image is generated, we run a for loop to save a list of all the white pixels in the image. This is output to a file called white_pixels.txt.


```roscore
cd catkin_ws
source devel/setup.bash
rosloaunch pointcloud_to_lasercan ranger.launch```
