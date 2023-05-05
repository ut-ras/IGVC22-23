# sabertooth_rospy

ROS node to control a diff-drive robot with a sabertooth motor controller.

### Downloading and Running
---
- Install pysabertooth (A required python library for this package). Run `python -m pip install pysabertooth`

- If you already have a catkin workspace : Clone this repo in the `/src` folder of your workspace. Build packages using `catkin_make`.

- If you're creating a workspace for the first time, I'd suggest taking a look at ROS tutorials to create [catkin workspaces](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), [catkin_packages](http://wiki.ros.org/catkin/Tutorials/CreatingPackage) and [cloning packages from git](https://wiki.nps.edu/display/RC/Setting+up+a+ROS+package+from+Git)

  TL;DR:

  ```bash
  source /opt/ros/<ROS_DISTRO>/setup.bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone <link to repo>
  cd ~/catkin_ws
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
  ```
  
### Note
Depending on your ubuntu setup, you may need to give read/write permissions to the port.
This can be done by running `chmod +x /dev/tty<port>`. You may need to run this as root (sudo).


