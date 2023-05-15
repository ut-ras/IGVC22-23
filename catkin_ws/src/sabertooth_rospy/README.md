# sabertooth_rospy

ROS node to control a diff-drive robot with a sabertooth motor controller with Teleop_twist_joy node
Tested with Xbox 360, Logitech F310 controllers

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


If your user doesn't belong to the serial group, add user to the dialout group by running `sudo usermod -a -G dialout $USER`

# Update
- Added teleop_twist_joy as package dependency in place of cloing a separate git submodule. Running a rosdep install mentioned in the TL;DR will install it.
- Changed launch file to launch teleop specific parameters


### To-Do

---

- [x] ROS node to subscribe to cmd_vel from teleop_twist_joy
- [x] Convert cmd_vel to diff-drive velocities for each wheel
- [x] Sabertooth_rospy node to detect a connected sabertooth device
- [x] Send drive commands to sabertooth motor controller
- [x] teleop_twist_joy tuning to get adequate speed on robot
- [x] change linear and angular axis for driving robot
- [x] launch file to launch teleop, cmd_vel conversion and sabertooth to motor comm
- [x] Code cleanup and commenting
- [x] README.md instructions
- [ ] Udev instructions to rename device to appear as /dev/sabertooth 
