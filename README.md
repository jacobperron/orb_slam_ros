# orb_slam_ros #

ROS bindings for [ORB_SLAM](https://github.com/jacobperron/orb_slam.git).

## Install (catkin) ##

```bash
# Install catkin-tools
$ sudo apt-get install python-catkin-tools
# Setup catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws && catkin init
# Clone ORB-SLAM and this repository into src directory
$ git clone https://github.com/jacobperron/orb_slam.git src/orb_slam
$ git clone https://github.com/jacobperron/orb_slam_ros.git src/orb_slam_ros
# Build
$ catkin build
```

## Run ##

See example launch and configuration files in `orb_slam_ros/launch` and `orb_slam_ros/config` respectively.
