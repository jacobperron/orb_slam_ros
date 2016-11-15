# orb_slam_ros #

ROS bindings for [ORB_SLAM](https://github.com/jacobperron/orb_slam.git).

## Depends ##

* [Pangolin](https://github.com/stevenlovegrove/Pangolin) (see [orb_slam/install-dependencies](https://github.com/jacobperron/orb_slam#install-dependencies) for instruction)
* [ROS](http://ros.org) (*indigo* or greater)
* catkin-tools (`sudo apt-get install python-catkin-tools`)

## Install (catkin) ##

```bash
# Setup catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws && catkin init
# Clone ORB-SLAM and this repository into src directory
$ git clone https://github.com/jacobperron/orb_slam.git src/orb_slam
$ git clone https://github.com/jacobperron/orb_slam_ros.git src/orb_slam_ros
# Build ORB-SLAM
$ cd src/orb_slam
$ ./build.sh
$ catkin build --this
# Build
$ catkin build orb_slam_ros
```

## Run ##

See example launch and configuration files in `orb_slam_ros/launch` and `orb_slam_ros/config` respectively.
