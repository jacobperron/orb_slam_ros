#ifndef ORB_SLAM_ROS_MONO_SLAM_NODE_H
#define ORB_SLAM_ROS_MONO_SLAM_NODE_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>

#include "orb_slam/System.h"

class MonoSLAMNode
{
public:
  explicit MonoSLAMNode(ros::NodeHandle& nh);
  ~MonoSLAMNode();
  virtual void spin();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  image_transport::ImageTransport it_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_br_;

  ros::Publisher cam_pose_pub_;
  image_transport::Subscriber img_sub_;

private:
  geometry_msgs::PoseStamped cam_pose_msg_;
  orb_slam::System* slam_ptr_;

  std::string odom_frame_;
  std::string camera_frame_;
  std::string world_frame_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};  // class MonoORBSLAM
#endif  // ORB_SLAM_ROS_MONO_SLAM_NODE_H
