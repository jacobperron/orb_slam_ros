#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "orb_slam/System.h"

#include "orb_slam_ros/mono_slam_node.h"

MonoSLAMNode::MonoSLAMNode(ros::NodeHandle& nh) :
  nh_(nh),
  priv_nh_("~"),
  it_(nh),
  tf_listener_(tf_buffer_),
  tf_br_()
{
  std::string settings_file;
  std::string vocab_file;
  bool use_pangolin_viewer;

  priv_nh_.param<std::string>("settings_file", settings_file, "");
  priv_nh_.param<std::string>("vocab_file", vocab_file, "");
  priv_nh_.param<bool>("use_pangolin_viewer", use_pangolin_viewer, true);
  priv_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
  priv_nh_.param<std::string>("camera_frame", camera_frame_, "camera_optical");
  priv_nh_.param<std::string>("world_frame", world_frame_, "orb_slam");

  bool do_shutdown = false;

  if (settings_file.empty())
  {
    ROS_ERROR("[MONO_SLAM_NODE] Settings file not provided");
    do_shutdown = true;
  }

  if (vocab_file.empty())
  {
    ROS_ERROR("[MONO_SLAM_NODE] Vocabulary file not provided");
    do_shutdown = true;
  }

  if (do_shutdown)
  {
    ros::shutdown();
  }

  // Initialize SLAM system
  slam_ptr_ = new orb_slam::System(vocab_file, settings_file, orb_slam::System::MONOCULAR, use_pangolin_viewer);

  img_sub_ = it_.subscribe("camera/image_raw", 60, &MonoSLAMNode::imageCallback, this);
  cam_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("camera/pose", 60);
}

MonoSLAMNode::~MonoSLAMNode()
{
  if (slam_ptr_)
  {
    // Stop all threads
    slam_ptr_->Shutdown();

    // Save camera trajectory
    slam_ptr_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    delete slam_ptr_;
  }
}

void MonoSLAMNode::spin()
{
  ros::spin();
}

void MonoSLAMNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat_<float> cam_pose = slam_ptr_->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

  if (cam_pose.empty()) return;

  // Convert camera pose to right-handed coordinate system
  cv::Mat_<float> cam_rotation = cam_pose(cv::Range(0, 3), cv::Range(0, 3));
  tf2::Matrix3x3 tf_mat_lh(cam_pose(0, 0), cam_pose(0, 1), cam_pose(0, 2),
                           cam_pose(1, 0), cam_pose(1, 1), cam_pose(1, 2),
                           cam_pose(2, 0), cam_pose(2, 1), cam_pose(2, 2));
  double r, p, y;
  tf_mat_lh.getRPY(r, p, y);
  tf2::Matrix3x3 tf_mat_rh;
  tf_mat_rh.setRPY(-r, -p, y);
  tf2::Quaternion tf_quat;
  tf_mat_rh.getRotation(tf_quat);
  tf2::Transform world_to_camera_tf(tf_quat, tf2::Vector3(cam_pose(0, 3), cam_pose(1, 3), -cam_pose(2, 3)));

  // Lookup camera frame to odom
  tf2::Transform camera_to_odom_tf;
  try
  {
    const geometry_msgs::TransformStamped camera_to_odom_msg = tf_buffer_.lookupTransform(
        camera_frame_,
        odom_frame_,
        ros::Time(0),
        ros::Duration(0.05));
    tf2::convert(camera_to_odom_msg.transform, camera_to_odom_tf);
  }
  catch (const tf2::TransformException &ex)
  {
    ROS_ERROR_STREAM("[MONO_SLAM_NODE] Failed to lookup transform: " << ex.what());
  }

  // Compute world to odom transform
  tf2::Transform world_to_odom_tf = world_to_camera_tf * camera_to_odom_tf;

  // Publish TF
  geometry_msgs::TransformStamped world_to_odom_msg;
  tf2::convert(world_to_odom_tf, world_to_odom_msg.transform);
  world_to_odom_msg.header.stamp = ros::Time::now();
  world_to_odom_msg.header.frame_id = world_frame_;
  world_to_odom_msg.child_frame_id = odom_frame_;
  tf_br_.sendTransform(world_to_odom_msg);

  // Publish pose
  tf2::convert(tf_quat, cam_pose_msg_.pose.orientation);
  cam_pose_msg_.pose.position.x = cam_pose(0, 3);
  cam_pose_msg_.pose.position.y = cam_pose(1, 3);
  cam_pose_msg_.pose.position.z = -cam_pose(2, 3);
  cam_pose_msg_.header.stamp = ros::Time::now();
  cam_pose_msg_.header.frame_id = world_frame_;
  cam_pose_pub_.publish(cam_pose_msg_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mono_slam_node");
  ros::NodeHandle nh;

  MonoSLAMNode slam_node(nh);

  try
  {
    slam_node.spin();
  }
  catch (std::runtime_error& ex)
  {
    ROS_FATAL_STREAM("[MONO_SLAM_NODE] Runtime error: " << ex.what());
    return 1;
  }

  return 0;
}
