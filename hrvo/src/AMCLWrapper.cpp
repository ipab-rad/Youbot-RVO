/**
* Created by Nantas Nardelli
* \file   AMCLWrapper.cpp
* \brief  Deals with the AMCL data
*/

#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

#include "AMCLWrapper.h"
#include "Definitions.h"


namespace hrvo {

AMCLWrapper::AMCLWrapper()
{
  sub_ = amcl_nh_.subscribe("/amcl_pose",
                       1,
                       &AMCLWrapper::receive_pose,
                       this);
  ROS_INFO("Subscribing to default AMCL pose");

  odom_sub_ = nh_.subscribe("/odom", 1,
                           &AMCLWrapper::receive_odom, this);

  ROS_INFO("Subsribing to default Odom pose");
  is_msg_received = false;
  is_odom_received = false;
  callback_counter = 0;
}


AMCLWrapper::AMCLWrapper(std::string sub_name)
{
  odom_sub_ = nh_.subscribe("/" + sub_name + "/odom", 1,
                           &AMCLWrapper::receive_odom, this);
  std::string info = "Subscribing to " + sub_name + " odom updates";
  ROS_INFO("%s", info.c_str());

  sub_ = amcl_nh_.subscribe("/" + sub_name + "/amcl_pose",
                       1,
                       &AMCLWrapper::receive_pose,
                       this);
  info = "Subscribing to " + sub_name + " AMCL pose";
  ROS_INFO("%s", info.c_str());

  callback_counter = 0;
  is_msg_received = false;
  is_odom_received = false;
}


AMCLWrapper::~AMCLWrapper()
{

};


void AMCLWrapper::receive_pose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  // callback_counter++;
  DEBUG("p.x: " << pose_msg->pose.pose.position.x << std::endl);
  DEBUG("p.y: " << pose_msg->pose.pose.position.y << std::endl);
  DEBUG("p.z: " << pose_msg->pose.pose.position.z << std::endl);
  if (!(pose_msg->pose.pose.position.x == 0 &&
    pose_msg->pose.pose.position.y == 0 &&
    pose_msg->pose.pose.position.z == 0)) {
    is_msg_received = true;
    received_pose_ = *pose_msg;
    callback_counter = 0;
  }
  else {
  //   INFO("Callback counter = " << callback_counter << std::endl);
  // }

  // if (callback_counter > 1){
    is_msg_received = false;
    // callback_counter = 0;
  }
}


void AMCLWrapper::receive_odom(
    const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  is_odom_received = true;
  received_odom_ = *odom_msg;
}


void AMCLWrapper::updatePose()
{
// WARN("AMCLWrapper: update is called!" << std::endl);

header_ = received_pose_.header;
full_pose_ = received_pose_.pose.pose;
covariance_ = received_pose_.pose.covariance;
// DEBUG("p.x: " << full_pose_.position.x << std::endl);
// DEBUG("p.y: " << full_pose_.position.y << std::endl);
// DEBUG("p.z: " << full_pose_.position.z << std::endl);

if (!(full_pose_.position.x == 0.0 &&
      full_pose_.position.y == 0.0 &&
      full_pose_.position.z == 0.0)) {
  WARN("AMCLWrapper: using AMCL!" << std::endl);
}
else {
  // assuming odometry has been received
  WARN("AMCLWrapper: using odometry!" << std::endl);
  full_pose_ = received_odom_.pose.pose;
  }
}


void AMCLWrapper::pretty_print_msg()
{
  ERR("--------------------" << std::endl);
  DEBUG("Pose from AMCL:" << std::endl);
  DEBUG("p.x: " << full_pose_.position.x << std::endl);
  DEBUG("p.y: " << full_pose_.position.y << std::endl);
  DEBUG("p.z: " << full_pose_.position.z << std::endl);
  DEBUG("o.x: " << full_pose_.orientation.x << std::endl);
  DEBUG("o.y: " << full_pose_.orientation.y << std::endl);
  DEBUG("o.z: " << full_pose_.orientation.z << std::endl);
  DEBUG("o.w: " << full_pose_.orientation.w << std::endl);
  WARN("Covariance from AMCL:" << std::endl);
  WARN("[");
  int count = 0;
  for (boost::array<double, 36>::iterator i(covariance_.begin());
       i != covariance_.end(); ++i) {
    count += 1;
    WARN(i);

    if (boost::next(i) != covariance_.end())
      std::cout << ',';
    if(count % 4 == 0 && count != 35) { WARN(std::endl); }
  }
  WARN("]" << std::endl);
  ERR("--------------------" << std::endl);
}


void AMCLWrapper::pretty_print_pose()
{
  if (!is_msg_received)
  {
    Vector2 p = this->get_position();
    double o = this->get_orientation();
    ERR("--------------------" << std::endl);
    WARN("Pose from odometry:" << std::endl);
    WARN("p.x: " << p.getX() << std::endl);
    WARN("p.y: " << p.getY() << std::endl);
    WARN("Yaw: " << o << std::endl);
    ERR("--------------------" << std::endl);
}
else {
Vector2 p = this->get_position();
double o = this->get_orientation();
ERR("--------------------" << std::endl);
WARN("Pose from AMCL in 2D:" << std::endl);
WARN("p.x: " << p.getX() << std::endl);
WARN("p.y: " << p.getY() << std::endl);
WARN("Yaw: " << o << std::endl);
ERR("--------------------" << std::endl);
}
}


geometry_msgs::PoseWithCovarianceStamped AMCLWrapper::get_full_msg()
{
  return received_pose_;
}


std_msgs::Header AMCLWrapper::get_header()
{
  return header_;
}


geometry_msgs::Pose AMCLWrapper::get_full_pose()
{
  return full_pose_;
}


Vector2 AMCLWrapper::get_position()
{
  return Vector2(full_pose_.position.x, full_pose_.position.y);
}


double AMCLWrapper::get_orientation()
{
  if (is_msg_received)
  {
    return tf::getYaw(full_pose_.orientation);
  }
  return 0.0;
}


boost::array<double, 36> AMCLWrapper::get_cov()
{
  return covariance_;
}
}
