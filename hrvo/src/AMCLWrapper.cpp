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
  sub_ = nh_.subscribe("/amcl_pose",
                       1,
                       &AMCLWrapper::receive_pose,
                       this);
  ROS_INFO("Subscribing to default AMCL pose");

  odom_sub_ = nh_.subscribe("/odom", 1,
                           &AMCLWrapper::receive_odom, this);

  ROS_INFO("Subsribing to default Odom pose");
  is_msg_received = false;
  is_odom_received = false;
}


AMCLWrapper::AMCLWrapper(std::string sub_name)
{
  sub_ = nh_.subscribe("/" + sub_name + "/amcl_pose",
                       1,
                       &AMCLWrapper::receive_pose,
                       this);
  std::string info = "Subscribing to " + sub_name + " AMCL pose";
  ROS_INFO("%s", info.c_str());

  odom_sub_ = nh_.subscribe("/" + sub_name + "/odom", 1,
                           &AMCLWrapper::receive_odom, this);
  info = "Subscribing to " + sub_name + " odom updates";
  ROS_INFO("%s", info.c_str());
  is_msg_received = false;
  is_odom_received = false;
}


AMCLWrapper::~AMCLWrapper()
{

};


void AMCLWrapper::receive_pose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  is_msg_received = true;
  received_pose_ = *pose_msg;
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
if (full_pose_.position.x != 0.0 &&
        full_pose_.position.y != 0.0 &&
        full_pose_.position.z != 0.0) {
is_msg_received = true;
}
  else {
    // assume odometry has been received
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
    ERR("Tried to print non-initialised AMCL pose!");
    return;
  }
  Vector2 p = this->get_position();
  double o = this->get_orientation();
  ERR("--------------------" << std::endl);
  WARN("Pose from AMCL in 2D:" << std::endl);
  WARN("p.x: " << p.getX() << std::endl);
  WARN("p.y: " << p.getY() << std::endl);
  WARN("Yaw: " << o << std::endl);
  ERR("--------------------" << std::endl);
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
  // WARN("AMCLWrapper: get_position is called!" << std::endl);
  // ERR("p.x: " << full_pose_.position.x << std::endl);
  // ERR("x" << full_pose_.position.x);
  // ERR("y" << full_pose_.position.y);
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
