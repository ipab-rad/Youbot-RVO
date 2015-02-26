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
  odCount = 0;
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

  odCount = 0;
  is_msg_received = false;
  is_odom_received = false;
}


AMCLWrapper::~AMCLWrapper()
{

};


void AMCLWrapper::receive_pose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  // ERR("AMCL RECEIVED");
  // DEBUG("p.x: " << pose_msg->pose.pose.position.x << std::endl);
  // DEBUG("p.y: " << pose_msg->pose.pose.position.y << std::endl);
  // DEBUG("p.z: " << pose_msg->pose.pose.position.z << std::endl);
  if (!(pose_msg->pose.pose.position.x == 0.0 &&
    pose_msg->pose.pose.position.y == 0.0))
  {
    is_msg_received = true;
    received_pose_ = *pose_msg;
    // DEBUG("RECEIVED POSE=" << received_pose_.pose.pose << std::endl);
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
  usleep(8000);
  // WARN("AMCLWrapper: update is called!" << std::endl);
  WARN("AMCL:" << is_msg_received << " ODOM:" << is_odom_received << std::endl);
  // DEBUG("p.x: " << full_pose_.position.x << std::endl);
  // DEBUG("p.y: " << full_pose_.position.y << std::endl);
  // DEBUG("p.z: " << full_pose_.position.z << std::endl);

  // if (!(full_pose_.position.x == 0.0 &&
  //       full_pose_.position.y == 0.0)) 
  // is_msg_received = false;
  if (is_msg_received)
  {
    WARN("AMCLWrapper: using AMCL!" << std::endl);
    header_ = received_pose_.header;
    full_pose_ = received_pose_.pose.pose;
    // DEBUG("FULL POSE=" << full_pose_ << std::endl);
    covariance_ = received_pose_.pose.covariance;
  }
  if (is_odom_received)
  {
    // assuming odometry has been received
    WARN("AMCLWrapper: using odometry!" << std::endl);
    odom_pose_ = received_odom_.pose.pose;
  }
  if (!is_msg_received && !is_odom_received)
  {
    ERR("NO AMCL OR ODOMETRY RECEIVED!" << std::endl);
  }
}


void AMCLWrapper::pretty_print_msg()
{
  ERR("--------------------" << std::endl);
  WARN("Message received:" << is_msg_received << std::endl);
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
  // if (is_msg_received) {odCount++;} else {odCount = 0;}
  // DEBUG("Od Count: " << odCount << std::endl);
  if (is_msg_received)
  {
    Vector2 p = this->get_position();
    double o = this->get_orientation();
    ERR("--------------------" << std::endl);
    WARN("Pose from AMCL:" << std::endl);
    WARN("p.x: " << p.getX() << std::endl);
    WARN("p.y: " << p.getY() << std::endl);
    WARN("Yaw: " << o << std::endl);
    ERR("--------------------" << std::endl);
  }
  else if (is_odom_received)
  {
    Vector2 p = this->get_odom_position();
    // double o = this->get_odom_orientation();
    ERR("--------------------" << std::endl);
    WARN("Offset from Odom:" << std::endl);
    WARN("p.x: " << p.getX() << std::endl);
    WARN("p.y: " << p.getY() << std::endl);
    WARN("Yaw: NIL" << std::endl);
    ERR("--------------------" << std::endl);
  }
  else
  {
    ERR("--------------------" << std::endl);
    WARN("NO DATA" << std::endl);
    WARN("NO DATA" << std::endl);
    WARN("NO DATA" << std::endl);
    WARN("NO DATA" << std::endl);
    ERR("--------------------" << std::endl);
  }
  is_msg_received = false; 
  is_odom_received = false;
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

Vector2 AMCLWrapper::get_odom_position()
{
  return Vector2(odom_pose_.position.x, odom_pose_.position.y);
}


double AMCLWrapper::get_odom_orientation()
{
  // if (is_odom_received)
  // {
  //   return tf::getYaw(odom_pose_.orientation);
  // }
  return 0.0;
}


boost::array<double, 36> AMCLWrapper::get_cov()
{
  return covariance_;
}
}
