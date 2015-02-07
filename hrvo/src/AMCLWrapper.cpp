/**
* Created by Nantas Nardelli
* \file   AMCLWrapper.cpp
* \brief  Deals with the AMCL data
*/

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
  is_msg_received = false;
}

AMCLWrapper::AMCLWrapper(std::string sub_name)
{
  sub_ = nh_.subscribe("/" + sub_name + "/amcl_pose",
                       1,
                       &AMCLWrapper::receive_pose,
                       this);
  std::string info = "Subscribing to " + sub_name + " AMCL pose";
  ROS_INFO("%s", info.c_str());
  is_msg_received = false;
}

AMCLWrapper::~AMCLWrapper()
{

};

void AMCLWrapper::receive_pose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  // DEBUG("CALLBACK" << std::endl);
  is_msg_received = true;
  ERR("IS THIS CALLED?" << std::endl);
  received_pose_ = *pose_msg;
  // DEBUG("END_CALLBACK" << std::endl);
}

void AMCLWrapper::updatePose()
{
  // WARN("AMCLWrapper: update is called!" << std::endl);
  header_ = received_pose_.header;
  full_pose_ = received_pose_.pose.pose;
  covariance_ = received_pose_.pose.covariance;
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
  WARN("Cov: " << covariance_.data() << std::endl);
  WARN("[");
  for (boost::array<double, 36>::iterator i(covariance_.begin());
       i != covariance_.end(); ++i) {
    WARN(i);
    if (boost::next(i) != covariance_.end())
      std::cout << ',';
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
  Vector2 p = get_position();
  double o = get_orientation();
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
  if (!is_msg_received)
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
