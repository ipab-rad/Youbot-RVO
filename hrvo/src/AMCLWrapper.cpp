/**
* Created by Nantas Nardelli
* \file   AMCLWrapper.cpp
* \brief  Deals with the AMCL data
*/

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
}

AMCLWrapper::AMCLWrapper(std::string sub_name)
{
  sub_ = nh_.subscribe(sub_name + "/amcl_pose",
                       1,
                       &AMCLWrapper::receive_pose,
                       this);
  std::string info = "Subscribing to " + sub_name + " AMCL pose";
  ROS_INFO("%s", info.c_str());
}

AMCLWrapper::~AMCLWrapper() {};

void AMCLWrapper::receive_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
  received_pose_ = *pose_msg;
  update();
}

void AMCLWrapper::pretty_print()
{
  ERR("--------------------" << std::endl);
  ERR("Pose from AMCL:" << std::endl);
  ERR("p.x: " << pose_.position.x << std::endl);
  ERR("p.y: " << pose_.position.y << std::endl);
  ERR("p.z: " << pose_.position.z << std::endl);
  ERR("o.x: " << pose_.orientation.x << std::endl);
  ERR("o.y: " << pose_.orientation.y << std::endl);
  ERR("o.z: " << pose_.orientation.z << std::endl);
  ERR("o.w: " << pose_.orientation.w << std::endl);
  WARN("Covariance from AMCL:" << std::endl);
  WARN("Cov: " << covariance_.data() << std::endl);
  WARN("[");
  for (boost::array<double, 36>::iterator i(covariance_.begin()); i != covariance_.end(); ++i) {
    WARN(i);
    if (boost::next(i) != covariance_.end())
      std::cout << ',';
  }
  WARN("]" << std::endl);
  ERR("--------------------" << std::endl);
}

void AMCLWrapper::update()
{
  header_ = received_pose_.header;
  pose_ = received_pose_.pose.pose;
  covariance_ = received_pose_.pose.covariance;
}

geometry_msgs::PoseWithCovarianceStamped AMCLWrapper::get_full_msg()
{
  return received_pose_;
}

std_msgs::Header AMCLWrapper::get_header()
{
  return header_;
}

geometry_msgs::Pose AMCLWrapper::get_pose()
{
  return pose_;
}

boost::array<double, 36> AMCLWrapper::get_cov()
{
  return covariance_;
}

}
