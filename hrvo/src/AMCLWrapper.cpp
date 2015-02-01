/**
* Created by Nantas Nardelli
* \file   AMCLWrapper.cpp
* \brief  Deals with the AMCL data
*/

#include "AMCLWrapper.h"

namespace hrvo {

AMCLWrapper::AMCLWrapper()
{
  sub_ = nh_.subscribe("/amcl_node",
                      1,
                      &AMCLWrapper::receive_pose,
                      this);
  ROS_INFO("Subscribing to default AMCL node");
}

AMCLWrapper::AMCLWrapper(std::string sub_name)
{
  sub_ = nh_.subscribe(sub_name + "/amcl_node",
                      1,
                      &AMCLWrapper::receive_pose,
                      this);
  std::string info = "Subscribing to " + sub_name + " AMCL node";
  ROS_INFO("%s", info.c_str());
}

AMCLWrapper::~AMCLWrapper() {};

void AMCLWrapper::receive_pose(const geometry_msgs::Twist::ConstPtr &pose_msg)
{
  pose_ = *pose_msg;
}

void AMCLWrapper::update()
{
  ROS_INFO("AMCLWrapper.update() NOT READY YET");
}

}
