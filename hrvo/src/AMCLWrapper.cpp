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

void AMCLWrapper::receive_pose(const geometry_msgs::Twist::ConstPtr &pose_msg)
{
  pose_ = *pose_msg;
}

void AMCLWrapper::pretty_print()
{
    ERR("l.x: " << pose_.linear.x << std::endl);
    ERR("l.y: " << pose_.linear.y << std::endl);
    ERR("l.z: " << pose_.linear.z << std::endl);
    WARN("a.x: " << pose_.angular.x << std::endl);
    WARN("a.y: " << pose_.angular.y << std::endl);
    WARN("a.z: " << pose_.angular.z << std::endl);
}

void AMCLWrapper::update()
{

  ROS_INFO("AMCLWrapper.update() NOT READY YET");
}

}
