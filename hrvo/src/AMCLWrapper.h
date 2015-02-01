/**
 * Created by Nantas Nardelli <n.nardelli@sms.ed.ac.uk>
 * \file   AMCLWrapper.h
 * \brief  Declares the AMCLWrapper class.
 */

#ifndef AMCL_WRAPPER_H_
#define AMCL_WRAPPER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>

namespace hrvo {

class AMCLWrapper
{
 public:
  AMCLWrapper();
  AMCLWrapper(std::string sub_name);
  ~AMCLWrapper();
  void update();
  void receive_pose(const geometry_msgs::Twist::ConstPtr& pose_msg);
  void pretty_print();
 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  geometry_msgs::Twist pose_;
};

}

#endif /* AMCLWRAPPER_H_ */
