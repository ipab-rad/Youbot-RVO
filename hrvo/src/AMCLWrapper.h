/**
 * Created by Nantas Nardelli <n.nardelli@sms.ed.ac.uk>
 * \file   AMCLWrapper.h
 * \brief  Declares the AMCLWrapper class.
 */

#ifndef AMCL_WRAPPER_H_
#define AMCL_WRAPPER_H_

#include <string>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

namespace hrvo {

class AMCLWrapper
{

 public:
  AMCLWrapper();
  AMCLWrapper(std::string sub_name);
  ~AMCLWrapper();
  void receive_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
  void pretty_print();
  void update();
  geometry_msgs::PoseWithCovarianceStamped get_full_msg();
  std_msgs::Header get_header();
  geometry_msgs::Pose get_pose();
  boost::array<double, 36> get_cov();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  geometry_msgs::PoseWithCovarianceStamped received_pose_;
  std_msgs::Header header_;
  geometry_msgs::Pose pose_;
  boost::array<double, 36> covariance_;

};

}

#endif /* AMCLWRAPPER_H_ */
