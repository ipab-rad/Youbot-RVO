/**
 * Created by Nantas Nardelli <n.nardelli@sms.ed.ac.uk>
 * \file   AMCLWrapper.h
 * \brief  Declares the AMCLWrapper class.
 */

#ifndef AMCL_WRAPPER_H_
#define AMCL_WRAPPER_H_

#include "Environment.h"
#include "Simulator.h"
#include <string>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include "Vector2.h"

namespace hrvo {

class AMCLWrapper
{

 public:
  AMCLWrapper();
  AMCLWrapper(std::string sub_name);
  ~AMCLWrapper();
  void receive_pose(const boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> pose_msg);
  void pretty_print_msg();
  void pretty_print_pose();
  void update();
  geometry_msgs::PoseWithCovarianceStamped get_full_msg();
  std_msgs::Header get_header();
  geometry_msgs::Pose get_full_pose();
  Vector2 get_position();
  boost::array<double, 36> get_cov();
  double get_orientation();
  void setEnvPointer(Environment *environment) {environment_ = environment;}
  void setPlannerPointer(Simulator *planner) {planner_ = planner;}


  bool initialised;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  geometry_msgs::PoseWithCovarianceStamped received_pose_;
  std_msgs::Header header_;
  geometry_msgs::Pose full_pose_;
  boost::array<double, 36> covariance_;

};

}

#endif /* AMCLWRAPPER_H_ */
