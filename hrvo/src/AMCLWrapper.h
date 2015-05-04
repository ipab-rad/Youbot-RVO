/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-04
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Declares the AMCLWrapper class (By Nantas Nardelli)
*/

#ifndef AMCL_WRAPPER_H_
#define AMCL_WRAPPER_H_

#include "Environment.h"
#include "Simulator.h"
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "Vector2.h"

#include <string>

namespace hrvo {
class Environment;
class Simulator;

class AMCLWrapper {
 public:
  AMCLWrapper();
  explicit AMCLWrapper(std::string sub_name);
  ~AMCLWrapper();
  void receive_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&
                    pose_msg);
  void receive_odom(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void pretty_print_msg();
  void pretty_print_pose();
  void updatePose();
  geometry_msgs::PoseWithCovarianceStamped get_full_msg();
  std_msgs::Header get_header();
  geometry_msgs::Pose get_full_pose();
  Vector2 get_position();
  boost::array<double, 36> get_cov();
  double get_orientation();
  Vector2 get_odom_position();
  double get_odom_orientation();
  void setEnvPointer(Environment *environment) {environment_ = environment;}
  void setPlannerPointer(Simulator *planner) {planner_ = planner;}
  bool is_msg_received;
  bool is_odom_received;
  int callback_counter;

 private:
  std::size_t odCount;
  ros::NodeHandle nh_;
  ros::NodeHandle amcl_nh_;
  ros::Subscriber sub_;
  ros::Subscriber odom_sub_;
  geometry_msgs::PoseWithCovarianceStamped received_pose_;
  nav_msgs::Odometry received_odom_;

  std_msgs::Header header_;
  geometry_msgs::Pose full_pose_;
  geometry_msgs::Pose odom_pose_;
  boost::array<double, 36> covariance_;
  Environment* environment_;
  Simulator* planner_;
};

}  // namespace hrvo

#endif /* AMCLWRAPPER_H_ */
