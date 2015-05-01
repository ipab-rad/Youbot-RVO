/**
 * Created by Nantas Nardelli <n.nardelli@sms.ed.ac.uk>
 * \file   BumperWrapper.h
 * \brief  Declares the BumperWrapper class.
 */

#ifndef BUMPER_WRAPPER_H_
#define BUMPER_WRAPPER_H_

#include "Environment.h"
#include "Simulator.h"
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include "Vector2.h"

#include <string>

namespace hrvo {
class Environment;
class Simulator;

class BumperWrapper {
 public:
  BumperWrapper();
  explicit BumperWrapper(std::string sub_name);
  ~BumperWrapper();
  void receive_data(const std_msgs::Int32MultiArray::ConstPtr& msg);
  void pretty_print();
  void update_data();
  void setEnvPointer(Environment *environment) {environment_ = environment;}
  void setPlannerPointer(Simulator *planner) {planner_ = planner;}
  int activated();
  bool is_msg_received;
  int callback_counter;

 private:
  std_msgs::Int32MultiArray received_data_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  Environment* environment_;
  Simulator* planner_;
  std::vector<int> bumper_data_;
  std::vector<std::string> POSITIONS;
};

}  // namespace hrvo

#endif /* BumperWRAPPER_H_ */
