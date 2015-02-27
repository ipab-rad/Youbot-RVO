/**
* Created by Nantas Nardelli
* \file   BumperWrapper.cpp
* \brief  Deals with the Bumper bumper_data_
*/

#include <numeric>

#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

#include "BumperWrapper.h"
#include "Definitions.h"

namespace hrvo {

BumperWrapper::BumperWrapper(std::string sub_name)
{
  sub_ = nh_.subscribe("/" + sub_name + "/bumper_kilt", 1,
                           &BumperWrapper::receive_data, this);
  std::string info = "Subscribing to " + sub_name + " bumper kilt";
  ROS_INFO("%s", info.c_str());
  is_msg_received = false;
  bumper_data_.resize(8, 0);
  std::string positions[] = {"TOP",
                             "TOP-RIGHT",
                             "RIGHT",
                             "BOTTOM-RIGHT",
                             "BOTTOM",
                             "LEFT-BOTTOM",
                             "LEFT",
                             "TOP-LEFT"};
  for (int i = 0; i < 8; i++) {
    POSITIONS.push_back(positions[i]);
  }
}

BumperWrapper::~BumperWrapper()
{
};


void BumperWrapper::receive_data(
    const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  is_msg_received = true;
  received_data_ = *msg;
}

void BumperWrapper::update_data()
{
  if (is_msg_received) {
    bumper_data_ = received_data_.data;
  }
}


void BumperWrapper::pretty_print()
{
  ERR("--------------------" << std::endl);
  WARN("BUMPER KILT STATE:" << std::endl);
  std::string value;
  for(int i = 0; i < 8; i++) {
    DEBUG(POSITIONS[i] << ":\t");
    if (bumper_data_[i] == 1) {
      ERR("ACTIVE");
    }
    else {
      DEBUG("INACTIVE");
    }
    DEBUG(std::endl);
  }
  ERR("--------------------" << std::endl);
}

int BumperWrapper::activated()
{
  int sum_of_elems = 0 ;
  for(int i = 0; i < 8; i++) {
    sum_of_elems += bumper_data_[i];
    if (sum_of_elems > 2)
      return 9;
  }

  for(int i = 0; i < 8; i++) {
    int value = bumper_data_[i];
    if (value)
      return i+1;
  }

  return 0;
}
}
