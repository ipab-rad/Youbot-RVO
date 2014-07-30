// AgentsDispatcher.hpp --- 
// 
// Filename: AgentDispatcher.h
// Description: Send path info to the ros core
// Author: Federico Boniardi
// Maintainer: Federico Boniardi
// Created: Mon Jul  7 22:43:37 2014 (+0100)
// Version: 0.1.0
// Last-Updated: Tue Jul 8 02:27:04 2014 (+0100)
//           By: Federico Boniardi
//     Update #: 1
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
//   . Subscribe to PTrackingBridge/targetsEstimation
//   . Publish independent topics for each agent detected
//   . If HRVO is defined, invoke remote service in hrvo to
//     create a new agent
// 

// Change Log:
//      changed folders and namespaces to ptracking_wrapper
// 
// 
// 
// The MIT License (MIT)
// 
// Copyright (c) 2014 Federico Boniardi
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// 

// Code:

#ifndef AGENTSDISPATCHER_HPP_
#define AGENTSDISPATCHER_HPP_

#include <map>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "PTrackingBridge/TargetEstimations.h"

#define HRVO

#ifdef HRVO
#include "hrvo/AddAgentService.h"
#endif

namespace ptracking_wrapper {

class AgentsDispatcher
{
 public:
  AgentsDispatcher();
  virtual ~AgentsDispatcher();
  void getTargetEstimations(const PTrackingBridge::TargetEstimations::ConstPtr&);
 private:
  ros::NodeHandle nh_;
  std::map<int, ros::Publisher> pub_;
  ros::Subscriber sub_;
  std::map<int, nav_msgs::Path> path_;
  int iteration_;
#ifdef HRVO
  ros::ServiceClient hrvo_add_agent_;
#endif
  // external parameters
  std::string agents_topic_;
  std::string frame_id_;
  bool rviz_;
};

} /* namespace ptracking_wrapper */

#endif /* AGENTSDISPATCHER_HPP_ */

// 
// AgentsDispatcher.hpp ends here
