// AgentsDispatcher.cpp --- 
// 
// Filename: AgentsDispatcher.cpp
// Description: Implementation of AgentsDispatcher.hpp
// Author: Federico Boniardi
// Maintainer: Federico Boniardi
// Created: Mon Jul  7 23:00:17 2014 (+0100)
// Version: 0.1.0
// Last-Updated: 
//           By: 
//     Update #: 0
// URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Change Log:
// 
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

#include <cmath>
#include <sstream>
#include <exception>

#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "agents_dispatcher/AgentsDispatcher.hpp"

namespace agents_dispatcher {

AgentsDispatcher::AgentsDispatcher() 
{
  if ( !nh_.getParam("/agentsDispatcher/ptracking_topic", agents_topic_) )
    agents_topic_ = "/agent_1/PTrackingBridge/targetEstimations";
  ROS_INFO_STREAM("subscribed to topic " << agents_topic_);

  if ( !nh_.getParam("/agentsDispatcher/frame_id", frame_id_) )
    frame_id_ = "/map";
  ROS_INFO_STREAM("reference frame " << frame_id_ );
  
  sub_ = nh_.subscribe(agents_topic_, 1, &AgentsDispatcher::getTargetEstimations, this);

#ifdef HRVO
  hrvo_add_agent_ = nh_.serviceClient<hrvo::AddAgentService>("hrvo_add_agent");
  ROS_INFO("connected to service 'hrvo_add_agent'");
#endif
  
  iteration_ = 0;
}

AgentsDispatcher::~AgentsDispatcher()
{
  ROS_INFO("shutting down agentsDispatcher");
}

void AgentsDispatcher::getTargetEstimations(const PTrackingBridge::TargetEstimations::ConstPtr& tgts_msg)
{
  iteration_++;
  for(unsigned int i=0; i<tgts_msg->identities.size(); ++i) {
    int index = tgts_msg->identities.at(i);

    path_[index].header.seq = iteration_;
    path_[index].header.stamp = ros::Time::now();
    path_[index].header.frame_id = frame_id_;

    geometry_msgs::PoseStamped pose_t;
    pose_t.header.seq = iteration_;
    pose_t.header.stamp = ros::Time::now();
    pose_t.header.frame_id = frame_id_;
    pose_t.pose.position.x = tgts_msg->positions.at(i).x;
    pose_t.pose.position.y = tgts_msg->positions.at(i).y;

    double v_x = tgts_msg->velocities.at(i).x;
    double v_y = tgts_msg->velocities.at(i).y;
    double yaw = v_x > 0 ? std::atan(v_y/v_x) : 0;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    pose_t.pose.orientation.x = q.x;
    pose_t.pose.orientation.y = q.y;
    pose_t.pose.orientation.z = q.z;
    pose_t.pose.orientation.w = q.w;

    path_[index].poses.push_back(pose_t);

    std::map<int, ros::Publisher>::iterator it = pub_.find(index);
    if ( it == pub_.end() ) {
      std::stringstream ss_id;
      ss_id << "/agentsDispatcher/agent_" << index;
      try {
        pub_[index] = nh_.advertise<nav_msgs::Path>(ss_id.str(), 1);
        ROS_INFO_STREAM("advertising the topic " << ss_id.str());
        
      } catch (std::exception e) {
        ROS_ERROR("unabled to publish the path for agent %d -- %s", index, e.what());
      }

#ifdef HRVO
      hrvo::AddAgentService srv;

      srv.request.id = index;
      srv.request.topic_id = ss_id.str();
      srv.request.position.x = tgts_msg->positions.at(i).x;
      srv.request.position.y = tgts_msg->positions.at(i).y;
      srv.request.velocity.x = tgts_msg->velocities.at(i).x;
      srv.request.velocity.y = tgts_msg->velocities.at(i).y;
                
      if ( hrvo_add_agent_.call(srv) ) {
        if ( srv.response.succeeded ) {
          ROS_INFO("added agent %d to the hrvo planner", index);
        } else {
          ROS_ERROR("failed to add agent %d to the hrvo planner", index);
        }
      } else  {
        ROS_ERROR("fail to contact the service hrvo_add_agent");
      }
#endif
    }
  }
}

} /* namespace agents_dispatcher */

// 
// AgentsDispatcher.cpp ends here
