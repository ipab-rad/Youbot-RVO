/**
* Created by Alejandro Bordallo
* \file   Planner.cpp
* \brief  Defines the Planner class.
*/

#include "Planner.h"


namespace hrvo {

  Planner::Planner(ros::NodeHandle nh)
  {
    ;
  }

  Planner::~Planner()
  {
    ;
  }

  void Planner::sendGoal()
  {
    //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", false);

  // //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  // ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {ROS_INFO("Hooray, the base moved 1 meter forward");}
  else
    {ROS_INFO("The base failed to move forward 1 meter for some reason");}
  }
}