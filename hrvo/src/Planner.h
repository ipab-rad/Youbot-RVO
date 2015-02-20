/**
* Created by Alejandro Bordallo
* \file   Environment.h
* \brief  Declares the Environment class.
*/
#ifndef HRVO_PLANNER_H_
#define HRVO_PLANNER_H_

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace hrvo {

  class Planner
  {
    public:
      Planner(ros::NodeHandle nh);

      ~Planner();

      void sendGoal();
  };


}



#endif /* HRVO_PLANNER_H_ */