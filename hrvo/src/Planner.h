/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-04
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Declares the Planner class.
*/

#ifndef HRVO_PLANNER_H_
#define HRVO_PLANNER_H_

#include <ros/ros.h>
#include "Vector2.h"

#include "Definitions.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
MoveBaseClient;

typedef actionlib::SimpleClientGoalState::StateEnum GoalStateEnum;
typedef actionlib::SimpleClientGoalState GoalState;

namespace hrvo {

class Planner {
 public:
  explicit Planner(ros::NodeHandle nh);

  ~Planner();

  void sendNewGoal(Vector2 vgoal);

  void cancelGoal();

  GoalStateEnum checkGoalState();

 private:
  bool goal_sent;

  // int goalState;

  MoveBaseClient* acPointer_;

  ros::NodeHandle nh_;
};


}  // namespace hrvo



#endif /* HRVO_PLANNER_H_ */
