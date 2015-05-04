/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-04
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Defines the Planner class.
*/

#include "Planner.h"

namespace hrvo {

Planner::Planner(ros::NodeHandle nh) {
  goal_sent = false;
  nh_ = nh;

  // Tell the action client that we want to spin a thread by default
  acPointer_ = new MoveBaseClient("move_base", true);
}

Planner::~Planner() {
}

void Planner::sendNewGoal(Vector2 vGoal) {
  // //wait for the action server to come up
  while (!acPointer_->waitForServer(ros::Duration(2.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal mgoal;

  // //we'll send a goal to the robot to move 1 meter forward
  mgoal.target_pose.header.frame_id = "base_link";
  mgoal.target_pose.header.stamp = ros::Time::now();

  mgoal.target_pose.pose.position.x = vGoal.getX();
  // mgoal.target_pose.pose.position.y = vGoal.getY();
  // mgoal.target_pose.pose.position.x = 1.0;
  mgoal.target_pose.pose.position.y = vGoal.getY();
  mgoal.target_pose.pose.orientation.w = 1.0;

  // if (!goal_sent)
  // {INFO("Sending goal");
  // ac.sendGoal(mgoal);}

  if (!goal_sent) {
    INFO("Sending goal");
    acPointer_->sendGoal(mgoal);
  }
  // acPointer_->sendGoal(mgoal);

  // ac.waitForResult();

  if (acPointer_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot already at goal");
    acPointer_->cancelGoal();
  }
}

void Planner::cancelGoal() {
  acPointer_->cancelGoal();
}

GoalStateEnum Planner::checkGoalState() {
  // INFO("GOAL " << acPointer_->getState().state_);
  INFO("GOAL ");
  GoalStateEnum goalstate = acPointer_->getState().state_;
  switch (goalstate) {
  case GoalState::PENDING:
    WARN("PENDING")
    break;
  case GoalState::ACTIVE:
    INFO("ACTIVE")
    break;
  case GoalState::RECALLED:
    WARN("RECALLED")
    break;
  case GoalState::REJECTED:
    ERR("REJECTED")
    break;
  case GoalState::PREEMPTED:
    WARN("PREEMPTED")
    break;
  case GoalState::ABORTED:
    ERR("ABORTED")
    break;
  case GoalState::SUCCEEDED:
    INFO("SUCCEEDED")
    break;
  case GoalState::LOST:
    ERR("LOST")
    break;
  default:
    ERR("INVALID!");
    break;
  }
  INFO(std::endl);
  return goalstate;
}
}  // namespace hrvo
