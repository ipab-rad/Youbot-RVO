/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-04
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Declaration of functions for inter-robot pose/velocity comms
*/

#ifndef HRVO_ROBOTCOMMS_H_
#define HRVO_ROBOTCOMMS_H_

#include "Definitions.h"
#include "Parameter.h"
#include "Simulator.h"
#include "Environment.h"

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <map>

namespace hrvo {

class RobotComms {
 public:
  RobotComms(const Actor* nActorID, const std::string* sActorID);
  ~RobotComms();

  void publishPose();

  void initRobotTrackers();

  void receiveRobot1Pose(const geometry_msgs::Pose& msg);
  void receiveRobot2Pose(const geometry_msgs::Pose& msg);
  void receiveRobot3Pose(const geometry_msgs::Pose& msg);
  void receiveRobot4Pose(const geometry_msgs::Pose& msg);
  void receiveRobot5Pose(const geometry_msgs::Pose& msg);
  void receiveRobot6Pose(const geometry_msgs::Pose& msg);

  void receiveRobot1Vel(const geometry_msgs::Twist& msg);
  void receiveRobot2Vel(const geometry_msgs::Twist& msg);
  void receiveRobot3Vel(const geometry_msgs::Twist& msg);
  void receiveRobot4Vel(const geometry_msgs::Twist& msg);
  void receiveRobot5Vel(const geometry_msgs::Twist& msg);
  void receiveRobot6Vel(const geometry_msgs::Twist& msg);

  void setEnvPointer(Environment* environment) {environment_ = environment;}
  void setPlannerPointer(Simulator* planner) {planner_ = planner;}

 private:
// Private members
  size_t startGoal_; // Default goal
  ros::NodeHandle nh_;
  const Actor* nActorID_;
  const std::string* sActorID_;

// Class pointers
  Environment* environment_;
  Simulator* planner_;

// ROS Publishers
  ros::Publisher posePub_;

// ROS Subscribers
  std::map<Actor, std::size_t> trackedRobots_; // Robot Name, AgentID in planner
  std::map<std::size_t, ros::Subscriber> robotPoseSubs;   // AgentID
  std::map<std::size_t, ros::Subscriber> robotVelSubs;    // AgentID
  std::map<std::size_t, Vector2>* robotPoses;              // AgentID
  std::map<std::size_t, Vector2>* robotVels;               // AgentID

};

}  // namespace hrvo

#endif /* HRVO_ROBOTCOMMS_H_ */
