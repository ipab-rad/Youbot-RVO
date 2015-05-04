/**
* @Copyright: "Copyright[2015]<Alejandro Bordallo>"
* Created by Alejandro Bordallo
* \file   Environment.h
* \brief  Declares the Environment class.
*/
#ifndef HRVO_ENVIRONMENT_H_
#define HRVO_ENVIRONMENT_H_

#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>

#include "AMCLWrapper.h"
#include "Tracker.h"

#include "Simulator.h"

#include "Agent.h"

#include "PTrackingBridge/TargetEstimations.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"

#include "Parameter.h"

#include "Definitions.h"

#include "Planner.h"
#include "BumperWrapper.h"

typedef actionlib::SimpleClientGoalState::StateEnum GoalStateEnum;
typedef actionlib::SimpleClientGoalState GoalState;

namespace hrvo {
class BumperWrapper;
class AMCLWrapper;
class Tracker;
class Simulator;
class Agent;
class Goal;

class Environment {
 public:
  Environment(enum Actor actorID, const Vector2 startPos);
  ~Environment();

  void goalSetup();

  enum Actor getActorID() {return nActorID_;}

  std::string getStringActorID() {return sActorID_;}


  float getPlannerGlobalTime() {return planner_->getGlobalTime();}

  // NOT USED. UNRESOLVED BUG WHERE ODOMETRY IS NOT EXTRACTED PROPERLY
  void setupPlanner();

  // TRACKER FUNCTIONS
  void initAMCL();
  void initTracker();
  void initBumper();
  void initRobotTrackers();

  // void receiveRobotPose(const geometry_msgs::Pose& msg,
  // const ros::MessageEvent<std_msgs::String const>& event);

  // void receiveRobotVel(const geometry_msgs::Twist& msg,
  // const ros::MessageEvent<std_msgs::String const>& event);

  // ** SHAMEFUR DISPRAY!!! **
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

  void updateLocalisation(bool USE_TRACKER);
  //    void updateTracker();

  void updateRobotAgents();

  std::map<int, std::size_t> getTrackerIDs();

  void setTrackOtherAgents(bool trackOtherAgents);

  void setAgentTracker(int TrackerID, std::size_t AgentID);

  // PLANNER FUNCTIONS
  void setPlannerPosition(Vector2 plannerPos)
  {planner_->setAgentPosition(THIS_ROBOT, plannerPos);}

  void disablePlannerAgent()  {planner_->setAgentType(THIS_ROBOT, INACTIVE);}

  void setPlannerParam();

  std::size_t addVirtualAgent(std::string id, const Vector2 startPos,
                              std::size_t goalNo);

  std::size_t addPedestrianAgent(std::string id, const Vector2 startPos,
                                 std::size_t goalNo);

  std::size_t getAgentType(std::size_t agentNo) {
    return planner_->getAgentType(agentNo);
  }

  // GOAL FUNCTIONS
  std::size_t addPlannerGoal(const Vector2 goalPosition) {
    return planner_->addGoal(goalPosition);
  }

  std::size_t getPlannerGoal() {return planner_->getAgentGoal(THIS_ROBOT);}

  void setPlannerGoal(std::size_t goalNo) {
    planner_->setAgentGoal(THIS_ROBOT, goalNo);
  }

  void editPlannerGoal(std::size_t goalNo, Vector2 goalPosition) {
    planner_->editGoal(goalNo, goalPosition);
  }

  void setPlannerInitialGoal(std::size_t initialGoalNo) {
    initialGoalNo_ = initialGoalNo;
  }

  std::size_t getPlannerInitalGoal() { return initialGoalNo_; }

  void loadPlannerInitialGoal();

  std::size_t addAndSetPlannerGoal(const Vector2 goalPosition);

  std::size_t getNumPlannerGoals()  { return planner_->getNumGoals(); }

  Vector2 getPlannerGoalPosition(std::size_t goalNo) const {
    return planner_->getGoalPosition(goalNo);
  }

  bool getReachedPlannerGoal() const {
    return planner_->agents_[THIS_ROBOT]->reachedGoal_;
  }

  bool getVirtualAgentReachedGoal(std::size_t simID, std::size_t agentNo);

  void setPlannerGoalPlan(size_t goalPlan)  {goalPlan_ = goalPlan;}

  void setNextGoal();

  void cycleGoalsClockwise();

  void cycleGoalsCounterClockwise();

  void cycleGoals1_2();

  void cycleGoals2_3();

  void cycleGoals3_1();

  // SIMULATOR FUNCTIONS
  void setSimParam(std::size_t simID);

  void doPlannerStep();

  void doSimulatorStep(std::size_t simID);

  std::size_t addAndSetSimGoal(std::size_t simID, std::size_t agentNo,
                               const Vector2 goalPosition);

  std::map<std::size_t, std::size_t> setupModel(std::size_t agentNo,
      std::map<size_t, Vector2> possGoals);

  std::size_t inferGoals(std::size_t agentNo,
                         std::map<std::size_t, std::size_t> simIDs);

  std::size_t getNumPlannerAgents() { return planner_->agents_.size(); }

  Vector2 getPlannerAgentPosition(std::size_t agentNo) {
    return planner_->getAgentPosition(agentNo);
  }

  Vector2 getPlannerAgentVelocity(std::size_t agentNo) {
    return planner_->getAgentVelocity(agentNo);
  }

  float getPlannerAgentAvgSpeed(std::size_t agentNo) {
    return planner_->getAgentPrefSpeed(agentNo);
  }

  float getPlannerAgentMaxSpeed(std::size_t agentNo) {
    return planner_->getAgentMaxSpeed(agentNo);
  }

  float getPlannerAgentPrefSpeed(std::size_t agentNo) {
    return planner_->getAgentPrefSpeed(agentNo);
  }

  std::size_t addSimulation();

  void deleteSimulation(std::size_t simID);

  std::map<std::size_t, Simulator *>* getSimVectPointer() {
    return simvectPoint_;
  }

  // ROBOT FUNCTIONS

  void resetOdomPosition()  {planner_->resetOdomPosition();}

  void stopYoubot();

  void emergencyStop();

  Environment* getEnvPointer() {return this;}

  bool reachedMoveGoal() {return reachedGoal;}

 private:
  friend class Simulator;
  friend class Agent;
  friend class Goal;

  // Private members
  enum Actor nActorID_;
  std::string sActorID_;
  Vector2 startPos_;
  size_t startGoal_;
  Vector2 currPos_;
  size_t currGoal_;
  Vector2 currVel_;
  std::size_t initialGoalNo_;
  std::size_t goal1_;
  std::size_t goal2_;
  std::size_t goal3_;
  std::size_t goalPlan_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh2_;
  bool newGoal;
  bool reachedGoal;

  Simulator *planner_;
  Planner *newPlanner_;
  Tracker *tracker_;
  AMCLWrapper *amclwrapper_;
  BumperWrapper *bumperwrapper_;

  // Robot Tracking Pub
  ros::Publisher posePub_;
  // ros::Subscriber RobotPoseSub1_;
  // ros::Subscriber RobotPoseSub2_;
  // ros::Subscriber RobotPoseSub3_;
  // ros::Subscriber RobotPoseSub4_;
  // ros::Subscriber RobotPoseSub5_;
  // ros::Subscriber RobotPoseSub6_;

  std::map<Actor, std::size_t>
  trackedRobots_;          // Robot Name, AgentID in planner
  std::map<std::size_t, ros::Subscriber> robotPoseSubs;   // AgentID
  std::map<std::size_t, ros::Subscriber> robotVelSubs;    // AgentID
  std::map<std::size_t, Vector2> robotPoses;              // AgentID
  std::map<std::size_t, Vector2> robotVels;               // AgentID



  std::map<int, std::size_t>
  trackedAgents_;  // First : Tracker ID, Second : SimAgent ID
  std::map<int, std::vector<float> >

  // First : Tracker ID, Second : Cumulative Diff
  // between Odometry and Tracker Position
  trackerCompOdom_;
  std::map<std::size_t, Vector2> possGoals_;
  std::map<std::size_t, std::size_t> simIDs_;
  std::map<std::size_t, Simulator *> simvect_;
  std::map<std::size_t, Simulator *>* simvectPoint_;
  std::map<std::size_t, float>  maxSpeed_;
};

}  // namespace hrvo

#endif /* HRVO_ENVIRONMENT_H_ */
