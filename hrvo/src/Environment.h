/**
* Created by Alejandro Bordallo
* \file   Environment.h
* \brief  Declares the Environment class.
*/
#ifndef HRVO_ENVIRONMENT_H_
#define HRVO_ENVIRONMENT_H_


#ifndef HRVO_SIMULATOR_H_
#include "Simulator.h"
#endif

#ifndef HRVO_AGENT_H_
#include "Agent.h"
#endif

#include "PTrackingBridge/TargetEstimations.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"
#include "ros/ros.h"

#ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
#endif

namespace hrvo {
  class Simulator;
  class Agent;
  class Goal;

  class Environment
  {
  public:
    Environment(enum Actor actorID, const Vector2 startPos);
    ~Environment();

    void goalSetup();

    enum Actor getActorID() {return nActorID_;}

    std::string getStringActorID() {return sActorID_;}

    float getPlannerGlobalTime() {return planner_->getGlobalTime();}

    void setupPlanner();  // NOT USED. UNRESOLVED BUG WHERE ODOMETRY IS NOT EXTRACTED PROPERLY

    void updateTracker();

    void setTrackOtherAgents(bool trackOtherAgents) {trackOtherAgents_ = trackOtherAgents;}

    void setAgentTracker(int TrackerID, std::size_t AgentID)  {trackedAgents_[TrackerID] = AgentID;
      if (AgentID == THIS_ROBOT) {robotTrackerID_ = TrackerID;}}

    std::map<int, std::size_t> getTrackerIDs();

    void setPlannerPosition(Vector2 plannerPos) {planner_->setAgentPosition(THIS_ROBOT, plannerPos);}

    void disablePlannerAgent()  {planner_->setAgentType(THIS_ROBOT, INACTIVE);}

    void receiveTrackerData(const PTrackingBridge::TargetEstimations::ConstPtr& msg);
    
    void setPlannerParam();

    std::size_t addVirtualAgent(std::string id, const Vector2 startPos, std::size_t goalNo);

    std::size_t addPedestrianAgent(std::string id, const Vector2 startPos, std::size_t goalNo);

    std::size_t addPlannerGoal(const Vector2 goalPosition);

    std::size_t getAgentType(std::size_t agentNo) {return planner_->getAgentType(agentNo);}

    std::size_t getPlannerGoal() {return planner_->getAgentGoal(THIS_ROBOT);}

    void setPlannerGoal(std::size_t goalNo);

    void setPlannerInitialGoal(int goalIndex);

    std::size_t addAndSetPlannerGoal(const Vector2 goalPosition);

    std::size_t setSimParam(std::size_t simID);

    std::size_t getNumPlannerGoals()  { return planner_->getNumGoals(); }

    Vector2 getGoalPlannerPosition(std::size_t goalNo) const { return planner_->getGoalPosition(goalNo); }

    bool getReachedPlannerGoal() const { return planner_->agents_[THIS_ROBOT]->reachedGoal_;}

    bool getVirtualAgentReachedGoal(std::size_t simID, std::size_t agentNo);

    void cycleGoalsClockwise();

    void cycleGoalsCounterClockwise();

    void doPlannerStep();

    void doSimulatorStep(std::size_t simID);

    std::size_t addAndSetSimGoal(std::size_t simID, std::size_t agentNo, const Vector2 goalPosition);

    std::map<std::size_t, std::size_t> setupModel(std::size_t agentNo, std::map<size_t, Vector2> possGoals);

    std::size_t inferGoals(std::size_t agentNo, std::map<std::size_t, std::size_t> simIDs);

    std::size_t getNumPlannerAgents() { return planner_->agents_.size(); }

    Vector2 getPlannerAgentPosition(std::size_t agentNo) { return planner_->getAgentPosition(agentNo); }

    Vector2 getPlannerAgentVelocity(std::size_t agentNo) { return planner_->getAgentVelocity(agentNo); }
    
    float getPlannerAgentAvgSpeed(std::size_t agentNo) { return planner_->getAgentPrefSpeed(agentNo); }
    
    float getPlannerAgentMaxSpeed(std::size_t agentNo) { return planner_->getAgentMaxSpeed(agentNo); }

    std::size_t addSimulation();

    void deleteSimulation(std::size_t simID);

    std::map<std::size_t, Simulator *>* getSimVectPointer() {return simvectPoint_;}

    void resetOdomPosition()  {planner_->resetOdomPosition();}

    void stopYoubot();

    void emergencyStop();

    float calculateAvgMaxSpeeds(int AgentID, Vector2 AgentVel);


    private:
      friend class Simulator;
      friend class Agent;
      friend class Goal;

      enum Actor nActorID_;
      std::string sActorID_; 
      Vector2 startPos_;
      size_t startGoal_;
      Vector2 currPos_;
      size_t currGoal_;
      Vector2 currVel_;
      std::size_t goal1_;
      std::size_t goal2_;
      std::size_t goal3_;
      
      std::map<size_t, Vector2> prevPos;
      bool trackOtherAgents_;

      ros::NodeHandle nh_;
      ros::Subscriber Targsub;
      PTrackingBridge::TargetEstimations msg_;

      Simulator *planner_;

      int robotTrackerID_;
      int trackerComparisonCounter_;
      std::map<int, std::size_t> trackedAgents_;  // First : Tracker ID, Second : SimAgent ID
      std::map<int, float> trackerCompOdom_;      // First : Tracker ID, Second : Cumulative Diff between Odometry and Tracker Position
      std::map<std::size_t, Vector2> possGoals_;
      std::map<std::size_t, std::size_t> simIDs_;
      std::map<std::size_t, Simulator *> simvect_;
      std::map<std::size_t, Simulator *>* simvectPoint_;
      std::map<std::size_t, std::map<std::size_t, float> > agentVelHistory_;  // SimAgentID : VelCount : Velocity Magnitude
      std::map<std::size_t, std::size_t>  agentVelCount_;
      std::map<std::size_t, float>  maxSpeed_;

  };

}

#endif /* HRVO_ENVIRONMENT_H_ */