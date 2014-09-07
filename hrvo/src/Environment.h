/**
* Created by Alejandro Bordallo
* \file   Environment.h
* \brief  Declares the Environment class.
*/

#include <vector>
#include <map>

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

namespace hrvo {
  class Simulator;
  class Agent;
  class Goal;
  class KdTree;

  class Environment
  {
  public:
    Environment(enum Actor actorID, const Vector2 startPos);
    ~Environment();

    enum Actor getActorID() {return nActorID_;}

    std::string getStringActorID() {return sActorID_;}

    float getPlannerGlobalTime() {return planner_->getGlobalTime();}

    void updateTracker();

    void setTrackOtherAgents(bool trackOtherAgents) {trackOtherAgents_ = trackOtherAgents;}

    void setAgentTracker(int TrackerID, std::size_t AgentID)  {trackedAgents_[TrackerID] = AgentID;}

    std::map<int, std::size_t> getTrackerIDs();

    void receiveTrackerData(const PTrackingBridge::TargetEstimations::ConstPtr& msg);
    
    void setPlannerParam();

    std::size_t addVirtualAgent(std::string id, const Vector2 startPos, std::size_t goalNo);

    std::size_t addPedestrianAgent(std::string id, const Vector2 startPos, std::size_t goalNo);

    std::size_t addPlannerGoal(const Vector2 goalPosition);

    std::size_t getPlannerGoal(std::size_t agentNo) {return planner_->getAgentGoal(agentNo);}

    int setPlannerGoal(std::size_t goalNo);

    std::size_t addAndSetPlannerGoal(const Vector2 goalPosition);

    std::size_t setSimParam(std::size_t simID);

    float getGlobalPlannerTime() const { return planner_->getGlobalTime(); }

    std::size_t getNumPlannerGoals()  { return planner_->getNumGoals(); }

    Vector2 getGoalPlannerPosition(std::size_t goalNo) const { return planner_->getGoalPosition(goalNo); }

    bool getReachedPlannerGoal() const { return planner_->agents_[THIS_ROBOT]->reachedGoal_;}

    bool getVirtualAgentReachedGoal(std::size_t simID, std::size_t agentNo);
    // { simvect_[simID]->agents_[agentNo].reachedGoal_;  }

    void doPlannerStep();

    void doSimulatorStep(std::size_t simID);

    std::size_t addAndSetSimGoal(std::size_t simID, std::size_t agentNo, const Vector2 goalPosition);

    std::map<std::size_t, std::size_t> setupModel(std::size_t agentNo, std::map<size_t, Vector2> possGoals);

    std::size_t inferGoals(std::size_t agentNo, std::map<std::size_t, std::size_t> simIDs);

    void setupPlannerModel(std::size_t agentNo);

    std::map<std::size_t, float> inferAllGoals(std::size_t agentNo);

    float getGoalRatio (std::size_t goalNo) {return goalRatio_[goalNo];}

    // bool plannerReachedGoals();

    std::size_t getNumPlannerAgents() const { return planner_->agents_.size(); }

    Vector2 getPlannerAgentPosition(std::size_t agentNo) const { return planner_->getAgentPosition(agentNo); }

    Vector2 getPlannerAgentVelocity(std::size_t agentNo) const { return planner_->getAgentVelocity(agentNo); }
    
    std::size_t addSimulation();

    void deleteSimulation(std::size_t simID);

    void stopYoubot();

  /**
  * \brief    Sends to all agents an emergency stop command.
  */
    void emergencyStop();

    std::pair<float, float> calculateAvgMaxSpeeds(int AgentID, Vector2 AgentVel);


    private:
      friend class Simulator;
      friend class Agent;
      friend class Goal;
      friend class KdTree;

      enum Actor nActorID_;
      std::string sActorID_; 
      Vector2 startPos_;
      size_t startGoal_;
      float goalRatio_[3];
      
      bool prevPosInit;
      bool trackOtherAgents_;

      ros::NodeHandle nh_;
      ros::Subscriber Targsub;
      PTrackingBridge::TargetEstimations msg_;

      Simulator *planner_;

      std::map<int, std::size_t> trackedAgents_;  // First : Tracker ID, Second : SimAgent ID
      std::map<std::size_t, Vector2> possGoals_;
      std::map<std::size_t, std::size_t> simIDs_;
      std::map<std::size_t, Simulator *> simvect_;
      std::map<std::size_t, std::map<std::size_t, float> > agentVelHistory_;  // SimAgentID : VelCount : Velocity Magnitude
      std::map<std::size_t, std::size_t>  agentVelCount_;
      std::map<std::size_t, float> inferredGoalsSum_;
      std::map<std::size_t, std::map<std::size_t, float> > inferredAgentGoalsSum_;
      std::map<std::size_t, std::map<std::size_t, std::map<std::size_t, float> > > inferredGoalHistory_;
      std::map<std::size_t, std::map<std::size_t, std::size_t> > inferredGoalCount_;


  };

}
