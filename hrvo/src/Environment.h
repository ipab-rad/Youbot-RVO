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

    void compsActorID();

    enum Actor getActorID() {return nActorID_;}

    std::string getsActorID() {return sActorID_;}

    std::size_t addTracker();
    
    void setPlannerParam();

    std::size_t addVirtualAgent(std::string id, const Vector2 startPos, std::size_t goalNo);

    std::size_t addPlannerGoal(const Vector2 goalPosition);

    int setPlannerGoal(std::size_t goalNo);

    std::size_t addAndSetPlannerGoal(const Vector2 goalPosition);

    std::size_t setSimParam(std::size_t simID);

    float getPlannerGlobalTime() const { return planner_->getGlobalTime(); }

    bool getPlannerReachedGoal() const { return planner_->agents_[THIS_ROBOT]->reachedGoal_;}

    bool getVirtualAgentReachedGoal(std::size_t simID, std::size_t agentNo);
    // { simvect_[simID]->agents_[agentNo].reachedGoal_;  }

    void doPlannerStep();

    void doSimulatorStep(std::size_t simID);

    // bool plannerReachedGoals();

    std::size_t getNumAgents() const { return planner_->agents_.size(); }

    Vector2 getAgentPosition(std::size_t agentNo) const { return planner_->agents_[agentNo]->position_; }

    std::size_t addSimulation();

    int deleteSimulation(std::size_t simID);

    void stopYoubot();

  /**
  * \brief    Sends to all agents an emergency stop command.
  */
    void emergencyStop();


    private:
      friend class Simulator;
      friend class Agent;
      friend class Goal;
      friend class KdTree;

      enum Actor nActorID_;
      std::string sActorID_; 
      Vector2 startPos_;
      size_t startGoal_;

      Simulator *planner_;

      std::map<std::size_t, Simulator *> simvect_;




  };

}
