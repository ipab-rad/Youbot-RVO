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

    std::size_t addTracker();
    
    void setPlannerParam();

    std::size_t addVirtualAgent(std::string id, const Vector2 startPos, std::size_t goalNo);

    std::size_t addPlannerGoal(const Vector2 goalPosition);

    int setPlannerGoal(std::size_t goalNo);

    std::size_t setSimParam(std::size_t simID);

    std::size_t addSimulation();

    int deleteSimulation(std::size_t simID);

  /**
  * \brief    Sends to all agents an emergency stop command.
  */
    void emergencyStop();


    private:
      friend class Simulator;
      friend class Agent;
      friend class Goal;
      friend class KdTree;

      enum Actor nactorID_;
      Vector2 startPos_;
      size_t startGoal_;

      Simulator *planner_;
      std::map<std::size_t, Simulator *> simvect_;




  };

}
