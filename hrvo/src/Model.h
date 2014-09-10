/**
* Created by Alejandro Bordallo
* \file   Model.h
* \brief  Declares the Model class.
*/

#include <vector>
#include <map>

#ifndef HRVO_SIMULATOR_H_
#include "Simulator.h"
#endif

#ifndef HRVO_AGENT_H_
#include "Agent.h"
#endif

#ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
#endif

#ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
#endif

#ifndef HRVO_ENVIRONMENT_H_
#include "Environment.h"
#endif

namespace hrvo {
  class Simulator;
  class Agent;
  class Goal;
  class Environment;

  class Model
  {
  public:
    Model(Environment* PlannerPt);
    ~Model();

    std::map<std::size_t, std::size_t> setupModel(std::size_t agentNo, std::map<std::size_t, Vector2> possGoals);

    std::size_t inferGoals(std::size_t agentNo, std::map<std::size_t, std::size_t> simIDs);

    private:
      friend class Simulator;
      friend class Agent;
      friend class Goal;
      friend class Environment;

      Environment* PlannerPt_;
      std::map<std::size_t, Simulator *> simvect_;

      float goalRatio_[3];                              // TODO: Allocate depending on Goal num
      std::map<std::size_t, float> inferredGoalsSum_;
      std::map<std::size_t, std::map<std::size_t, float> > inferredAgentGoalsSum_;
      std::map<std::size_t, std::map<std::size_t, std::map<std::size_t, float> > > inferredGoalHistory_;
      std::map<std::size_t, std::map<std::size_t, std::size_t> > inferredGoalCount_;

  };

}
