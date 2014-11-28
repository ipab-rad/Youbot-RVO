/**
* Created by Alejandro Bordallo
* \file   Model.h
* \brief  Declares the Model class.
*/

#ifndef HRVO_MODEL_H_
#define HRVO_MODEL_H_

#include <vector>
#include <map>

#ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
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

    void setupModel(std::size_t agentNo, std::map<std::size_t, Vector2> possGoals);

    std::size_t inferGoals(std::size_t agentNo);

    std::vector<Vector2> getSimVels() {return simVels_;}

    std::map<std::size_t, float> getGoalLikelihoods() {return goalLikelihood_;}

    // float* getgoalRatio() {return goalRatio_;}

    std::vector<float> getGoalRatios() {return goalRatios_;}

    private:
      friend class Simulator;
      friend class Agent;
      friend class Goal;
      friend class Environment;

      // Private Members
      Environment* PlannerPt_;
      std::map<std::size_t, Simulator *>* simvectPoint_;
      std::vector<Vector2> simVels_;

      // float goalRatio_[3];  // TODO: Allocate depending on Goal num
      std::vector<float> goalRatios_;
      std::vector<bool> reachedGoal_;
      std::vector<bool> leftGoal_;


      bool prevPosInit;
      std::vector<Vector2> currSimVels_;
      std::vector<Vector2> pastSimVels_;

      std::map<std::size_t, std::size_t> simIDs_;
      std::map<std::size_t, float> inferredGoalsSum_;
      // std::map<std::size_t, std::map<std::size_t, float> > inferredAgentGoalsSum_;
      // std::map<std::size_t, std::map<std::size_t, float> > inferredGoalHistory_;  // Goal, Count
      std::map<std::size_t, std::vector<float> > inferredGoalsHistory_;
      std::map<std::size_t, std::size_t> inferredGoalCount_;
      std::map<std::size_t, float> goalLikelihood_;
      std::map<std::size_t, float> prevPrior_;

  };

}

#endif /* HRVO_MODEL_H_ */
