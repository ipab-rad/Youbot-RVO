/* 
* @Author: GreatAlexander
* @Date:   2014-09-10 15:23:09
* @Last Modified by:   GreatAlexander
* @Last Modified time: 2014-09-10 15:33:34
*/

#ifndef HRVO_MODEL_H_
#include "Model.h"
#endif

// #ifndef HRVO_SIMULATOR_H_
// #include "Simulator.h"
// #endif

// #ifndef HRVO_AGENT_H_
// #include "Agent.h"
// #endif

#ifndef HRVO_ENVIRONMENT_H_
#include "Environment.h"
#endif

// #ifndef HRVO_DEFINITIONS_H_
// #include "Definitions.h"
// #endif

namespace hrvo {

  Model::Model(Environment* PlannerPt)
  { 
    PlannerPt_ = PlannerPt;
    simvectPoint_ = PlannerPt_->getSimVectPointer();
    prevPosInit = false;
  }

  Model::~Model()
  {
    ;
  }

  void Model::setupModel(std::size_t agentNo, std::map<std::size_t, Vector2> possGoals)
  {    
    std::size_t numGoals = possGoals.size();

    for (std::size_t i = 0; i < numGoals; ++i) 
    {
      simIDs_[i] = PlannerPt_->addSimulation();
      std::size_t goalID = (*simvectPoint_)[simIDs_[i]]->addGoal(possGoals[i]);
      (*simvectPoint_)[simIDs_[i]]->setAgentGoal(agentNo, goalID);
      if (inferredGoalCount_.find(i) == inferredGoalCount_.end())
      {inferredGoalCount_[i] = 0;}

      if (inferredGoalsSum_.empty())
      {
        for (std::size_t l = 0; l < possGoals.size(); ++l)
        {
            inferredGoalsSum_[l] = GOAL_SUM_PRIOR;
        }
      }
      // INFO("simID=" << simIDs_[i] << " ");
      // INFO(simNumGoals=" << simnumGoals << std::endl);
      // INFO("Assigned GoalPos" << i << "of" << numGoals << ": " << simvect_[simIDs_[i]]->getGoalPosition(goalID) << std::endl);
    }
  }

  std::size_t Model::inferGoals(std::size_t agentNo)
  {
    if (!simVels_.empty())
      {simVels_.clear();} // Reset Simulated Velocities

    if (!goalRatios_.empty())
      {goalRatios_.clear();} // Reset Goal Ratios

    const Vector2 currVel = PlannerPt_->getPlannerAgentVelocity(agentNo);
    // const Vector2 currVel = PlannerPt_->getPlannerAgentPosition(agentNo);
    std::map<std::size_t, float> inferredGoals;

    for (std::size_t j = 0; j < simIDs_.size(); ++j)
    {
      PlannerPt_->doSimulatorStep(simIDs_[j]);
      simVels_.push_back((*simvectPoint_)[simIDs_[j]]->getAgentVelocity(agentNo));
      // simVels_.push_back((*simvectPoint_)[simIDs_[j]]->getAgentPosition(agentNo));
    }

    if (!prevPosInit)
    {
      currSimVels_ = simVels_;
      pastSimVels_ = simVels_;
      prevPosInit = true;
    }
    else
    {
      currSimVels_ = pastSimVels_;
      pastSimVels_ = simVels_;
    }

    for (std::size_t j = 0; j < simIDs_.size(); ++j)
    {
      if (DISPLAY_INFERENCE_VALUES) {INFO("currVel=[" << currVel << "] " << "simVel=[" << currSimVels_[j] << "]" << std::endl);}
      inferredGoals[j] = sqrdiff(currVel, currSimVels_[j]);
    }


    bool stopAtGoal = false;
    for (std::size_t j = 0; j < simIDs_.size(); ++j)
    {
      if ((*simvectPoint_)[simIDs_[j]]->getAgentReachedGoal(agentNo))
      {
        stopAtGoal = true;
        INFO("Agent" << agentNo << " reached Goal"<< j <<std::endl);
        for (std::size_t l = 0; l < inferredGoals.size(); ++l)
        {
          // Reset goal belief when goal is reached, to be removed with moving average
          inferredGoalsSum_[l] = GOAL_SUM_PRIOR;
        }
      }
    }
    if (!stopAtGoal)
      {INFO("Agent" << agentNo << " is travelling..."<<std::endl);}


    float inferredGoalsTotal(0.0f);
    for (std::size_t j = 0; j < inferredGoals.size(); ++j)
    {
      if (DISPLAY_INFERENCE_VALUES) {INFO("Goal" << j << "=" << inferredGoals[j] << " ");}
      
      // inferredGoalHistory_[j][inferredGoalCount_[j]] = inferredGoals[j];
      inferredGoalsHistory_[j].insert(inferredGoalsHistory_[j].begin(), inferredGoals[j]);
      inferredGoalsHistory_[j].resize(GOAL_INFERENCE_HISTORY);  // TODO: Add moving average

      // inferredGoalHistory_[j][0] = inferredGoals[j];

      // inferredGoalCount_[j] += 1;
      // if (inferredGoalCount_[j] == GOAL_INFERENCE_HISTORY) 
      //   {inferredGoalCount_[j] = 0;}

      // inferredGoalsSum_[j] = GOAL_SUM_PRIOR;
      for (int i = 0; i < inferredGoalsHistory_[j].size(); ++i)
      {
        // TODO: Implement discounted sum over past Sum:(1 - disc)^t-1 * Value  where disc(0 < disc =< 1)
        inferredGoalsSum_[j] += inferredGoalsHistory_[j][i];
      }
      
      inferredGoalsTotal += 1 / inferredGoalsSum_[j];
      if (DISPLAY_INFERENCE_VALUES) 
        {INFO("GoalSum" << j << "=" << inferredGoalsSum_[j] << " " << std::endl);}
    }
    INFO(std::endl);

    INFO("Goal ratio=");
    float goalRatio[inferredGoals.size()];
    float maxLikelihoodRatio = 0.0f;
    std::size_t maxLikelihoodGoal = 0;
    for (std::size_t k = 0; k < inferredGoals.size(); ++k)
    {
      goalRatio[k] = ((1 / inferredGoalsSum_[k]) / inferredGoalsTotal);
      if (k != 0) {INFO(":"); }
      INFO(goalRatio[k]);
      goalRatios_.push_back(goalRatio[k]);
      if (goalRatio[k] > maxLikelihoodRatio) {maxLikelihoodRatio = goalRatio[k]; maxLikelihoodGoal = k;}
    }
    INFO(std::endl);

    for (std::size_t j = 0; j < simIDs_.size(); ++j)
      { PlannerPt_->deleteSimulation(simIDs_[j]); }

    INFO("Agent" << agentNo << " is likely going to Goal" << maxLikelihoodGoal << std::endl);
    return maxLikelihoodGoal;
  }

}