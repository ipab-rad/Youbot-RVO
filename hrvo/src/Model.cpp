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
      if (inferredGoalCount_[agentNo].find(i) == inferredGoalCount_[agentNo].end())
      {inferredGoalCount_[agentNo][i] = 0;}
      // INFO("simID=" << simIDs_[i] << " ");
      // INFO(simNumGoals=" << simnumGoals << std::endl);
      // INFO("Assigned GoalPos" << i << "of" << numGoals << ": " << simvect_[simIDs_[i]]->getGoalPosition(goalID) << std::endl);
    }
  }

  std::size_t Model::inferGoals(std::size_t agentNo)
  {
    if (!simVels_.empty())
      {simVels_.clear();} // Reset Simulated Velocities

    const Vector2 currVel = PlannerPt_->getPlannerAgentVelocity(agentNo);
    std::map<std::size_t, float> inferredGoals;

    for (std::size_t j = 0; j < simIDs_.size(); ++j)
    {
      PlannerPt_->doSimulatorStep(simIDs_[j]);
      simVels_.push_back((*simvectPoint_)[simIDs_[j]]->getAgentVelocity(agentNo));
      if (DISPLAY_INFERENCE_VALUES) {INFO("currVel=[" << currVel << "] " << "simVel=[" << simVels_.back() << "]" << std::endl);}
      inferredGoals[j] = sqrdiff(currVel, simVels_.back());
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
            inferredAgentGoalsSum_[agentNo][l] = GOAL_SUM_PRIOR;
        }
      }
    }
    if (!stopAtGoal)
      {INFO("Agent" << agentNo << " is travelling..."<<std::endl);}

    if (inferredAgentGoalsSum_[agentNo].empty())
    {
        for (std::size_t l = 0; l < inferredGoals.size(); ++l)
        {
            inferredAgentGoalsSum_[agentNo][l] = GOAL_SUM_PRIOR;
        }
    }

    float inferredGoalsTotal(0.0f);
    for (std::size_t j = 0; j < inferredGoals.size(); ++j)
    {
        if (DISPLAY_INFERENCE_VALUES) {INFO("Goal" << j << "=" << inferredGoals[j] << " ");}
        
        inferredGoalHistory_[agentNo][j][inferredGoalCount_[agentNo][j]] = inferredGoals[j];

        // inferredGoalHistory_[agentNo][j][0] = inferredGoals[j];

        inferredGoalCount_[agentNo][j] += 1;
        if (inferredGoalCount_[agentNo][j] == GOAL_INFERENCE_HISTORY) 
          {inferredGoalCount_[agentNo][j] = 0;}

        inferredAgentGoalsSum_[agentNo][j] = GOAL_SUM_PRIOR;
        for (int i = 0; i < inferredGoalHistory_[agentNo][j].size(); ++i)
        {
          inferredAgentGoalsSum_[agentNo][j] += inferredGoalHistory_[agentNo][j][i];
        }
        
        // inferredAgentGoalsSum_[agentNo][j] += inferredGoals[j]; // TODO: Add moving average
        inferredGoalsTotal += 1 / inferredAgentGoalsSum_[agentNo][j];
        if (DISPLAY_INFERENCE_VALUES) 
          {INFO("GoalSum" << j << "=" << inferredAgentGoalsSum_[agentNo][j] << " " << std::endl);}
    }
    INFO(std::endl);

    INFO("Goal ratio=");
    float goalRatio[inferredGoals.size()];
    float maxLikelihoodRatio = 0.0f;
    std::size_t maxLikelihoodGoal = 0;
    for (std::size_t k = 0; k < inferredGoals.size(); ++k)
    {
    goalRatio[k] = ((1 / inferredAgentGoalsSum_[agentNo][k]) / inferredGoalsTotal);
    if (k != 0) {INFO(":"); }
    INFO(goalRatio[k]);
    goalRatio_[k] = goalRatio[k];
    if (goalRatio[k] > maxLikelihoodRatio) {maxLikelihoodRatio = goalRatio[k]; maxLikelihoodGoal = k;}
    }
    INFO(std::endl);

    for (std::size_t j = 0; j < simIDs_.size(); ++j)
      { PlannerPt_->deleteSimulation(simIDs_[j]); }

    INFO("Agent" << agentNo << " is likely going to Goal" << maxLikelihoodGoal << std::endl);
    return maxLikelihoodGoal;
  }

}