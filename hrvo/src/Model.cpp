/* 
* @Author: GreatAlexander
* @Date:   2014-09-10 15:23:09
* @Last Modified by:   GreatAlexander
* @Last Modified time: 2014-09-10 15:33:34
*/

#ifndef HRVO_MODEL_H_
#include "Model.h"
#endif


namespace hrvo {

  Model::Model(Environment* PlannerPt)
  { 
    PlannerPt_ = PlannerPt;
  }

  Model::~Model()
  {
    ;
  }

  std::map<std::size_t, std::size_t> Model::setupModel(std::size_t agentNo, std::map<std::size_t, Vector2> possGoals)
  {    
    DEBUG("Accesed" << std::endl)
    std::map<std::size_t, std::size_t> simIDs;
    DEBUG("Accesed" << std::endl)
    std::size_t numGoals = possGoals.size();
    DEBUG("Accesed" << std::endl)

    for (std::size_t i = 0; i < numGoals; ++i) 
    {
      simIDs[i] = PlannerPt_->addSimulation();
      DEBUG("Accesed" << std::endl)
      std::size_t goalID = simvect_[simIDs[i]]->addGoal(possGoals[i]);  // TODO: simvect_ Copy map or pointer to map
      DEBUG("Accesed" << std::endl)
      simvect_[simIDs[i]]->setAgentGoal(agentNo, goalID);
      DEBUG("Accesed" << std::endl)
      if (inferredGoalCount_[agentNo].find(i) == inferredGoalCount_[agentNo].end())
        DEBUG("Accesed" << std::endl)
      {inferredGoalCount_[agentNo][i] = 0;}
      DEBUG("Accesed" << std::endl)
      // INFO("simID=" << simIDs[i] << " ");
      // INFO(simNumGoals=" << simnumGoals << std::endl);
      // INFO("Assigned GoalPos" << i << "of" << numGoals << ": " << simvect_[simIDs[i]]->getGoalPosition(goalID) << std::endl);
    }
    return simIDs;
  }

  std::size_t Model::inferGoals(std::size_t agentNo, std::map<std::size_t, std::size_t> simIDs)
  {

    const Vector2 currVel = PlannerPt_->getPlannerAgentVelocity(agentNo);
    std::map<std::size_t, float> inferredGoals;

    for (std::size_t j = 0; j < simIDs.size(); ++j)
    {
      PlannerPt_->doSimulatorStep(simIDs[j]);
      Vector2 simVel = simvect_[simIDs[j]]->getAgentVelocity(agentNo);
      INFO("currVel=[" << currVel << "] " << "simVel=[" << simVel << "]" << std::endl);
      inferredGoals[j] = sqrdiff(currVel, simVel);
    }

    bool stopAtGoal = false;
    for (std::size_t j = 0; j < simIDs.size(); ++j)
    {
      if (simvect_[simIDs[j]]->getAgentReachedGoal(agentNo))
      {
        stopAtGoal = true;
        INFO("Agent reached Goal"<< j <<std::endl);
        for (std::size_t l = 0; l < inferredGoals.size(); ++l)
        {
            inferredAgentGoalsSum_[agentNo][l] = GOAL_SUM_PRIOR;
        }
      }
    }
    if (!stopAtGoal)
      {INFO("Agent is travelling..."<<std::endl);}

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
        INFO("Goal" << j << "=" << inferredGoals[j] << " ");
        
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
        INFO("GoalSum" << j << "=" << inferredAgentGoalsSum_[agentNo][j] << " " << std::endl);
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

    for (std::size_t j = 0; j < simIDs.size(); ++j)
      { PlannerPt_->deleteSimulation(simIDs[j]); }

    return maxLikelihoodGoal;
  }

}