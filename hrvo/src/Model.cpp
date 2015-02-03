/* 
* @Author: GreatAlexander
* @Date:   2014-09-10 15:23:09
* @Last Modified by:   GreatAlexander
* @Last Modified time: 2014-09-10 15:33:34
*/

#include "Model.h"

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

    if (reachedGoal_.empty() || leftGoal_.empty())
    {
      for (std::size_t l = 0; l < possGoals.size(); ++l)
      {
      reachedGoal_.push_back(false);
      leftGoal_.push_back(false);
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

  Vector2 currVel = PlannerPt_->getPlannerAgentVelocity(agentNo);
  // const Vector2 currVel = PlannerPt_->getPlannerAgentPosition(agentNo);
  std::map<std::size_t, float> inferredGoals;

  // If agents have not moved at all yet, initialise simVels_ to avoid NaNs
  if (currVel == STOP && PlannerPt_->getPlannerAgentPrefSpeed(agentNo) == 0.0f)
  {
    Vector2 VelInit = Vector2(0.001f, 0.001f);
    for (std::size_t j = 0; j < simIDs_.size(); ++j)
    {
      simVels_.push_back(VelInit);
    }
    currVel = VelInit;
  }
  else
  {
    for (std::size_t j = 0; j < simIDs_.size(); ++j)
    {
      PlannerPt_->doSimulatorStep(simIDs_[j]);
      simVels_.push_back((*simvectPoint_)[simIDs_[j]]->getAgentVelocity(agentNo));
      // simVels_.push_back((*simvectPoint_)[simIDs_[j]]->getAgentPosition(agentNo));
    }
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
    if (USE_PROB_MODEL)
    {
      if (!BIVARIATE)
      {
        float var = sqrtf(0.25);
        float var2 = (MAX_PEOPLE_ACCELERATION / 2) / ROS_FREQ;  // Max accel, divided equally between x,y comp, for 1/10Sec
        float xval = currVel.getX();
        float xmean = currSimVels_[j].getX();
        float yval = currVel.getY();
        float ymean = currSimVels_[j].getY();
        float probx = (1 / sqrtf(2 * HRVO_PI * var2)) * exp(-( pow((xval - xmean), 2) / (2 * var2) ) );
        float proby = (1 / sqrtf(2 * HRVO_PI * var2)) * exp(-( pow((yval - ymean), 2) / (2 * var2) ) );
        // inferredGoals[j] = probx * proby;
        inferredGoals[j] = ((probx + proby) / 2);
        // * inferredGoalsHistory_[j].back()
      }
      else
      {
        // BIVARIATE
        float x = currVel.getX();
        float y = currVel.getY();
        float ux = currSimVels_[j].getX();
        float uy = currSimVels_[j].getY();
        float v2x = (MAX_PEOPLE_ACCELERATION / 2) / ROS_FREQ;
        float v2y = (MAX_PEOPLE_ACCELERATION / 2) / ROS_FREQ;
        float vx = sqrtf(v2x);
        float vy = sqrtf(v2y);
        float corr = 0.0f;
        float corr2 = pow(corr,2.0f);
        float t1 = pow(x-ux, 2.0f)/v2x;
        float t2 = pow(y-uy, 2.0f)/v2y;
        float t3 = (2*corr*(x-ux)*(y-uy))/(vx*vy);
        float p = (1 / (2 * HRVO_PI * vx * vy * sqrtf(1-corr2)));
        float e = exp(-(1/(2*(1-corr2))) * (t1 + t2 - t3));
        inferredGoals[j] = p * e;
      }

    }
    else
    {
      inferredGoals[j] = sqrdiff(currVel, currSimVels_[j]);
    }
  }

  // NORMALISING SIM PRIORS
  if (USE_PROB_MODEL)
  {
    float likelihoodSum = 0.0f;
    for (std::size_t j = 0; j < inferredGoals.size(); ++j)
    {
      likelihoodSum += inferredGoals[j];
    }
  }

  bool travelling = true;
  bool resetPriors = false;
  for (std::size_t j = 0; j < simIDs_.size(); ++j)
  {
    bool stopAtGoal = (*simvectPoint_)[simIDs_[j]]->getAgentReachedGoal(agentNo);
    if (stopAtGoal)
    { 
      travelling = false;
      reachedGoal_[j] = true;
      if (DISPLAY_INFERENCE_VALUES){
      INFO("Agent" << agentNo << " reached Goal"<< j <<std::endl);}
      // for (std::size_t l = 0; l < inferredGoals.size(); ++l)
      // {
      //   // Reset goal belief when goal is reached, to be replaced with moving average
      //   inferredGoalsSum_[l] = GOAL_SUM_PRIOR;
      // }
    }
    else if (!stopAtGoal && reachedGoal_[j] == false)
    {
      leftGoal_[j] = false;
    }
    else if (!stopAtGoal && reachedGoal_[j] == true)
    {
      if (DISPLAY_INFERENCE_VALUES){
      INFO("Agent" << agentNo << " left Goal"<< j <<std::endl);}
      reachedGoal_[j] = false;
      leftGoal_[j] = true;
      resetPriors = true;
    }
  }

  if (travelling)
  { if (DISPLAY_INFERENCE_VALUES){
    INFO("Agent" << agentNo << " is travelling..."<<std::endl);}
  }

  float inferredGoalsTotal(0.0f);
  for (std::size_t j = 0; j < inferredGoals.size(); ++j)
  {
    if (DISPLAY_INFERENCE_VALUES) {INFO("Goal" << j << "=" << inferredGoals[j] << std::endl);}

    // inferredGoalHistory_[j][inferredGoalCount_[j]] = inferredGoals[j];
    inferredGoalsHistory_[j].insert(inferredGoalsHistory_[j].begin(), inferredGoals[j]);
    inferredGoalsHistory_[j].resize(GOAL_INFERENCE_HISTORY);

    // inferredGoalHistory_[j][0] = inferredGoals[j];

    // inferredGoalCount_[j] += 1;
    // if (inferredGoalCount_[j] == GOAL_INFERENCE_HISTORY) 
    //   {inferredGoalCount_[j] = 0;}

    
    if (USE_PROB_MODEL)
    {
      // inferredGoalsSum_[j] = inferredGoals[j];
      goalLikelihood_[j] = inferredGoalsHistory_[j].front();
      float uniformPrior = 1.0f / inferredGoals.size();
      float norm = 1.0f / inferredGoals.size();
      if (inferredGoalsHistory_[j].size() == 1)
      {
        prevPrior_[j] = uniformPrior;
      }
      else if (inferredGoalsHistory_[j].size() > 1)
      {
        prevPrior_[j] = inferredGoalsHistory_[j][1];
      }
      if (!resetPriors)
      {
        goalLikelihood_[j] = goalLikelihood_[j] * prevPrior_[j];
        // goalLikelihood_[j] = goalLikelihood_[j] * (uniformPrior + (PRIOR_LAMBDA * prevPrior_[j]));
      }
      else
      {
        goalLikelihood_[j] = uniformPrior;
      }

      // for (int i = 1; i < inferredGoalsHistory_[j].size(); ++i)
      // {
      //   DEBUG(inferredGoalsHistory_[j][i] << " ");
      //   // inferredGoalsSum_[j] = inferredGoalsSum_[j] * inferredGoalsHistory_[j][i];
      //   // inferredGoalsSum_[j] += (inferredGoalsHistory_[j][i] * uniformPrior) / norm ;  // Not Discounted Summation
      //   // goalLikelihood_[j] = inferredGoalsSum_[j] * inferredGoalsHistory_[j][i];
      //   // inferredGoalsSum_[j] += (inferredGoalsHistory_[j][i] * uniformPrior) / norm ;
      // }
      // DEBUG(std::endl);
      // goalLikelihood_[j] = inferredGoalsSum_[j] / inferredGoalsHistory_[j].size();
      inferredGoalsTotal += goalLikelihood_[j];
      if (DISPLAY_INFERENCE_VALUES) 
        {INFO("GoalLikelihood" << j << "=" << goalLikelihood_[j] << " " << std::endl);}
    }
    else
    {
      inferredGoalsSum_[j] = GOAL_SUM_PRIOR;
      for (int i = 0; i < inferredGoalsHistory_[j].size(); ++i)
      {
        DEBUG(inferredGoalsHistory_[j][i] << " ");
        // inferredGoalsSum_[j] += inferredGoalsHistory_[j][i];
        // Discounted sum over history. Sum:(1 - disc)^t-1 * Value  where disc(0 < disc =< 1)
        
        // Normal Discount
        // inferredGoalsSum_[j] += pow((1 - GOAL_HISTORY_DISCOUNT), ((i + 1) - 1)) * inferredGoalsHistory_[j][i];

        // Inverted Discount
        inferredGoalsSum_[j] += pow((1 - GOAL_HISTORY_DISCOUNT), ((GOAL_INFERENCE_HISTORY - i) - 1)) * inferredGoalsHistory_[j][i];
      }
      DEBUG(std::endl);
      inferredGoalsTotal += 1 / inferredGoalsSum_[j];
      goalLikelihood_[j] = inferredGoalsSum_[j];
      if (DISPLAY_INFERENCE_VALUES) 
        {INFO("GoalSum" << j << "=" << inferredGoalsSum_[j] << " " << std::endl);}
    }
  }
  if (DISPLAY_INFERENCE_VALUES) {INFO(std::endl);}
  // TODO: Finish Probabilistic summation / prior multiplication

  if (!goalRatios_.empty())
    {goalRatios_.clear();} // Reset Goal Ratios

  float maxLikelihoodRatio = 0.0f;
  std::size_t maxLikelihoodGoal = 0;
  if (DISPLAY_INFERENCE_VALUES) {INFO("Goal ratio=");}
  float goalRatio = 0.0f;

  for (std::size_t k = 0; k < inferredGoals.size(); ++k)
  {
    if (inferredGoalsTotal == 0)
    {
      goalRatio = 1.0f / inferredGoals.size();  // Avoiding NaNs when likelihoods are all 0
    }
    else  
    {
      if (USE_PROB_MODEL)
      {
        goalRatio = goalLikelihood_[k] / inferredGoalsTotal;
        // goalRatio = goalLikelihood_[k];
        inferredGoalsHistory_[k][0] = goalRatio;
      }
      else
      {
        goalRatio = ((1 / inferredGoalsSum_[k]) / inferredGoalsTotal);
      }
    }
    if (DISPLAY_INFERENCE_VALUES) 
    {
      if (k != 0) {INFO(":"); }
      INFO(goalRatio);
    }
    goalRatios_.push_back(goalRatio);
    if (goalRatio > maxLikelihoodRatio) {maxLikelihoodRatio = goalRatio; maxLikelihoodGoal = k;}
  }

  for (std::size_t j = 0; j < simIDs_.size(); ++j)
    { PlannerPt_->deleteSimulation(simIDs_[j]); }
  if (DISPLAY_INFERENCE_VALUES) {
    INFO(std::endl);
    INFO("Agent" << agentNo << " is likely going to Goal" << maxLikelihoodGoal << std::endl);
    INFO(std::endl);
  }
  return maxLikelihoodGoal;
}

}