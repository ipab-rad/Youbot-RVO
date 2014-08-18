/**
* Created by Alejandro Bordallo
* \file   Environment.cpp
* \brief  Defines the Environment class.
*/

#ifndef HRVO_ENVIRONMENT_H_
#include "Environment.h"
#endif

#ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
#endif

#ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
#endif


namespace hrvo {


  Environment::Environment(enum Actor actorID, const Vector2 startPos)
  { 
    nActorID_ = actorID;
    startPos_ = startPos;
    planner_ = new Simulator(nh_, "planner", nActorID_);
    this->setPlannerParam();
    sActorID_ = getActorName(nActorID_);
    startGoal_ = planner_->addGoal(startPos);
    planner_->addAgent(getActorName(nActorID_), ROBOT, startPos_, startGoal_);
    std::cout << "HRVO Planner for " << sActorID_ << " Constructed" << std::endl;
    Targsub = nh_.subscribe("/agent_1/PTrackingBridge/targetEstimations", 1000, &Environment::addTracker, this);
    ROS_INFO("Suscribing to TargetEstimations");
    prevPosInit = false;
  }
  Environment::~Environment()
  {
    delete planner_;
    planner_ = NULL;

    for (std::map<std::size_t, Simulator *>::iterator iter = simvect_.begin(); iter != simvect_.end(); ++iter) 
    {
      delete iter->second;
    // delete *iter;
    // *iter = NULL;
    }
  }

  void Environment::addTracker(const PTrackingBridge::TargetEstimations::ConstPtr& msg)
  {
    // std::cout << "Received!" << std::endl;
    if (!msg->identities.empty())
    {
      std::size_t size = msg->identities.size();
      for (std::size_t i = 0; i < size; ++i)
      {
        int id = msg->identities[i];
        std::string sid = intToString(id); 
        Vector2 agentPos = Vector2(-1 * msg->positions[i].x, msg->positions[i].y);
        Vector2 agentVel = Vector2(-1 * msg->velocities[i].x, msg->velocities[i].y);

        // ROS_INFO("Agent:%d detected", id);
        if (!prevPosInit)
        {
          prevPos_ = Vector2(-1 * msg->positions[i].x, msg->positions[i].y);
          prevPosInit = true;
        }

        if (trackedAgents_.find(id)==trackedAgents_.end() )
        { 
          trackedAgents_[id] = this->addPedestrianAgent("TrackedPerson" + sid, agentPos, this->addPlannerGoal(agentPos));
          std::cout<< "New agent" << trackedAgents_[id] << " with tracker" << sid << std::endl;
          planner_->setAgentMaxAccel(trackedAgents_[id], 2.0f);
          planner_->setAgentMaxSpeed(trackedAgents_[id], 2.0f);
          planner_->setAgentPrefSpeed(trackedAgents_[id], 1.0f);
          }
        else
        {
          planner_->setAgentPosition(trackedAgents_[id], prevPos_);
          planner_->setAgentVelocity(trackedAgents_[id], agentVel);
          // planner_->setAgentVelocity(trackedAgents_[id], STOP);
          prevPos_ = agentPos;
        }

      }

    }

  }

  void Environment::setPlannerParam()
  {
    planner_->setTimeStep(SIM_TIME_STEP);
    planner_->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_SPEED, MAX_SPEED, 0.0f, 0.6f, STOP, 0.0f); 
  }

  std::size_t Environment::addVirtualAgent(std::string id, const Vector2 startPos, std::size_t goalNo)
  {
    return planner_->addAgent(sActorID_ + "_v" + id, SIMAGENT, startPos, goalNo);
  }

  std::size_t Environment::addPedestrianAgent(std::string id, const Vector2 startPos, std::size_t goalNo)
  {
    return planner_->addAgent(sActorID_ + "_p" + id, PERSON, startPos, goalNo);
  }


  std::size_t Environment::addPlannerGoal(const Vector2 goalPosition)
  {
    return planner_->addGoal(goalPosition);
  }

  int Environment::setPlannerGoal(std::size_t goalNo)
  {
    planner_->setAgentGoal(THIS_ROBOT, goalNo);
    return 0;
  }

  std::size_t Environment::addAndSetPlannerGoal(const Vector2 goalPosition)
  {
    std::size_t goalNo = planner_->addGoal(goalPosition);
    planner_->setAgentGoal(THIS_ROBOT, goalNo);
    return goalNo;
  }

  std::size_t Environment::setSimParam(std::size_t simID)
  {
    simvect_[simID]->setTimeStep(SIM_TIME_STEP);
    simvect_[simID]->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_SPEED, MAX_SPEED, 0.0f, 0.6f, STOP, 0.0f);
  }

  bool Environment::getVirtualAgentReachedGoal(std::size_t simID, std::size_t agentNo)
  {
    return simvect_[simID]->agents_[agentNo]->reachedGoal_;
  }

  void Environment::doPlannerStep()
  {
    planner_->doStep();
  }

  void Environment::doSimulatorStep(std::size_t simID)
  {
    simvect_[simID]->doStep();
  }

  std::size_t Environment::addAndSetSimGoal(std::size_t simID, std::size_t agentNo, const Vector2 goalPosition)
  {
    std::size_t goalNo = simvect_[simID]->addGoal(goalPosition);
    simvect_[simID]->setAgentGoal(agentNo, goalNo);
    return goalNo;
  }

  void Environment::setupPlannerModel(std::size_t agentNo)
  {    
    std::map<std::size_t, std::size_t> simIDs;
    std::size_t numGoals = planner_->getNumGoals();

    for (std::size_t i = 0; i < numGoals; ++i) 
    {
      simIDs[i] = this->addSimulation();
      simvect_[simIDs[i]]->setAgentGoal(agentNo, i);
      // std::cout << "simID=" << simIDs[i] << " ";
      std::size_t simnumGoals = simvect_[simIDs[i]]->getNumGoals();
      // std::cout << " simNumGoals=" << simnumGoals << std::endl;
      std::cout << "Assigned GoalPos" << i << "of" << simnumGoals << ": " << simvect_[simIDs[i]]->getGoalPosition(i) << std::endl;
    }

    simIDs_ = simIDs;
  }

  std::map<std::size_t, std::size_t> Environment::setupModel(std::size_t agentNo, std::map<std::size_t, Vector2> possGoals)
  {    
    std::map<std::size_t, std::size_t> simIDs;
    std::size_t numGoals = possGoals.size();

    for (std::size_t i = 0; i < numGoals; ++i) 
    {
      simIDs[i] = this->addSimulation();
      std::size_t goalID = simvect_[simIDs[i]]->addGoal(possGoals[i]);
      simvect_[simIDs[i]]->setAgentGoal(agentNo, goalID);
      // std::cout << "simID=" << simIDs[i] << " ";
      // std::cout << " simNumGoals=" << simnumGoals << std::endl;
      // std::cout << "Assigned GoalPos" << i << "of" << numGoals << ": " << simvect_[simIDs[i]]->getGoalPosition(goalID) << std::endl;
    }

    return simIDs;
  }

  std::size_t Environment::inferGoals(std::size_t agentNo, std::map<std::size_t, std::size_t> simIDs)
  {

    const Vector2 currVel = planner_->getAgentVelocity(agentNo);  // TODO: GET FROM TRACKER
    std::map<std::size_t, float> inferredGoals;

    // std::cout << "simID size " << simIDs.size() << std::endl;

    for (std::size_t j = 0; j < simIDs.size(); ++j)
    {
      this->doSimulatorStep(simIDs[j]);
      // std::cout << "dostep" << std::endl;
      Vector2 simVel = simvect_[simIDs[j]]->getAgentVelocity(agentNo);
      // std::cout << "getsimvel" << std::endl;
      std::cout << "currVel=[" << currVel << "] " << "simVel=[" << simVel << "]" << std::endl;
      inferredGoals[j] = normaldiff(currVel, simVel);
      // std::cout << "DifftoGoal" << j << "=" << inferredGoals[j] << std::endl;
      // this->deleteSimulation(simIDs[j]);
    }

    // std::size_t goalInferID;
    // if (inferredAgentGoalsSum_.empty())
    // {
    //   goalInferID = 0;
    // }
    // else
    // {
    //   goalInferID = inferredAgentGoalsSum_.size();
    // }

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
        std::cout << "Goal" << j << "=" << inferredGoals[j] << " ";
        inferredAgentGoalsSum_[agentNo][j] += inferredGoals[j]; 
        inferredGoalsTotal += 1 / inferredAgentGoalsSum_[agentNo][j];
        std::cout << "GoalSum" << j << "=" << inferredAgentGoalsSum_[agentNo][j] << " ";
    }
    std::cout << std::endl;

    std::cout << "Goal ratio=";
    float goalRatio[inferredGoals.size()];
    float maxLikelihoodRatio = 0.0f;
    std::size_t maxLikelihoodGoal = 0;
    for (std::size_t k = 0; k < inferredGoals.size(); ++k)
    {
    goalRatio[k] = ((1 / inferredAgentGoalsSum_[agentNo][k]) / inferredGoalsTotal);
    if (k != 0) {std::cout <<  ":"; }
    std::cout << goalRatio[k];
    if (goalRatio[k] > maxLikelihoodRatio) {maxLikelihoodRatio = goalRatio[k]; maxLikelihoodGoal = k;}
    }
    std::cout << std::endl;

    return maxLikelihoodGoal;

  }

  std::map<std::size_t, float> Environment::inferAllGoals(std::size_t agentNo)
  {

    const Vector2 currVel = planner_->getAgentVelocity(agentNo);  // TODO: GET FROM TRACKER
    std::map<std::size_t, float> inferredGoals;

    // std::cout << "simID size " << simIDs.size() << std::endl;

    for (std::size_t j = 0; j < simIDs_.size(); ++j)
    {
      this->doSimulatorStep(simIDs_[j]);
      // std::cout << "dostep" << std::endl;
      Vector2 simVel = simvect_[simIDs_[j]]->getAgentVelocity(agentNo);
      // std::cout << "getsimvel" << std::endl;
      std::cout << "currVel=[" << currVel << "] " << "simVel=[" << simVel << "]" << std::endl;
      inferredGoals[j] = normaldiff(currVel, simVel);
      std::cout << "DifftoGoal" << j << "=" << inferredGoals[j] << std::endl;
      // this->deleteSimulation(simIDs_[j]);

      // this->doSimulatorStep(iter->second);
      // std::cout << "dostep" << std::endl;
      // Vector2 simVel = simvect_[iter->second]->getAgentVelocity(agentNo);
      // std::cout << "getsimvel" << std::endl;
      // inferredGoals[iter->second] = normaldiff(currVel, simVel);
      // std::cout << "EGoal" << iter->first << "=" << inferredGoals[iter->second] << std::endl;
      // this->deleteSimulation(iter->second);
    }

    return inferredGoals;

  }

  std::size_t Environment::addSimulation()
  {
    std::size_t simID;
    if (simvect_.empty())
    {
      simID = 0;
    }
    else
    {
      // simID = simvect_.rbegin()->first + 1;

      simID = simvect_.size();
    }
    simvect_[simID] = new Simulator(nh_, "simulation", nActorID_, simID);

    std::size_t nAgents = planner_->getNumAgents();
    simvect_[simID]->setTimeStep(SIM_TIME_STEP);
    // simvect_[simID]->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_SPEED, MAX_SPEED, 0.0f, 0.6f, STOP, 0.0f); 
    simvect_[simID]->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_PEOPLE_SPEED, MAX_PEOPLE_SPEED, 0.0f, MAX_PEOPLE_ACCELERATION, STOP, 0.0f); 
    

    simvect_[simID]->goals_=planner_->goals_;

    for (std::size_t i = 0; i < nAgents; ++i)
    {
      Vector2 plannerPos = planner_->getAgentPosition(i);
      Vector2 plannerVel = planner_->getAgentVelocity(i);
      // Vector2 vplannerGoal = planner_->getGoalPosition(planner_->getAgentGoal(i));
      // std::size_t nplannerGoal = simvect_[simID]->addGoal(vplannerGoal);
      std::size_t nplannerGoal = planner_->getAgentGoal(i);
      simvect_[simID]->addAgent(sActorID_ + "_s" + boost::lexical_cast<std::string>(simID) + "Agent_" + boost::lexical_cast<std::string>(i), SIMAGENT, plannerPos, nplannerGoal);
      simvect_[simID]->setAgentVelocity(i, plannerVel);
    }
    // std::cout << "HRVO Simulation for " << getActorName(nActorID_) << " with " << nAgents << " Agents with SimID_" << simID << " constructed" << std::endl;
    return simID;
  }

  void Environment::deleteSimulation(std::size_t simID)
  {
    free(simvect_[simID]);
    simvect_.erase(simID);


  }

  

  void Environment::stopYoubot()
  {
    planner_->setAgentVelocity(THIS_ROBOT, STOP);
  }

  void Environment::emergencyStop()
  {
    for (std::size_t i = 0; i < planner_->getNumAgents(); ++i)
    {
        planner_->setAgentVelocity(i, STOP);
    }

    for(std::map<std::size_t, Simulator *>::iterator iter = simvect_.begin(); iter != simvect_.end(); ++iter)
    {
      std::size_t simID = iter->first;
      for (std::size_t i = 0; i < simvect_[simID]->getNumAgents(); ++i)
      {
        simvect_[simID]->setAgentVelocity(i, STOP);
      }
    }


  }



}