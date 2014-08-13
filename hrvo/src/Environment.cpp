/**
* Created by Alejandro Bordallo
* \file   Environment.cpp
* \brief  Defines the Environment class.
*/

#ifndef HRVO_ENVIRONMENT_H_
#include "Environment.h"
#endif




// #ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
// #endif

// #ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
// #endif

namespace hrvo {


  Environment::Environment(enum Actor actorID, const Vector2 startPos)
  {
    nActorID_ = actorID;
    startPos_ = startPos;
    planner_ = new Simulator("planner", nActorID_);
    this->setPlannerParam();
    this->compsActorID();
    startGoal_ = planner_->addGoal(startPos);
    planner_->addAgent(getActorName(nActorID_), ROBOT, startPos_, startGoal_);
    std::cout << "HRVO Planner for " << getActorName(nActorID_) << " Constructed" << std::endl;
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
  
  void Environment::compsActorID()
  {
    std::ostringstream ostr;
    ostr << nActorID_;
    sActorID_ = ostr.str();
  }

  std::size_t Environment::addTracker()
  {
  ;
  }

  void Environment::setPlannerParam()
  {
    planner_->setTimeStep(SIM_TIME_STEP);
    planner_->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_SPEED, MAX_SPEED, 0.0f, 0.6f, STOP, 0.0f); 
  }

  std::size_t Environment::addVirtualAgent(std::string id, const Vector2 startPos, std::size_t goalNo)
  {
    planner_->addAgent(sActorID_ + "_v" + id, SIMAGENT, startPos, goalNo);
  }

  std::size_t Environment::addPlannerGoal(const Vector2 goalPosition)
  {
    std::size_t goalNo = planner_->addGoal(goalPosition);
    return goalNo;
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

  // bool Environment::plannerReachedGoals() 
  // { 
  //   return planner->reachedGoals_;
  // }

  std::size_t Environment::addSimulation()
  {
    if (simvect_.empty())
    {
      std::size_t simID = 0;
      simvect_[simID] = new Simulator("simulation", nActorID_, simID);
      return simID;
    }
    
    std::size_t simID = simvect_.rbegin()->first + 1;
    simvect_[simID] = new Simulator("simulation", nActorID_, simID);
    return simID;
  }

  int Environment::deleteSimulation(std::size_t simID)
  {
    delete simvect_[simID];
    simvect_.erase(simID);

    return 0;
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