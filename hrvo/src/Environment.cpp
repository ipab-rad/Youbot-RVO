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
    DEBUG("HRVO Planner for " << sActorID_ << " Constructed" << std::endl);
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
    // DEBUG("Received!" << std::endl);
    if (!msg->identities.empty())
    {
      std::size_t numAgents = msg->identities.size();


      DEBUG("Identities:");
      std::map<int, std::size_t> ids;
      for (int i = 0; i < numAgents; ++i)
      {
        ids[i] = msg->identities[i];
        DEBUG(ids[i] <<",");
      }
      INFO(std::endl);
      
      for(std::map<int, std::size_t>::iterator iter = trackedAgents_.begin(); iter != trackedAgents_.end(); ++iter)
      {
        bool found = false;
        DEBUG("Tracker:" << iter->first << " identity:" << iter->second);
        for (int i = 0; i < ids.size(); ++i)
        {
          if (ids[i] == iter->first)
          {
            DEBUG(" active" << std::endl);
            found = true;
          }
        }
        if (!found || ids.size() == 0)
        {
          DEBUG(" eliminated" << std::endl);
          if (iter->second != THIS_ROBOT)
          {
            {planner_->setAgentPosition(iter->second, STOP);
              planner_->setAgentVelocity(iter->second, STOP);}
          // std::find(planner_->agents_.begin(), planner_->agents_.end(), iter->second)!=planner_->agents_.end() {
          // for (std::vector<Agent *>::iterator Viter = planner_->agents_.begin(); Viter != planner_->agents_.end(); ++Viter) {
          // delete planner_->agents_[iter->second];
          // delete Viter;
          // Viter = NULL;
          }
            // planner_->agents_.erase(iterator __position);
          trackedAgents_.erase(iter);
        }

      }



      for (std::size_t i = 0; i < numAgents; ++i)
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

        if (assignTrackerWhenAlone && (trackedAgents_.empty() || numAgents == 1))
        {
          trackedAgents_[id] = THIS_ROBOT;
          // DEBUG("Assigned tracker" << id << "to Youbot_" << nActorID_ << std::endl);
        }
        if (trackedAgents_.find(id)==trackedAgents_.end() && trackedAgents_.size() < MAX_NO_TRACKED_AGENTS )
        {
          trackedAgents_[id] = this->addPedestrianAgent("TrackedPerson" + sid, agentPos, this->addPlannerGoal(agentPos));
          DEBUG("New agent" << trackedAgents_[id] << " with tracker" << sid << std::endl);
          planner_->setAgentMaxAccel(trackedAgents_[id], 2.0f);
          planner_->setAgentMaxSpeed(trackedAgents_[id], 2.0f);
          planner_->setAgentPrefSpeed(trackedAgents_[id], 1.0f);
        }
        else
        {
          if (trackedAgents_[id] == THIS_ROBOT)
          {
            planner_->setOdomNeeded(false);
            planner_->setAgentPosition(THIS_ROBOT, agentPos + trackerOffset);
          }
          else if (trackedAgents_.find(id)!=trackedAgents_.end())
          {
          planner_->setAgentPosition(trackedAgents_[id], prevPos_);
          planner_->setAgentVelocity(trackedAgents_[id], agentVel);
          // planner_->setAgentVelocity(trackedAgents_[id], STOP);
          prevPos_ = agentPos;
          }
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
    planner_->setOdomNeeded(true);
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
      // INFO("simID=" << simIDs[i] << " ");
      std::size_t simnumGoals = simvect_[simIDs[i]]->getNumGoals();
      // INFO(" simNumGoals=" << simnumGoals << std::endl);
      INFO("Assigned GoalPos" << i << "of" << simnumGoals << ": " << simvect_[simIDs[i]]->getGoalPosition(i) << std::endl);
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
      // INFO("simID=" << simIDs[i] << " ");
      // INFO(simNumGoals=" << simnumGoals << std::endl);
      // INFO("Assigned GoalPos" << i << "of" << numGoals << ": " << simvect_[simIDs[i]]->getGoalPosition(goalID) << std::endl);
    }

    return simIDs;
  }

  std::size_t Environment::inferGoals(std::size_t agentNo, std::map<std::size_t, std::size_t> simIDs)
  {

    const Vector2 currVel = planner_->getAgentVelocity(agentNo);  // TODO: GET FROM TRACKER
    std::map<std::size_t, float> inferredGoals;

    // DEBUG("simID size " << simIDs.size() << std::endl);

    for (std::size_t j = 0; j < simIDs.size(); ++j)
    {
      this->doSimulatorStep(simIDs[j]);
      // DEBUG("dostep" << std::endl);
      Vector2 simVel = simvect_[simIDs[j]]->getAgentVelocity(agentNo);
      // DEBUG("getsimvel" << std::endl);
      INFO("currVel=[" << currVel << "] " << "simVel=[" << simVel << "]" << std::endl);
      inferredGoals[j] = normaldiff(currVel, simVel);
      // DEBUG("DifftoGoal" << j << "=" << inferredGoals[j] << std::endl);
      // this->deleteSimulation(simIDs[j]);
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
        INFO("Goal" << j << "=" << inferredGoals[j] << " ");
        inferredAgentGoalsSum_[agentNo][j] += inferredGoals[j]; 
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
      INFO("currVel=[" << currVel << "] " << "simVel=[" << simVel << "]" << std::endl);
      inferredGoals[j] = normaldiff(currVel, simVel);
      INFO("DifftoGoal" << j << "=" << inferredGoals[j] << std::endl);
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