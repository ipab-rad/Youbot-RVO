/**
* Created by Alejandro Bordallo
* \file   Environment.cpp
* \brief  Defines the Environment class.
*/

#ifndef HRVO_ENVIRONMENT_H_
#include "Environment.h"
#endif

#ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
#endif

namespace hrvo {

  Environment::Environment(enum Actor actorID, const Vector2 startPos)
  { 
    this->loadConfig();   // Load configuration parameters from ROS launch file
    INFO("FROM_ENV=" << ENABLE_PLANNER << std::endl);
    nActorID_ = actorID;
    startPos_ = startPos;
    planner_ = new Simulator(nh_, "planner", nActorID_);
    this->setPlannerParam();
    sActorID_ = getActorName(nActorID_);
    startGoal_ = planner_->addGoal(startPos_);
    planner_->addAgent(getActorName(nActorID_), ROBOT, startPos_, startGoal_);
    this->goalSetup();
    trackOtherAgents_ = false;
    robotTrackerID_ = -1;
    simvectPoint_ = &simvect_;
    DEBUG("HRVO Planner for " << sActorID_ << " Constructed" << std::endl);
    Targsub = nh_.subscribe("/agent_1/PTrackingBridge/targetEstimations", 1, &Environment::receiveTrackerData, this);
    ROS_INFO("Suscribing to TargetEstimations");
  }

  Environment::~Environment()
  {
    delete planner_;
    planner_ = NULL;

    for (std::map<std::size_t, Simulator *>::iterator iter = simvect_.begin(); iter != simvect_.end(); ++iter) 
    {
      delete iter->second;
      iter->second = NULL;
      simvect_.erase(iter);
    }
  }

  void Environment::loadConfig()
  {
    ros::param::get("enablePlanner", ENABLE_PLANNER);
    ros::param::get("onlyOdometry", ONLY_ODOMETRY);
    ros::param::get("assignTrackerWhenAlone", ASSIGN_TRACKER_WHEN_ALONE);
    ros::param::param("rosFreq", ROS_FREQ, 10);
    ros::param::param("trackerOdomComparisons", TRACKER_ODOM_COMPARISONS, 10);
    ros::param::param("velocityAverageWindow", VELOCITY_AVERAGE_WINDOW, 1 * ROS_FREQ);
    ros::param::param("maxNoTrackedAgents", MAX_NO_TRACKED_AGENTS, 5);
  }

  void Environment::goalSetup()
  {
    goal1_ = this->addPlannerGoal(I_g1);
    goal2_ = this->addPlannerGoal(I_g2);
    goal3_ = this->addPlannerGoal(I_g3);
  }

  void Environment::setupPlanner()  // NOT USED. UNRESOLVED BUG WHERE ODOMETRY IS NOT EXTRACTED PROPERLY
  {
    currPos_ = this->getPlannerAgentPosition(THIS_ROBOT);
    currVel_ = this->getPlannerAgentVelocity(THIS_ROBOT);
    currGoal_ = this->getPlannerGoal();
    Vector2 curr_odom = planner_->getCurrOdomOffset(THIS_ROBOT);
    Vector2 prev_odom = planner_->getPrevOdomOffset(THIS_ROBOT);
    DEBUG("Extracted pos " << currPos_ << " vel " << currVel_ << " goal " << currGoal_ << std::endl);
    DEBUG("Extracted curr " << curr_odom << " prev " << prev_odom << std::endl);

    std::vector<Goal *> plannerGoals_ = planner_->goals_;

    // delete planner_;
    // planner_ = NULL;

    // planner_ = new Simulator(nh_, "planner", nActorID_);
    // this->setPlannerParam();
    // planner_->goals_ = plannerGoals_;
    // planner_->addAgent(getActorName(nActorID_), ROBOT, currPos_, currGoal_);
    // planner_->setAgentVelocity(THIS_ROBOT, currVel_);
    // planner_->setCurrOdomOffset(THIS_ROBOT, curr_odom);
    // planner_->setPrevOdomOffset(THIS_ROBOT, prev_odom);
    // planner_->setOdomUpdated(THIS_ROBOT, true);
  }

  void Environment::updateTracker()
  {
    // DEBUG("Received!" << std::endl);
    if (!msg_.identities.empty())
    {
      std::size_t numAgents = msg_.identities.size();

      std::map<int, std::size_t> ids = this->getTrackerIDs();
      DEBUG(this->getStringActorID()<<":"<<std::endl);

      // Check if trackers for existing agents are still active
      for(std::map<int, std::size_t>::iterator iter = trackedAgents_.begin(); iter != trackedAgents_.end(); ++iter)
      {
        int TrackID = iter->first;
        std::size_t AgentID = iter->second;
        bool found = false;

          
          DEBUG("Tracker" << TrackID << " for Agent" << AgentID);
          for (int i = 0; i < ids.size(); ++i)
          {
            if (ids[i] == TrackID)
            {
              DEBUG(" active" << std::endl);
              found = true;
            }
          }

          if (TrackID != robotTrackerID_ && AgentID == THIS_ROBOT)
          {
            ERR(this->getStringActorID() << " already has tracker " << robotTrackerID_ << " instead of " << TrackID << std::endl);
            trackedAgents_.erase(iter);
          }

          if (!found || ids.size() == 0)
          {
            DEBUG(" inactive" << std::endl);
            if (AgentID != THIS_ROBOT)
            {
              planner_->setAgentPosition(AgentID, EXIT);
              planner_->setAgentVelocity(AgentID, STOP);
              planner_->setAgentType(AgentID, INACTIVE);
              
            // std::find(planner_->agents_.begin(), planner_->agents_.end(), AgentID)!=planner_->agents_.end() {
            // for (std::vector<Agent *>::iterator Viter = planner_->agents_.begin(); Viter != planner_->agents_.end(); ++Viter) {
            // delete planner_->agents_[AgentID];
            // delete Viter;
            // Viter = NULL;
            }
              // planner_->agents_.erase(iterator __position);
            trackedAgents_.erase(iter);
            trackerCompOdom_.erase(TrackID);
          }

      }

      // Deactivate any pedestrian agents who are no longer being tracked
      for (std::size_t AgentNo = 1; AgentNo < planner_->getNumAgents(); ++AgentNo)
      {
        bool AgentTracked = false;
        for(std::map<int, std::size_t>::iterator iter = trackedAgents_.begin(); iter != trackedAgents_.end(); ++iter)
        {
          if (iter->second == AgentNo)  { AgentTracked = true;}
        }
        if (!AgentTracked && planner_->getAgentType(AgentNo) == PERSON )
        {
            planner_->setAgentPosition(AgentNo, EXIT);
            planner_->setAgentVelocity(AgentNo, STOP);
            planner_->setAgentType(AgentNo, INACTIVE);
        }

      }


      // Update existing tracked agents or create new agents for new trackers
      for (std::size_t i = 0; i < numAgents; ++i)
      {
        int TrackerID = msg_.identities[i];
        std::string sid = intToString(TrackerID);
        Vector2 agentPos = EXIT;
        if (INVERT_X)
        {
          agentPos = Vector2(-1 * msg_.positions[i].x, msg_.positions[i].y);
        }
        else
        {
          agentPos = Vector2(msg_.positions[i].x, msg_.positions[i].y);
        }
        Vector2 agentVel = STOP;
        if (!USE_TRACKER_VELOCITIES)
        {
          if (prevPos.find(TrackerID) != prevPos.end())
          {
            agentVel = (agentPos - prevPos[TrackerID]) * ROS_FREQ;
            prevPos[TrackerID] = agentPos;
          }
          else
          {
            prevPos[TrackerID] = agentPos;
          }
        }
        else
        {
          // agentVel = Vector2(-1 * msg_.velocities[i].x, msg_.velocities[i].y);
          if (INVERT_X)
          {
            agentVel = Vector2(-1 * msg_.averagedVelocities[i].x, msg_.averagedVelocities[i].y);
          }
          else
          {
          agentVel = Vector2(msg_.averagedVelocities[i].x, msg_.averagedVelocities[i].y);
          }
        }


        if (ASSIGN_TRACKER_WHEN_ALONE && (trackedAgents_.empty() || numAgents == 1))
        { 
          this->setAgentTracker(TrackerID, THIS_ROBOT);
          planner_->resetOdomPosition();
          DEBUG("Assigned lone tracker" << TrackerID << "to "<< sActorID_ << std::endl);
        }

        // If TrackerID has not been assigned to an agent yet, reassing to robot or create new agent
        if (trackOtherAgents_ && trackedAgents_.find(TrackerID)==trackedAgents_.end() && trackedAgents_.size() < MAX_NO_TRACKED_AGENTS )  
        { // TODO: Limit number of created agents
          if (TrackerID == robotTrackerID_ && ENABLE_PLANNER)
          { 
            this->setAgentTracker(TrackerID, THIS_ROBOT);
            planner_->resetOdomPosition();
            DEBUG("Re-assigned robot tracker" << TrackerID << " to "<< sActorID_ << std::endl);
          }
          else
          {
            trackedAgents_[TrackerID] = this->addPedestrianAgent("TrackedPerson" + sid, agentPos, this->addPlannerGoal(agentPos));
            DEBUG("New agent" << trackedAgents_[TrackerID] << " with tracker" << sid << std::endl);
            agentVelCount_[trackedAgents_[TrackerID]] = 0;
          }
        }

        // If TrackerID has been assigned already, update agent information
        if ((trackedAgents_[TrackerID] == THIS_ROBOT) && !ONLY_ODOMETRY)
        {
          planner_->setOdomNeeded(false);
          planner_->setAgentPosition(THIS_ROBOT, agentPos + trackerOffset);
        }
        else if (trackedAgents_.find(TrackerID)!=trackedAgents_.end())
        {

          // float avgVel = this->calculateAvgMaxSpeeds(TrackerID, agentVel);
          std::pair<float, Vector2> avgSpeedVel = this->calculateAvgMaxSpeeds(trackedAgents_[TrackerID], agentVel);
          float avgSpeed = avgSpeedVel.first;
          Vector2 avgVel = avgSpeedVel.second;

          planner_->setAgentPosition(trackedAgents_[TrackerID], agentPos + trackerOffset);
          // planner_->setAgentVelocity(trackedAgents_[TrackerID], agentVel);
          planner_->setAgentVelocity(trackedAgents_[TrackerID], avgVel);
          planner_->setAgentPrefSpeed(trackedAgents_[TrackerID], avgSpeed);
          planner_->setAgentMaxSpeed(trackedAgents_[TrackerID], maxSpeed_[trackedAgents_[TrackerID]]);
        // planner_->setAgentMaxAcceleration(trackedAgents_[TrackerID], maxAcc_);
        }

        // if (planner_->getOdomNeeded())
        // {


        // }
        // Increase comparison magnitude proportional to the difference between odometry and tracker positions
        if (ENABLE_PLANNER)
        {
          // if (trackerComparisonCounter_ < TRACKER_ODOM_COMPARISONS)
          // {
            // TODO: MOVING AVERAGE!!
            float odomdiff = sqrdiff(planner_->getOdomPosition(), agentPos);
            // if (planner_->getAgentType(trackedAgents_[TrackerID]) != INACTIVE)
            // {
              trackerCompOdom_[TrackerID].insert(trackerCompOdom_[TrackerID].begin(), odomdiff);
              trackerCompOdom_[TrackerID].resize(TRACKER_ODOM_COMPARISONS);
              DEBUG("Tracker " << TrackerID << " Pos " << agentPos << std::endl);
              // DEBUG("CompOdom " << odomdiff << std::endl);
            // }
          // }
        }
      }

      // Compare robot odometry with tracker positions in order to reacquire stolen tracker
      if (ENABLE_PLANNER)
      {
        // if (trackerComparisonCounter_ < TRACKER_ODOM_COMPARISONS)
        // {
        //   trackerComparisonCounter_++;
        // }
        // else 
        // {
          int TargetTrackerID = -1; // Initialisation values
          float minComp = 10.0f;
          std::map<int, float> odomSums;
          // Add up histories for all tracker odometry comparisons
          for(std::map<int, std::vector<float> >::iterator iter = trackerCompOdom_.begin(); iter != trackerCompOdom_.end(); ++iter)
          {
            int TrackerID = iter->first;
            std::vector<float> OdomComparison = iter->second;
            if (planner_->getAgentType(trackedAgents_[TrackerID]) != INACTIVE)
            {
              odomSums[TrackerID] = 0.0f;
              for(std::vector<float>::iterator iter = OdomComparison.begin(); iter != OdomComparison.end(); ++iter)
              {
                odomSums[TrackerID] += (*iter); 
              }
            }
          }
          for(std::map<int, float>::iterator iter = odomSums.begin(); iter != odomSums.end(); ++iter)
          {
            int TrackerID = iter->first;
            float odomSum = iter->second;
            if (odomSum < minComp)
            {
              TargetTrackerID = TrackerID;
              minComp = odomSum;
            }
          }



          if (TargetTrackerID == -1)
          { ERR("ERROR: NO TRACKER CAN BE ASSIGNED TO " << this->getStringActorID() << std::endl);}
          else if (trackedAgents_[TargetTrackerID] != THIS_ROBOT && trackerCompOdom_[TargetTrackerID].size() >= TRACKER_ODOM_COMPARISONS)
          {
            ERR("Tracker " << TargetTrackerID << "belongs to " << sActorID_ << " instead of " << trackedAgents_[TargetTrackerID] << std::endl);
            // If smallest is not the one assigned to robot, then substitute and delete extra tracked agent
            // planner_->setAgentPosition(trackedAgents_[robotTrackerID_], STOP);
            // planner_->setAgentVelocity(trackedAgents_[robotTrackerID_], STOP);
            // planner_->setAgentType(trackedAgents_[robotTrackerID_], INACTIVE);
            trackedAgents_.erase(robotTrackerID_);
            trackedAgents_[TargetTrackerID] = THIS_ROBOT;
            robotTrackerID_ = TargetTrackerID;
          }

            // Reset all comparisons

        // }
      }
      DEBUG(std::endl);

    }

  }

  std::map<int, std::size_t> Environment::getTrackerIDs()
  {
    std::size_t numAgents = msg_.identities.size();

    // DEBUG("Identities:");
    std::map<int, std::size_t> ids;     // First = Index, Second = Tracker ID
    for (int i = 0; i < numAgents; ++i)
    {
      ids[i] = msg_.identities[i];
      // DEBUG(ids[i] <<",");
    }
    // INFO(std::endl);
    return ids;
  }

  void Environment::receiveTrackerData(const PTrackingBridge::TargetEstimations::ConstPtr& msg)
  {
    msg_ = *msg;
  }

  void Environment::setPlannerParam()
  {
    planner_->setTimeStep(SIM_TIME_STEP);
    planner_->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_SPEED, MAX_SPEED, 0.0f, MAX_ACCELERATION, STOP, 0.0f); 
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

  void Environment::setPlannerGoal(std::size_t goalNo)
  {
    planner_->setAgentGoal(THIS_ROBOT, goalNo);
  }

  void Environment::editPlannerGoal(std::size_t goalNo, Vector2 goalPosition)
  {
    planner_->editGoal(goalNo, goalPosition);
  }

  void Environment::setPlannerInitialGoal(int goalIndex)
  {
    switch (goalIndex) {
    case 1:
      this->setPlannerGoal(goal1_);
      break;
    case 2:
      this->setPlannerGoal(goal2_);
      break;
    case 3:
      this->setPlannerGoal(goal3_);
      break;
    default:
      ERR("Initial goal for" << this->getStringActorID() << "does not exist!" << std::endl);
      break;
    }
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
    simvect_[simID]->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_SPEED, MAX_SPEED, 0.0f, MAX_PEOPLE_ACCELERATION, STOP, 0.0f);
  }

  bool Environment::getVirtualAgentReachedGoal(std::size_t simID, std::size_t agentNo)
  {
    return simvect_[simID]->agents_[agentNo]->reachedGoal_;
  }

  void Environment::setNextGoal()
  {
    switch (goalPlan_) {
    case GOAL_STOP:
      this->stopYoubot();
      break;
    case GOAL_CYCLE_CW:
      this->cycleGoalsClockwise();
      break;
    case GOAL_CYCLE_CCW:
      this->cycleGoalsCounterClockwise();
      break;
    case GOAL_1_2:
      this->cycleGoals1_2();
      break;
    case GOAL_2_3:
      this->cycleGoals2_3();
      break;
    case GOAL_3_1:
      this->cycleGoals3_1();
      break;
    case FOLLOW_AGENT:
      // this->setPlannerGoal(goal3_);
      break;
    default:
      ERR("Goal Plan for" << this->getStringActorID() << "does not exist!" << std::endl);
      break;
    }
  }

  void Environment::cycleGoalsClockwise()
  {
    if (this->getPlannerGoal() == goal1_)
    {this->setPlannerGoal(goal3_);}
    else if (this->getPlannerGoal() == goal3_)
      {this->setPlannerGoal(goal2_);}
    else if (this->getPlannerGoal() == goal2_)
      {this->setPlannerGoal(goal1_);}
    else
    {ERR("Could not cycle goals for " << this->getStringActorID() << std::endl)}
  }

  void Environment::cycleGoalsCounterClockwise()
  {
    if (this->getPlannerGoal() == goal1_)
    {this->setPlannerGoal(goal2_);}
    else if (this->getPlannerGoal() == goal2_)
      {this->setPlannerGoal(goal3_);}
    else if (this->getPlannerGoal() == goal3_)
      {this->setPlannerGoal(goal1_);}
    else
    {ERR("Could not cycle goals for " << this->getStringActorID() << std::endl)}
  }

  void Environment::cycleGoals1_2()
  {
    if (this->getPlannerGoal() != goal2_)
    {this->setPlannerGoal(goal2_);}
    else if (this->getPlannerGoal() != goal1_)
      {this->setPlannerGoal(goal1_);}
    else
    {ERR("Could not cycle goals for " << this->getStringActorID() << std::endl)}
  }

  void Environment::cycleGoals2_3()
  {
    if (this->getPlannerGoal() != goal3_)
    {this->setPlannerGoal(goal3_);}
    else if (this->getPlannerGoal() != goal2_)
      {this->setPlannerGoal(goal2_);}
    else
    {ERR("Could not cycle goals for " << this->getStringActorID() << std::endl)}
  }

  void Environment::cycleGoals3_1()
  {
    if (this->getPlannerGoal() != goal3_)
    {this->setPlannerGoal(goal3_);}
    else if (this->getPlannerGoal() != goal1_)
      {this->setPlannerGoal(goal1_);}
    else
    {ERR("Could not cycle goals for " << this->getStringActorID() << std::endl)}
  }

  void Environment::doPlannerStep()
  {
    if (planner_->odomNeeded_ && planner_->getAgentType(THIS_ROBOT) != INACTIVE) {WARN(sActorID_<< " using odometry for navigation" << std::endl);}
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
    simvect_[simID]->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_SPEED, MAX_SPEED, 0.0f, MAX_PEOPLE_ACCELERATION, STOP, 0.0f); 
    // simvect_[simID]->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_PEOPLE_SPEED, MAX_PEOPLE_SPEED, 0.0f, MAX_PEOPLE_ACCELERATION, STOP, 0.0f); 
    

    simvect_[simID]->goals_=planner_->goals_;

    for (std::size_t i = 0; i < nAgents; ++i)
    {
      Vector2 plannerPos = planner_->getAgentPosition(i);
      Vector2 plannerVel = planner_->getAgentVelocity(i);
      float prefSpeed = planner_->getAgentPrefSpeed(i);
      float maxSpeed = planner_->getAgentMaxSpeed(i);
      // Vector2 vplannerGoal = planner_->getGoalPosition(planner_->getAgentGoal(i));
      // std::size_t nplannerGoal = simvect_[simID]->addGoal(vplannerGoal);
      std::size_t nplannerGoal = planner_->getAgentGoal(i);
      simvect_[simID]->addAgent(sActorID_ + "_s" + boost::lexical_cast<std::string>(simID) + "Agent_" + boost::lexical_cast<std::string>(i), SIMAGENT, plannerPos, nplannerGoal);
      simvect_[simID]->setAgentVelocity(i, plannerVel);
      simvect_[simID]->setAgentPrefSpeed(i, prefSpeed);
      simvect_[simID]->setAgentMaxSpeed(i, maxSpeed);
    }
    // std::cout << "HRVO Simulation for " << getActorName(nActorID_) << " with " << nAgents << " Agents with SimID_" << simID << " constructed" << std::endl;
    return simID;
  }

  void Environment::deleteSimulation(std::size_t simID)
  {
    delete simvect_[simID];
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

    // for(std::map<std::size_t, Simulator *>::iterator iter = simvect_.begin(); iter != simvect_.end(); ++iter)
    // {
    //   std::size_t simID = iter->first;
    //   for (std::size_t i = 0; i < simvect_[simID]->getNumAgents(); ++i)
    //   {
    //     simvect_[simID]->setAgentVelocity(i, STOP);
    //   }
    // }
  }

  std::pair<float, Vector2> Environment::calculateAvgMaxSpeeds(int AgentID, Vector2 AgentVel)
  {
    // agentVelHistory_[trackedAgents_[AgentID]][agentVelCount_[trackedAgents_[AgentID]]] = AgentVel;
    // agentVelHistory_[trackedAgents_[AgentID]][agentVelCount_[trackedAgents_[AgentID]]] = abs(AgentVel);
    // agentVelCount_[trackedAgents_[AgentID]] += 1;
    // if (agentVelCount_[trackedAgents_[AgentID]] == VELOCITY_AVERAGE_WINDOW)
    //   {agentVelCount_[trackedAgents_[AgentID]] = 0;}
    // TODO: Make into vector, insert velocity at the beginning, eliminate last element if larger than window
    // TODO: Implement discounted sum over past Sum:(1 - disc)^t-1 * Value  where disc(0 < disc =< 1)
    agentVelHistory_[AgentID].insert(agentVelHistory_[AgentID].begin(), AgentVel);
    agentVelHistory_[AgentID].resize(VELOCITY_AVERAGE_WINDOW);
    std::size_t VelHist = agentVelHistory_[AgentID].size();

    float avgSpeed = 0.0f;
    float avgVelx = 0.0f;
    float avgVely = 0.0f;
    for(std::vector<Vector2>::iterator vel = agentVelHistory_[AgentID].begin(); vel != agentVelHistory_[AgentID].end(); ++vel)
    {
      avgSpeed += abs(*vel);
      avgVelx += (*vel).getX();
      avgVely += (*vel).getY();
    }
    avgSpeed = avgSpeed / agentVelHistory_[AgentID].size();
    Vector2 avgVel = Vector2(avgVelx / VelHist, avgVely / VelHist);
    if (avgSpeed > maxSpeed_[AgentID])
    {maxSpeed_[AgentID] = avgSpeed;}

    // DEBUG("Agent" << AgentID << " AvgVel="<<avgSpeed<<" maxVel="<<maxSpeed_[AgentID]<<std::endl);

    return std::make_pair(avgSpeed, avgVel);
  }



}