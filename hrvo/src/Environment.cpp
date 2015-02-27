/**
* Created by Alejandro Bordallo
* \file   Environment.cpp
* \brief  Defines the Environment class.
*/

#include "Environment.h"

namespace hrvo {

  Environment::Environment(enum Actor actorID, const Vector2 startPos)
  {
    nActorID_ = actorID;
    startPos_ = startPos;
    planner_ = new Simulator(nh_, "planner", nActorID_);
    if (!HRVO_PLANNER)
    {newPlanner_ = new Planner(nh_);}
    newGoal = true;
    reachedGoal = false;
    this->setPlannerParam();
    sActorID_ = getActorName(nActorID_);
    startGoal_ = planner_->addGoal(startPos_);
    if (IS_AMCL_ACTIVE) {this->initAMCL();}
    planner_->addAgent(getActorName(nActorID_), ROBOT, startPos_, startGoal_);
    posePub_ = nh_.advertise<geometry_msgs::Pose>("/" + sActorID_ + "/true_pose", 1);
    if (!TRACK_ROBOTS) {this->initRobotTrackers();}
    this->goalSetup();
    simvectPoint_ = &simvect_;
    this->initTracker();
    DEBUG("Environment for " << sActorID_ << " constructed" << std::endl);
  }

  Environment::~Environment()
  {
    delete planner_;
    planner_ = NULL;

    delete tracker_;
    tracker_ = NULL;

    delete amclwrapper_;
    amclwrapper_ = NULL;

    for (std::map<std::size_t, Simulator *>::iterator iter = simvect_.begin(); iter != simvect_.end(); ++iter)
    {
      delete iter->second;
      iter->second = NULL;
      simvect_.erase(iter);
    }
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

    /*
    delete planner_;
    planner_ = NULL;

    planner_ = new Simulator(nh_, "planner", nActorID_);
    this->setPlannerParam();
    planner_->goals_ = plannerGoals_;
    planner_->addAgent(getActorName(nActorID_), ROBOT, currPos_, currGoal_);
    planner_->setAgentVelocity(THIS_ROBOT, currVel_);
    planner_->setCurrOdomOffset(THIS_ROBOT, curr_odom);
    planner_->setPrevOdomOffset(THIS_ROBOT, prev_odom);
    planner_->setOdomUpdated(THIS_ROBOT, true);
    */
  }

  void Environment::initAMCL()
  {
    /* Initialise wrapper to update AMCL messages */
    amclwrapper_ = new AMCLWrapper(sActorID_);
    amclwrapper_->setEnvPointer(this);
    amclwrapper_->setPlannerPointer(planner_);
    DEBUG("AMCL for " << sActorID_ << " initialised" << std::endl);
  }

  void Environment::initTracker()
  {
    tracker_ = new Tracker();
    tracker_->setEnvPointer(this);
    tracker_->setPlannerPointer(planner_);
    DEBUG("Tracker setup for " << sActorID_ << " initialised" << std::endl);
  }

  void Environment::initRobotTrackers()
  {
    // DO THIS ONCE ALL ROBOTS HAVE STARTED
    // CHECK ALL EXISITING ROBOTS IN THE EXPERIMENT
    // ADD ALL ROBOTS TO THE PLANNER
    // SET UP ODOM/AMCL SUBSCRIBERS.
    DEBUG(sActorID_ << " subscribed to: ")
    if ((nActorID_ != YOUBOT_1) && MEGATRON_ACTIVE)
    {
      std::size_t agentid = planner_->addAgent(getActorName(YOUBOT_1), SIMAGENT, EXIT, startGoal_);
      trackedRobots_[YOUBOT_1] = agentid;
      robotPoseSubs[agentid] = nh_.subscribe("/youbot_1/true_pose", 1, &Environment::receiveRobot1Pose, this);
      robotVelSubs[agentid] = nh_.subscribe("/youbot_1/cmd_vel", 1, &Environment::receiveRobot1Vel, this);
      DEBUG(getActorName(YOUBOT_1) << " id " << agentid << ", ");
    }
    if ((nActorID_ != YOUBOT_2) && SOUNDWAVE_ACTIVE)
    {
      std::size_t agentid = planner_->addAgent(getActorName(YOUBOT_2), SIMAGENT, EXIT, startGoal_);
      trackedRobots_[YOUBOT_2] = agentid;
      robotPoseSubs[agentid] = nh_.subscribe("/youbot_2/true_pose", 1, &Environment::receiveRobot2Pose, this);
      robotVelSubs[agentid] = nh_.subscribe("/youbot_2/cmd_vel", 1, &Environment::receiveRobot2Vel, this);
      DEBUG(getActorName(YOUBOT_2) << " id " << agentid << ", ");
    }
    if ((nActorID_ != YOUBOT_3) && STARSCREAM_ACTIVE)
    {
      std::size_t agentid = planner_->addAgent(getActorName(YOUBOT_3), SIMAGENT, EXIT, startGoal_);
      trackedRobots_[YOUBOT_3] = agentid;
      robotPoseSubs[agentid] = nh_.subscribe("/youbot_3/true_pose", 1, &Environment::receiveRobot3Pose, this);
      robotVelSubs[agentid] = nh_.subscribe("/youbot_3/cmd_vel", 1, &Environment::receiveRobot3Vel, this);
      DEBUG(getActorName(YOUBOT_3) << " id " << agentid << ", ");
    }
    if ((nActorID_ != YOUBOT_4) && BLACKOUT_ACTIVE)
    {
      std::size_t agentid = planner_->addAgent(getActorName(YOUBOT_4), SIMAGENT, EXIT, startGoal_);
      trackedRobots_[YOUBOT_4] = agentid;
      robotPoseSubs[agentid] = nh_.subscribe("/youbot_4/true_pose", 1, &Environment::receiveRobot4Pose, this);
      robotVelSubs[agentid] = nh_.subscribe("/youbot_4/cmd_vel", 1, &Environment::receiveRobot4Vel, this);
      DEBUG(getActorName(YOUBOT_4) << " id " << agentid << ", ");
    }
    if ((nActorID_ != YOUBOT_5) && THUNDERCRACKER_ACTIVE)
    {
      std::size_t agentid = planner_->addAgent(getActorName(YOUBOT_5), SIMAGENT, EXIT, startGoal_);
      trackedRobots_[YOUBOT_5] = agentid;
      robotPoseSubs[agentid] = nh_.subscribe("/youbot_5/true_pose", 1, &Environment::receiveRobot5Pose, this);
      robotVelSubs[agentid] = nh_.subscribe("/youbot_5/cmd_vel", 1, &Environment::receiveRobot5Vel, this);
      DEBUG(getActorName(YOUBOT_5) << " id " << agentid << ", ");
    }
    if ((nActorID_ != PRIME) && PRIME_ACTIVE)
    {
      std::size_t agentid = planner_->addAgent(getActorName(PRIME), SIMAGENT, EXIT, startGoal_);
      trackedRobots_[PRIME] = agentid;
      robotPoseSubs[agentid] = nh_.subscribe("/prime/true_pose", 1, &Environment::receiveRobot6Pose, this);
      robotVelSubs[agentid] = nh_.subscribe("/prime/cmd_vel", 1, &Environment::receiveRobot6Vel, this);
      DEBUG(getActorName(PRIME)  << " id " << agentid);
    }
    DEBUG(std::endl);
    DEBUG("Multi-robot pose/vels subscriptions for " << sActorID_ << " complete" << std::endl);
  }

  // I hate myself for this....
  void Environment::receiveRobot1Pose(const geometry_msgs::Pose& msg)
  {
    robotPoses[trackedRobots_[YOUBOT_1]].setX(msg.position.x); 
    robotPoses[trackedRobots_[YOUBOT_1]].setY(msg.position.y);
  }
  void Environment::receiveRobot2Pose(const geometry_msgs::Pose& msg)
  {
    robotPoses[trackedRobots_[YOUBOT_2]].setX(msg.position.x); 
    robotPoses[trackedRobots_[YOUBOT_2]].setY(msg.position.y);
  }
  void Environment::receiveRobot3Pose(const geometry_msgs::Pose& msg)
  {
    robotPoses[trackedRobots_[YOUBOT_3]].setX(msg.position.x); 
    robotPoses[trackedRobots_[YOUBOT_3]].setY(msg.position.y);
  }
  void Environment::receiveRobot4Pose(const geometry_msgs::Pose& msg)
  {
    robotPoses[trackedRobots_[YOUBOT_4]].setX(msg.position.x); 
    robotPoses[trackedRobots_[YOUBOT_4]].setY(msg.position.y);
  }
  void Environment::receiveRobot5Pose(const geometry_msgs::Pose& msg)
  {
    robotPoses[trackedRobots_[YOUBOT_5]].setX(msg.position.x); 
    robotPoses[trackedRobots_[YOUBOT_5]].setY(msg.position.y);
  }
  void Environment::receiveRobot6Pose(const geometry_msgs::Pose& msg)
  {
    robotPoses[trackedRobots_[PRIME]].setX(msg.position.x); 
    robotPoses[trackedRobots_[PRIME]].setY(msg.position.y);
  }

  void Environment::receiveRobot1Vel(const geometry_msgs::Twist& msg)
  {
    robotVels[trackedRobots_[YOUBOT_1]].setX(msg.linear.x); 
    robotVels[trackedRobots_[YOUBOT_1]].setY(msg.linear.y);
  }
  void Environment::receiveRobot2Vel(const geometry_msgs::Twist& msg)
  {
    robotVels[trackedRobots_[YOUBOT_2]].setX(msg.linear.x); 
    robotVels[trackedRobots_[YOUBOT_2]].setY(msg.linear.y);
  }
  void Environment::receiveRobot3Vel(const geometry_msgs::Twist& msg)
  {
    robotVels[trackedRobots_[YOUBOT_3]].setX(msg.linear.x); 
    robotVels[trackedRobots_[YOUBOT_3]].setY(msg.linear.y);
  }
  void Environment::receiveRobot4Vel(const geometry_msgs::Twist& msg)
  {
    robotVels[trackedRobots_[YOUBOT_4]].setX(msg.linear.x); 
    robotVels[trackedRobots_[YOUBOT_4]].setY(msg.linear.y);
  }
  void Environment::receiveRobot5Vel(const geometry_msgs::Twist& msg)
  {
    robotVels[trackedRobots_[YOUBOT_5]].setX(msg.linear.x); 
    robotVels[trackedRobots_[YOUBOT_5]].setY(msg.linear.y);
  }
  void Environment::receiveRobot6Vel(const geometry_msgs::Twist& msg)
  {
    robotVels[trackedRobots_[PRIME]].setX(msg.linear.x); 
    robotVels[trackedRobots_[PRIME]].setY(msg.linear.y);
  }

  // void Environment::receiveRobotPose(const geometry_msgs::Pose& msg,
  //   const ros::MessageEvent<std_msgs::String const>& event)
  // {
  //   const std::string& pub_name = event.getPublisherName();
  //   if (pub_name.compare(1,8,"youbot_1"))
  //   {
  //     robotPoses[trackedRobots_[YOUBOT_1]].setX(msg.position.x); 
  //     robotPoses[trackedRobots_[YOUBOT_1]].setY(msg.position.y);
  //   }
  //   else if (pub_name.compare(1,8,"youbot_2"))
  //   {
  //     robotPoses[trackedRobots_[YOUBOT_2]].setX(msg.position.x);
  //     robotPoses[trackedRobots_[YOUBOT_2]].setY(msg.position.y);
  //   }
  //   else if (pub_name.compare(1,8,"youbot_3"))
  //   {
  //     robotPoses[trackedRobots_[YOUBOT_3]].setX(msg.position.x);
  //     robotPoses[trackedRobots_[YOUBOT_3]].setY(msg.position.y);
  //   }
  //   else if (pub_name.compare(1,8,"youbot_4"))
  //   {
  //     robotPoses[trackedRobots_[YOUBOT_4]].setX(msg.position.x);
  //     robotPoses[trackedRobots_[YOUBOT_4]].setY(msg.position.y);
  //   }
  //   else if (pub_name.compare(1,8,"youbot_5"))
  //   {
  //     robotPoses[trackedRobots_[YOUBOT_5]].setX(msg.position.x);
  //     robotPoses[trackedRobots_[YOUBOT_5]].setY(msg.position.y);
  //   }
  //   else if (pub_name.compare(1,5,"prime"))
  //   {
  //     robotPoses[trackedRobots_[PRIME]].setX(msg.position.x);
  //     robotPoses[trackedRobots_[PRIME]].setY(msg.position.y);
  //   }
  // }

  // void Environment::receiveRobotVel(const geometry_msgs::Twist& msg,
  //   const ros::MessageEvent<std_msgs::String const>& event)
  // {
  //   const std::string& pub_name = event.getPublisherName();
  //   if (pub_name.compare(1,8,"youbot_1"))
  //   {
  //     robotVels[trackedRobots_[YOUBOT_1]].setX(msg.linear.x); 
  //     robotVels[trackedRobots_[YOUBOT_1]].setY(msg.linear.y);
  //   }
  //   else if (pub_name.compare(1,8,"youbot_2"))
  //   {
  //     robotVels[trackedRobots_[YOUBOT_2]].setX(msg.linear.x);
  //     robotVels[trackedRobots_[YOUBOT_2]].setY(msg.linear.y);
  //   }
  //   else if (pub_name.compare(1,8,"youbot_3"))
  //   {
  //     robotVels[trackedRobots_[YOUBOT_3]].setX(msg.linear.x);
  //     robotVels[trackedRobots_[YOUBOT_3]].setY(msg.linear.y);
  //   }
  //   else if (pub_name.compare(1,8,"youbot_4"))
  //   {
  //     robotVels[trackedRobots_[YOUBOT_4]].setX(msg.linear.x);
  //     robotVels[trackedRobots_[YOUBOT_4]].setY(msg.linear.y);
  //   }
  //   else if (pub_name.compare(1,8,"youbot_5"))
  //   {
  //     robotVels[trackedRobots_[YOUBOT_5]].setX(msg.linear.x);
  //     robotVels[trackedRobots_[YOUBOT_5]].setY(msg.linear.y);
  //   }
  //   else if (pub_name.compare(1,5,"prime"))
  //   {
  //     robotVels[trackedRobots_[PRIME]].setX(msg.linear.x);
  //     robotVels[trackedRobots_[PRIME]].setY(msg.linear.y);
  //   }
  // }


  void Environment::updateLocalisation(bool USE_TRACKER)
  {
    if (IS_AMCL_ACTIVE)
    {
      amclwrapper_->updatePose();
      if (amclwrapper_->is_msg_received)
      {
        planner_->setAMCLPose(THIS_ROBOT, amclwrapper_->get_position());
      }
      planner_->setCurrOdomOffset(THIS_ROBOT, amclwrapper_->get_odom_position());
      // planner_->setCurrOdomOffset(THIS_ROBOT, amclwrapper_->get_position());
      // planner_->setSensedOrientation(THIS_ROBOT, amclwrapper_->get_orientation());
      amclwrapper_->pretty_print_pose();
    }
    if (USE_TRACKER) {
      tracker_->updateTracker();
    }
    if (!TRACK_ROBOTS)
    {
      //PUBLISH POSE
      geometry_msgs::Pose pose;
      pose.position.x = planner_->getAgentPosition(THIS_ROBOT).getX();
      pose.position.y = planner_->getAgentPosition(THIS_ROBOT).getY();
      posePub_.publish(pose);
      DEBUG("Publishing true pose for " << sActorID_ << std::endl << pose.position << std::endl);
    }
  }

  void Environment::updateRobotAgents()
  {
    // UPDATE POSE/VEL OF ALL TRACKED ROBOTS
    for(std::map<Actor, std::size_t>::iterator iter = trackedRobots_.begin(); iter != trackedRobots_.end(); ++iter)
    {
      std::size_t agentID = iter->second;
      planner_->setAgentPosition(agentID, robotPoses[agentID]);
      planner_->setAgentVelocity(agentID, robotVels[agentID]);
      DEBUG("Setting Agent " << agentID <<
       " Pos " << robotPoses[agentID] <<
       " Vel " << robotVels[agentID] << std::endl);
    }
  }

  std::map<int, std::size_t> Environment::getTrackerIDs()
  {
    return tracker_->getTrackerIDs();
  }

  void Environment::setTrackOtherAgents(bool trackOtherAgents)
  {
    tracker_->setTrackOtherAgents(trackOtherAgents);
  }

  void Environment::setAgentTracker(int TrackerID, std::size_t AgentID)
  {
    tracker_->setAgentTracker(TrackerID, AgentID);
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

  void Environment::loadPlannerInitialGoal()
  {
    switch (initialGoalNo_) {
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

  bool Environment::getVirtualAgentReachedGoal(std::size_t simID, std::size_t agentNo)
  {
    return simvect_[simID]->agents_[agentNo]->reachedGoal_;
  }

  void Environment::setNextGoal()
  {
    newGoal = true;
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

  std::size_t Environment::setSimParam(std::size_t simID)
  {
    simvect_[simID]->setTimeStep(SIM_TIME_STEP);
    simvect_[simID]->setAgentDefaults(NEIGHBOR_DIST, MAX_NEIGHBORS, AGENT_RADIUS, GOAL_RADIUS, PREF_SPEED, MAX_SPEED, 0.0f, MAX_PEOPLE_ACCELERATION, STOP, 0.0f);
  }

  void Environment::doPlannerStep()
  {
    if (planner_->odomNeeded_ && planner_->getAgentType(THIS_ROBOT) != INACTIVE) {WARN(sActorID_<< " using odometry for navigation" << std::endl);}


    if (!TRACK_ROBOTS) {this->updateRobotAgents();}
    if (HRVO_PLANNER)
    {
      planner_->doStep();
      planner_->setOdomNeeded(true);
    }
    else
    {
      DEBUG("Calculating..." << std::endl)
      Vector2 currGoalPos = this->getPlannerGoalPosition(this->getPlannerGoal());
      // Vector2 currGoalPos = Vector2(-6.3, 1.5);
      Vector2 currPos = this->getPlannerAgentPosition(THIS_ROBOT);
      Vector2 relGoal =  currGoalPos - currPos;
        DEBUG("Pos: " << currPos << " Goal: " << currGoalPos 
          << " RelGoal: " << relGoal << std::endl);
      if (newGoal)
      {
        // newPlanner_->sendNewGoal(Vector2(1.0f, 1.0f));
        newPlanner_->sendNewGoal(relGoal);
        newGoal = false;
        reachedGoal=false;
      }
      DEBUG("Using Move Base Planner" << std::endl);
      if (newPlanner_->checkGoalState() == GoalState::SUCCEEDED)
        { ERR("STOPPING!")
          this->stopYoubot();
          reachedGoal=true;}
      planner_->doStep();
    }
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
    if (!HRVO_PLANNER)
    {
      newPlanner_->cancelGoal();
    }
    planner_->setAgentVelocity(THIS_ROBOT, STOP);
  }

  void Environment::emergencyStop()
  {
    if (!HRVO_PLANNER)
    {
      newPlanner_->cancelGoal();
    }
    for (std::size_t i = 0; i < planner_->getNumAgents(); ++i)
    {
      planner_->setAgentVelocity(i, STOP);
    }
  }
}
