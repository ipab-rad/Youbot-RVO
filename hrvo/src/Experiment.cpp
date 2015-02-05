/*
* Experiment.cpp
* Experiment setup code by Alejandro Bordallo, based on HRVO for performing interactive dynamic planning
* Copyright notice included below
*/

#include "Experiment.h"

using namespace hrvo;

int main(int argc, char *argv[])
{
  if (CLEAR_SCREEN) {CLEAR();}
  ros::init(argc, argv, "hrvo_planner");
  ParamInitialise();

  // ************************************************************
  //                      ENVIRONMENT SETUP
  // ************************************************************

  PlannerMapPointer* PlannerMap_ = new PlannerMapPointer();
  ModelMapPointer* ModelMap_ = new ModelMapPointer();


  InitialiseRobots(PlannerMap_);

  SetupLogging(PlannerMap_, ModelMap_);

  std::signal(SIGINT, interrupt_callback);

  // ************************************************************
  //                      ROBOT SETUP
  // ************************************************************

  if (PERFORM_ROBOT_SETUP && !SAFETY_STOP)
  {
    if (ENABLE_PLANNER)
    {
      StopRobots(PlannerMap_);

      for(PlannerMapPointer::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
      {
        Environment* planner = iter->second;
        INFO("Press enter to perform setup for "
          << planner->getStringActorID() << std::endl);
        WaitReturn();

        MoveIntoArea(planner);

        SelectTracker(planner);
      }
    }
    else
      {WARN("Setup of robots skipped as planner is disabled" << std::endl);}
  }

  // ************************************************************
  //                      EXPERIMENT START
  // ************************************************************

  if (!SAFETY_STOP)
  {
    INFO("Press enter to start Experiment");
    WaitReturn();

    STARTED = true;
    INFO("Starting Experiment..." << std::endl);
    ros::Time begin = ros::Time::now();

    LoadInitialGoals(PlannerMap_);
  }

  startTime = (*PlannerMap_)[LOG_PLANNER]->getPlannerGlobalTime();

  while ( ros::ok() && !SAFETY_STOP )
  {
    if (CLEAR_SCREEN) {CLEAR();}
    ros::Rate update_freq(ROS_FREQ);

    SensingUpdate(PlannerMap_);

    PrintAgentState(PlannerMap_);

    PlannerStep(PlannerMap_);

    ModelStep(PlannerMap_, ModelMap_);

    ros::spinOnce();
    update_freq.sleep();
  }
  if (LOG_DATA){dataLog.close();}

  StopRobots(PlannerMap_);
  if (SAFETY_STOP) { EStopRobots(PlannerMap_);}

  return 0;
}

// ************************************************************
//                      EXPERIMENT FUNCTIONS
// ************************************************************

void hrvo::InitialiseRobots(PlannerMapPointer* PlannerMap)
{
  // TODO: Check if this section can be condensed
  if (MEGATRON_ACTIVE)
  {
    std::size_t id = (*PlannerMap).size()+1;
    (*PlannerMap)[id] = new Environment(YOUBOT_1, START_POS1);
    (*PlannerMap)[id]->setPlannerGoalPlan(MEGATRON_PLAN);
    (*PlannerMap)[id]->setPlannerInitialGoal(2);
  }
  if (SOUNDWAVE_ACTIVE)
  {
    std::size_t id = (*PlannerMap).size()+1;
    (*PlannerMap)[id] = new Environment(YOUBOT_2, START_POS2);
    (*PlannerMap)[id]->setPlannerGoalPlan(SOUNDWAVE_PLAN);
    (*PlannerMap)[id]->setPlannerInitialGoal(2);
  }
  if (STARSCREAM_ACTIVE)
  {
    std::size_t id = (*PlannerMap).size()+1;
    (*PlannerMap)[id] = new Environment(YOUBOT_3, START_POS3);
    (*PlannerMap)[id]->setPlannerGoalPlan(STARSCREAM_PLAN);
    (*PlannerMap)[id]->setPlannerInitialGoal(2);
  }
  if (BLACKOUT_ACTIVE)
  {
    std::size_t id = (*PlannerMap).size()+1;
    (*PlannerMap)[id] = new Environment(YOUBOT_4, START_POS4);
    (*PlannerMap)[id]->setPlannerGoalPlan(BLACKOUT_PLAN);
    (*PlannerMap)[id]->setPlannerInitialGoal(2);
  }
  if (THUNDERCRACKER_ACTIVE)
  {
    std::size_t id = (*PlannerMap).size()+1;
    (*PlannerMap)[id] = new Environment(YOUBOT_5, START_POS1);
    (*PlannerMap)[id]->setPlannerGoalPlan(THUNDERCRACKER_PLAN);
    (*PlannerMap)[id]->setPlannerInitialGoal(2);
  }
}

void hrvo::SetupLogging(PlannerMapPointer *PlannerMap, ModelMapPointer *ModelMap)
{
  // Setup logger environment when no youbot is present
  if (!ENABLE_PLANNER)
  {
    for(PlannerMapPointer::iterator iter = (*PlannerMap).begin(); iter != (*PlannerMap).end(); ++iter)
    {
      Environment* planner = iter->second;
      planner->setPlannerPosition(EXIT);
      planner->disablePlannerAgent();
      planner->setTrackOtherAgents(true);
    }
  }
  if (LOG_DATA) { logSetup(dataLog, PlannerMap, ModelMap);}
  INFO("Parameters: TimeStep=" << SIM_TIME_STEP
    << ", NumPlanningAgents=" << (*PlannerMap).size()
    << ", AgentRadius=" << AGENT_RADIUS << std::endl);
}

void hrvo::StopRobots(PlannerMapPointer* PlannerMap)
{
  for(PlannerMapPointer::iterator iter = (*PlannerMap).begin(); iter != (*PlannerMap).end(); ++iter)
  {
    Environment* planner = iter->second;
    for(int i = 0; i < WIFI_ATTEMPTS; ++i)
    {
      planner->stopYoubot();
    }
    WARN("Agent " << planner->getStringActorID()
      << " Stopping" << std::endl);
  }
}

void hrvo::EStopRobots(PlannerMapPointer* PlannerMap)
{
  for(PlannerMapPointer::iterator iter = (*PlannerMap).begin(); iter != (*PlannerMap).end(); ++iter)
  {
    for(int i = 0; i < WIFI_ATTEMPTS; ++i) {
      Environment* planner = iter->second;
      planner->emergencyStop();
    }
  }
  ERR("EMERGENCY STOP!" << std::endl);
  exit(1);
}

void hrvo::MoveIntoArea(Environment* planner)
{
  Vector2 ForwVec = planner->getPlannerAgentPosition(THIS_ROBOT) + goForwVec;
  planner->addAndSetPlannerGoal(ForwVec);
  STARTED = true;
  while ( !planner->getReachedPlannerGoal() && ros::ok() && !SAFETY_STOP )
  {
    if (CLEAR_SCREEN) {CLEAR();}
    ros::Rate update_freq(ROS_FREQ);

    INFO("Moving from " << planner->getPlannerAgentPosition(THIS_ROBOT)
      << " to Position " << ForwVec << std::endl);

    planner->doPlannerStep();

    ros::spinOnce();
    update_freq.sleep();
  }

  planner->stopYoubot();
  STARTED = false;
}

void hrvo::SelectTracker(Environment* planner)
{
  // planner->updateTracker();
  planner->updateLocalisation();
  std::map<int, std::size_t> ids = planner->getTrackerIDs();

  if (ids.empty())
  {
    WARN("No Trackers were found" << std::endl);
  }
  else if (!MANUAL_TRACKER_ASSIGNMENT)
  {
    planner->setAgentTracker(ids[ids.size()-1], THIS_ROBOT);
    INFO("Automatically assigned TrackerID " << ids[0]
      << " for " << planner->getStringActorID() << std::endl);
  }
  else if (MANUAL_TRACKER_ASSIGNMENT)
  {
    INFO("Enter TrackerID for " << planner->getStringActorID()
      << ":" << std::endl);
    int TrackerID = cinInteger();
    planner->setAgentTracker(TrackerID, THIS_ROBOT);
  }

  planner->resetOdomPosition();
  planner->setTrackOtherAgents(true);
}

void hrvo::LoadInitialGoals(PlannerMapPointer *PlannerMap)
{
  for(PlannerMapPointer::iterator iter = (*PlannerMap).begin(); iter != (*PlannerMap).end(); ++iter)
  {
    Environment* planner = iter->second;
    planner->loadPlannerInitialGoal();
  }
}

void hrvo::SensingUpdate(PlannerMapPointer* PlannerMap)
{
  for(PlannerMapPointer::iterator iter = (*PlannerMap).begin(); iter != (*PlannerMap).end(); ++iter)
  {
    Environment* planner = iter->second;
    // planner->updateTracker();
    planner->updateLocalisation();
  }
}

void hrvo::PrintAgentState(PlannerMapPointer* PlannerMap)
{
  for (std::size_t i = 0; i < (*PlannerMap)[LOG_PLANNER]->getNumPlannerAgents(); ++i)
  {
    if ((*PlannerMap)[LOG_PLANNER]->getAgentType(i) == INACTIVE)
      {INFO("Agent" << i << " Inactive"<< std::endl);}
    else
    {
      INFO("Agent" << i << " Pos: ["
        << (*PlannerMap)[LOG_PLANNER]->getPlannerAgentPosition(i)
        << "]" << std::endl);
    }
  }
  //AMCLpointer_->pretty_print();
}

void hrvo::PlannerStep(PlannerMapPointer *PlannerMap)
{
  if (ENABLE_PLANNER)
  {
    for(PlannerMapPointer::iterator iter = (*PlannerMap).begin(); iter != (*PlannerMap).end(); ++iter)
    {
      Environment* planner = iter->second;
      // Set new goal given goal plan
      if (planner->getReachedPlannerGoal())
        {planner->setNextGoal();}

      INFO(planner->getStringActorID() << " to Goal " << planner->getPlannerGoal() << std::endl);
      planner->doPlannerStep();
    }
  }
  else {WARN("Planner is disabled" << std::endl);}
  INFO(std::endl);
}

void hrvo::ModelStep(PlannerMapPointer *PlannerMap, ModelMapPointer *ModelMap)
{
  if (ENABLE_MODELLING)
  {
    bool Logged = false;
    std::map<std::size_t, std::map<std::size_t, Vector2> > possGoals; //EnvID, GoalID, GoalPos
    std::map<std::size_t, std::vector <std::size_t> > modelledAgents; //EnvID, AgentIDs

    for(PlannerMapPointer::iterator iter = (*PlannerMap).begin(); iter != (*PlannerMap).end(); ++iter)
    {
      std::size_t EnvID = iter->first;
      Environment* planner = iter->second;

      possGoals[EnvID][0] = I_g1;
      possGoals[EnvID][1] = I_g2;
      possGoals[EnvID][2] = I_g3;
      // possGoals[EnvID][2] = DynGoalPos;

      // TODO: Start at 0 to model main robot
      for(std::size_t AgentID = 0; AgentID < planner->getNumPlannerAgents(); ++AgentID)
      {
        if (planner->getAgentType(AgentID) != INACTIVE)
        {
          Logged = true;
          if ( (*ModelMap)[EnvID].find(AgentID) == (*ModelMap)[EnvID].end() )
          {
            (*ModelMap)[EnvID][AgentID] = new Model((*PlannerMap)[EnvID]);
          }  // TODO: Check if object is destroyed
          (*ModelMap)[EnvID][AgentID]->setupModel(AgentID, possGoals[EnvID]);
          (*ModelMap)[EnvID][AgentID]->inferGoals(AgentID);
          modelledAgents[EnvID].push_back(AgentID);
        }
      }
    }
    if (LOG_DATA && Logged)
    {
      if (!ENABLE_PLANNER) {(*PlannerMap)[LOG_PLANNER]->doPlannerStep();}
      logData(dataLog, LOG_PLANNER, (*PlannerMap)[LOG_PLANNER]->getPlannerGlobalTime() - startTime,
       modelledAgents[LOG_PLANNER], possGoals[LOG_PLANNER]);
    }
  }
  else {WARN("Model is disabled" << std::endl);}
}
