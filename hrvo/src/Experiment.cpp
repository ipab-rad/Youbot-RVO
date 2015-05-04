/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-04
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Main experiment code for multi-robot/human interactive navigation
*/

#include "Experiment.h"
#include "AMCLWrapper.h"

using namespace hrvo;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hrvo_planner");
  ParamInitialise();
  if (CLEAR_SCREEN) {CLEAR();}
  if (!LOADED_PARAM) {
    ERR("*************************************************" << std::endl <<
        "DEFAULT PARAMETERS LOADED, did you use roslaunch?" << std::endl <<
        "*************************************************" << std::endl);
  }

  // ************************************************************
  //                      ENVIRONMENT SETUP
  // ************************************************************

  RobotMapPointer* RobotMap_ = new RobotMapPointer();
  ModelMapPointer* ModelMap_ = new ModelMapPointer();


  InitialiseRobots(RobotMap_);

  SetupLogging(RobotMap_, ModelMap_);

  std::signal(SIGINT, interrupt_callback);

  // ************************************************************
  //                      ROBOT SETUP
  // ************************************************************

  if (PERFORM_ROBOT_SETUP && !SAFETY_STOP) {
    if (ENABLE_PLANNER) {
      StopRobots(RobotMap_);

      // if (!TRACK_ROBOTS) {InitRobotPoses(RobotMap_);}

      for (RobotMapPointer::iterator iter = (*RobotMap_).begin();
           iter != (*RobotMap_).end(); ++iter) {
        Environment* planner = iter->second;
        INFO("Press enter to perform setup for "
             << planner->getStringActorID() << std::endl);
        WaitReturn();

        MoveIntoArea(planner);

        SelectTracker(planner);

        MoveToInitialGoal(planner);
      }
    } else {
      WARN("Setup of robots skipped as planner is disabled" << std::endl);
    }
  }

  // ************************************************************
  //                      EXPERIMENT START
  // ************************************************************

  if (!SAFETY_STOP) {
    INFO("Press enter to start Experiment");
    WaitReturn();

    STARTED = true;
    INFO("Starting Experiment..." << std::endl);
    ros::Time begin = ros::Time::now();
  }

  startTime = (*RobotMap_)[LOG_PLANNER]->getPlannerGlobalTime();

  while ( ros::ok() && !SAFETY_STOP ) {
    if (CLEAR_SCREEN) {CLEAR();}
    ros::Rate update_freq(ROS_FREQ);

    SensingUpdate(RobotMap_);

    PrintAgentState(RobotMap_);

    PlannerStep(RobotMap_);

    ModelStep(RobotMap_, ModelMap_);

    ros::spinOnce();
    update_freq.sleep();
  }
  if (LOG_DATA) {
    DEBUG("LOG CLOSED!!" << std::endl);
    dataLog.close();
  }

  StopRobots(RobotMap_);
  if (SAFETY_STOP) { EStopRobots(RobotMap_);}
  ros::param::del("/youbot_experiment/loadedParam");

  return 0;
}

// ************************************************************
//                      EXPERIMENT FUNCTIONS
// ************************************************************

void hrvo::InitialiseRobots(RobotMapPointer* RobotMap) {
  // TODO(Alex): Check if this section can be condensed
  if (MEGATRON_ACTIVE) {
    std::size_t id = (*RobotMap).size() + 1;
    (*RobotMap)[id] = new Environment(MEGATRON, START_POS1);
    (*RobotMap)[id]->setPlannerGoalPlan(MEGATRON_PLAN);
    (*RobotMap)[id]->setPlannerInitialGoal(MEGATRON_GOAL);
  }
  if (SOUNDWAVE_ACTIVE) {
    std::size_t id = (*RobotMap).size() + 1;
    (*RobotMap)[id] = new Environment(SOUNDWAVE, START_POS2);
    (*RobotMap)[id]->setPlannerGoalPlan(SOUNDWAVE_PLAN);
    (*RobotMap)[id]->setPlannerInitialGoal(SOUNDWAVE_PLAN);
  }
  if (STARSCREAM_ACTIVE) {
    std::size_t id = (*RobotMap).size() + 1;
    (*RobotMap)[id] = new Environment(STARSCREAM, START_POS3);
    (*RobotMap)[id]->setPlannerGoalPlan(STARSCREAM_PLAN);
    (*RobotMap)[id]->setPlannerInitialGoal(STARSCREAM_GOAL);
  }
  if (BLACKOUT_ACTIVE) {
    std::size_t id = (*RobotMap).size() + 1;
    (*RobotMap)[id] = new Environment(BLACKOUT, START_POS4);
    (*RobotMap)[id]->setPlannerGoalPlan(BLACKOUT_PLAN);
    (*RobotMap)[id]->setPlannerInitialGoal(BLACKOUT_GOAL);
  }
  if (THUNDERCRACKER_ACTIVE) {
    std::size_t id = (*RobotMap).size() + 1;
    (*RobotMap)[id] = new Environment(THUNDERCRACKER, START_POS1);
    (*RobotMap)[id]->setPlannerGoalPlan(THUNDERCRACKER_PLAN);
    (*RobotMap)[id]->setPlannerInitialGoal(THUNDERCRACKER_GOAL);
  }
  if (PRIME_ACTIVE) {
    std::size_t id = (*RobotMap).size() + 1;
    (*RobotMap)[id] = new Environment(PRIME, START_POS1);
    (*RobotMap)[id]->setPlannerGoalPlan(PRIME_PLAN);
    (*RobotMap)[id]->setPlannerInitialGoal(PRIME_GOAL);
  }
}

void hrvo::SetupLogging(RobotMapPointer *RobotMap,
                        ModelMapPointer *ModelMap) {
  // Setup logger environment when no youbot is present
  if (!ENABLE_PLANNER) {
    for (RobotMapPointer::iterator iter = (*RobotMap).begin();
         iter != (*RobotMap).end(); ++iter) {
      Environment* planner = iter->second;
      planner->setPlannerPosition(EXIT);
      planner->disablePlannerAgent();
      planner->setTrackOtherAgents(true);
    }
  }
  if (LOG_DATA) { logSetup(dataLog, RobotMap, ModelMap);}
  INFO("Parameters: TimeStep=" << SIM_TIME_STEP <<
       ", NumPlanningAgents=" << (*RobotMap).size() <<
       ", AgentRadius=" << AGENT_RADIUS << std::endl);
}

void hrvo::StopRobots(RobotMapPointer* RobotMap) {
  for (RobotMapPointer::iterator iter = (*RobotMap).begin();
       iter != (*RobotMap).end(); ++iter) {
    Environment* planner = iter->second;
    for (int i = 0; i < WIFI_ATTEMPTS; ++i) {
      planner->stopYoubot();
    }
    WARN("Agent " << planner->getStringActorID()
         << " Stopping" << std::endl);
  }
}

void hrvo::EStopRobots(RobotMapPointer* RobotMap) {
  for (RobotMapPointer::iterator iter = (*RobotMap).begin();
       iter != (*RobotMap).end(); ++iter) {
    for (int i = 0; i < WIFI_ATTEMPTS; ++i) {
      Environment* planner = iter->second;
      planner->emergencyStop();
    }
  }
  ERR("EMERGENCY STOP!" << std::endl);
  exit(1);
}

void hrvo::InitRobotPoses(RobotMapPointer* RobotMap) {
  for (RobotMapPointer::iterator iter = (*RobotMap).begin();
       iter != (*RobotMap).end(); ++iter) {
    Environment* planner = iter->second;
    // planner->doPlannerStep();
    planner->updateLocalisation(false);
  }
}

void hrvo::MoveIntoArea(Environment* planner) {
  // Vector2 ForwVec = planner->getPlannerAgentPosition(THIS_ROBOT) + goForwVec;
  // planner->addAndSetPlannerGoal(ForwVec);
  std::size_t initGoal = 1;
  planner->setPlannerGoal(initGoal);
  STARTED = true;
  while ( !planner->getReachedPlannerGoal() && ros::ok() && !SAFETY_STOP ) {
    if (CLEAR_SCREEN) { CLEAR(); }
    ros::Rate update_freq(ROS_FREQ);
    planner->updateLocalisation(false);
    INFO("Moving from " << planner->getPlannerAgentPosition(THIS_ROBOT) <<
         " to Position " << planner->getPlannerGoalPosition(initGoal) <<
         std::endl);

    planner->doPlannerStep();

    ros::spinOnce();
    update_freq.sleep();
  }

  planner->stopYoubot();
  STARTED = false;
}

void hrvo::SelectTracker(Environment* planner) {
  if (TRACK_ROBOTS) {
    planner->updateLocalisation(true);
    std::map<int, std::size_t> ids = planner->getTrackerIDs();

    if (ids.empty()) {
      WARN("No Trackers were found" << std::endl);
    } else if (!MANUAL_TRACKER_ASSIGNMENT) {
      planner->setAgentTracker(ids[ids.size() - 1], THIS_ROBOT);
      INFO("Automatically assigned TrackerID " << ids[0]
           << " for " << planner->getStringActorID() << std::endl);
    } else if (MANUAL_TRACKER_ASSIGNMENT) {
      INFO("Enter TrackerID for " << planner->getStringActorID()
           << ":" << std::endl);
      int TrackerID = cinInteger();
      planner->setAgentTracker(TrackerID, THIS_ROBOT);
    }
  } else {
    WARN("Robots are NOT tracked by cameras, using AMCL/odom" << std::endl);
  }

  planner->resetOdomPosition();
  planner->setTrackOtherAgents(true);
}

void hrvo::MoveToInitialGoal(Environment* planner) {
  planner->loadPlannerInitialGoal();
  bool arrived = false;
  INFO("Press enter to navigate to initial goal "
       << planner->getPlannerInitalGoal() << std::endl);
  WaitReturn();
  STARTED = true;
  while ( ros::ok() && !SAFETY_STOP && !arrived ) {
    if (CLEAR_SCREEN) {CLEAR();}
    ros::Rate update_freq(ROS_FREQ);

    planner->updateLocalisation(true);

    INFO(planner->getStringActorID() << " Pos: ["
         << planner->getPlannerAgentPosition(THIS_ROBOT)
         << "] to Initial Goal " << planner->getPlannerGoal() << std::endl);
    planner->doPlannerStep();

    if (planner->getReachedPlannerGoal())
    {arrived = true;}

    ros::spinOnce();
    update_freq.sleep();
  }
  planner->stopYoubot();
  STARTED = false;
}

void hrvo::SensingUpdate(RobotMapPointer* RobotMap) {
  for (RobotMapPointer::iterator iter = (*RobotMap).begin();
       iter != (*RobotMap).end(); ++iter) {
    Environment* planner = iter->second;
    planner->updateLocalisation(true);
  }
}

void hrvo::PrintAgentState(RobotMapPointer* RobotMap) {
  for (std::size_t i = 0;
       i < (*RobotMap)[LOG_PLANNER]->getNumPlannerAgents(); ++i) {
    if ((*RobotMap)[LOG_PLANNER]->getAgentType(i) == INACTIVE) {
      INFO("Agent" << i << " Inactive" << std::endl);
    } else {
      INFO("Agent" << i << " Pos: ["
           << (*RobotMap)[LOG_PLANNER]->getPlannerAgentPosition(i)
           << "]" << std::endl);
    }
  }
}

void hrvo::PlannerStep(RobotMapPointer *RobotMap) {
  if (ENABLE_PLANNER) {
    for (RobotMapPointer::iterator iter = (*RobotMap).begin();
         iter != (*RobotMap).end(); ++iter) {
      Environment* planner = iter->second;
      // Set new goal given goal plan
      ERR("CURRENT GOAL: " << planner->getPlannerGoal() << std::endl);
      if (planner->getReachedPlannerGoal()) {
        DEBUG("NEXT GOAL1" << std::endl);
        planner->setNextGoal();
      }
      // else if (planner->reachedMoveGoal())
      //   { DEBUG("NEXT GOAL2" << std::endl);
      //     planner->setNextGoal();}

      INFO(planner->getStringActorID() << " to Goal " <<
           planner->getPlannerGoal() << std::endl);
      planner->doPlannerStep();
    }
  } else { WARN("Planner is disabled" << std::endl); }
  INFO(std::endl);
}

void hrvo::ModelStep(RobotMapPointer *RobotMap, ModelMapPointer *ModelMap) {
  if (ENABLE_MODELLING) {
    bool Logged = false;
    // EnvID, GoalID, GoalPos
    std::map<std::size_t, std::map<std::size_t, Vector2> > possGoals;
    // EnvID, AgentIDs
    std::map<std::size_t, std::vector <std::size_t> > modelledAgents;

    for (RobotMapPointer::iterator iter = (*RobotMap).begin();
         iter != (*RobotMap).end(); ++iter) {
      std::size_t EnvID = iter->first;
      Environment* planner = iter->second;

      // Goal Sampling
      if (GOAL_SAMPLING) {
        for (int x = 0; x < 10; ++x) {
          float xval = x;
          for (int y = 0; y < 10; ++y) {
            float yval = y;
            possGoals[EnvID][(x * 10) + y] = Vector2(-xval / 1, yval / 1);
          }
        }
      } else {
        possGoals[EnvID][0] = I_g1;
        possGoals[EnvID][1] = I_g2;
        possGoals[EnvID][2] = I_g3;
      }
      // possGoals[EnvID][2] = DynGoalPos;

      DEBUG("NumAgents=" << planner->getNumPlannerAgents() <<
            " Own:" << MODEL_OWN_ROBOT << std::endl);
      // TODO(Alex): Start at 0 to model main robot
      std::size_t firstModelledAgent = 1;
      if (MODEL_OWN_ROBOT) {firstModelledAgent = 0;}
      for (std::size_t AgentID = firstModelledAgent;
           AgentID < planner->getNumPlannerAgents(); ++AgentID) {
        if (planner->getAgentType(AgentID) != INACTIVE) {
          Logged = true;
          if ( (*ModelMap)[EnvID].find(AgentID) == (*ModelMap)[EnvID].end() ) {
            (*ModelMap)[EnvID][AgentID] = new Model((*RobotMap)[EnvID]);
          }  // TODO(Alex): Check if object is destroyed
          (*ModelMap)[EnvID][AgentID]->setupModel(AgentID, possGoals[EnvID]);
          (*ModelMap)[EnvID][AgentID]->inferGoals(AgentID);
          modelledAgents[EnvID].push_back(AgentID);
        }
      }
    }
    if (LOG_DATA && Logged) {
      if (!ENABLE_PLANNER) {(*RobotMap)[LOG_PLANNER]->doPlannerStep();}
      DEBUG("LOGGING!!" << std::endl);
      logData(dataLog, LOG_PLANNER,
              (*RobotMap)[LOG_PLANNER]->getPlannerGlobalTime() - startTime,
              modelledAgents[LOG_PLANNER], possGoals[LOG_PLANNER]);
    }
  } else { WARN("Model is disabled" << std::endl); }
}
