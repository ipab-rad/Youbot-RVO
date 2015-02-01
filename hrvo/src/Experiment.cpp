/*
* Experiment.cpp
* Experiment setup code by Alejandro Bordallo, based on HRVO for performing interactive dynamic planning
* Copyright notice included below
*\

/*
* HRVO Library
*
* Copyright (c) 2009-2014 University of North Carolina at Chapel Hill.
* All rights reserved.
*
* Permission to use, copy, modify, and distribute this software and its
* documentation for educational, non-commercial research, and non-profit
* purposes, without fee, and without a written agreement is hereby granted,
* provided that the above copyright notice, this paragraph, and the following
* four paragraphs appear in all copies.
*
* Permission to incorporate this software into commercial products may be
* obtained by contacting the authors <geom@cs.unc.edu> or the Office of
* Technology Development at the University of North Carolina at Chapel Hill
* <otd@unc.edu>.
*
* This software program and documentation are copyrighted by the University of
* North Carolina at Chapel Hill. The software program and documentation are
* supplied "as is," without any accompanying services from the University of
* North Carolina at Chapel Hill or the authors. The University of North
* Carolina at Chapel Hill and the authors do not warrant that the operation of
* the program will be uninterrupted or error-free. The end-user understands
* that the program was developed for research purposes and is advised not to
* rely exclusively on the program for any reason.
*
* IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
* AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
* CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
* SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
* CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*
* THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
* DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
* STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
* AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
* AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
* ENHANCEMENTS, OR MODIFICATIONS.
*
* Please send all bug reports to <geom@cs.unc.edu>.
*
* The authors may be contacted via:
*
* Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
* Dept. of Computer Science
* 201 S. Columbia St.
* Frederick P. Brooks, Jr. Computer Science Bldg.
* Chapel Hill, N.C. 27599-3175
* United States of America
*
* <http://gamma.cs.unc.edu/HRVO/>
*/

// #include "HRVO.h"

// #include "Definitions.h"

// #include "Environment.h"

// #include "Model.h"

#include "Experiment.h"

#include <cmath>

#include <fstream>
#include <csignal>

// #include <ros/ros.h>


using namespace hrvo;

// bool SAFETY_STOP = false;
// bool STARTED = false;


int main(int argc, char *argv[])
{
  if (CLEAR_SCREEN) {CLEAR();}
  ros::init(argc, argv, "hrvo_planner");
  ParamInitialise();

  // ************************************************************
  //                      ENVIRONMENT SETUP
  // ************************************************************

  typedef std::map<std::size_t, Environment *> PlannerMapPointer; // EnvID, EnvObject
  typedef std::map<std::size_t, std::map<std::size_t, Model*> > ModelMapPointer; // EnvID, ModelID, ModelObject

  PlannerMapPointer* PlannerMap_ = new PlannerMapPointer();
  ModelMapPointer* ModelMap_ = new ModelMapPointer();

  SetupRobots(PlannerMap_);

  // Setup logger environment when no youbot is present
  if (!ENABLE_PLANNER)
  {
    for(std::map<std::size_t, Environment *>::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
    {
      Environment* planner = iter->second;
      planner->setPlannerPosition(EXIT);
      planner->disablePlannerAgent();
      planner->setTrackOtherAgents(true);
    }
  }

  std::signal(SIGINT, interrupt_callback);

  std::ofstream log;
  if (LOG_DATA)
    { logSetup(log, PlannerMap_, ModelMap_);}

  INFO("Parameters: TimeStep=" << SIM_TIME_STEP << ", NumPlanningAgents=" << (*PlannerMap_).size() << ", AgentRadius=" << AGENT_RADIUS << std::endl);

  std::map<std::size_t, std::size_t> simIDs;

  ros::Rate update_freq(ROS_FREQ);

  // ************************************************************
  //                      ROBOT SETUP
  // ************************************************************

  if (PERFORM_ROBOT_SETUP && !SAFETY_STOP)
  {
    if (ENABLE_PLANNER)
    {
      // Stop robots at start up
      for(std::map<std::size_t, Environment *>::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
      {
        Environment* planner = iter->second;
        for(int i = 0; i < WIFI_ATTEMPTS; ++i) 
        {
          planner->stopYoubot();
        }
      }

      for(std::map<std::size_t, Environment *>::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
      {
        Environment* planner = iter->second;
        INFO("Press enter to perform setup for " << planner->getStringActorID() << std::endl);
        while( std::cin.get() != '\n') {;}

        Vector2 ForwVec = planner->getPlannerAgentPosition(THIS_ROBOT) + goForwVec;
        planner->addAndSetPlannerGoal(ForwVec);

        //  **** MOVE YOUBOT INTO AREA ****
        STARTED = true;
        while ( !planner->getReachedPlannerGoal() && ros::ok() && !SAFETY_STOP )
        {   
          if (CLEAR_SCREEN) {CLEAR();}
          INFO("Moving from " << planner->getPlannerAgentPosition(THIS_ROBOT) << " to Position " << ForwVec << std::endl);  

          planner->doPlannerStep();

          ros::spinOnce();
          update_freq.sleep();
        }

        planner->stopYoubot();
        STARTED = false;

        //  **** ASSIGN TRACKER ****
        planner->updateTracker();
        std::map<int, std::size_t> ids = planner->getTrackerIDs();

        if (ids.empty())
        {
          WARN("No Trackers were found" << std::endl);
        }
        else if (!MANUAL_TRACKER_ASSIGNMENT)
        {
          planner->setAgentTracker(ids[ids.size()-1], THIS_ROBOT);
          INFO("Automatically assigned TrackerID " << ids[0] << " for " << planner->getStringActorID() << std::endl);
        }
        else if (MANUAL_TRACKER_ASSIGNMENT)
        {
          INFO("Enter TrackerID for " << planner->getStringActorID() << ":" << std::endl);
          int TrackerID = cinInteger();
          planner->setAgentTracker(TrackerID, THIS_ROBOT);
        }

        planner->resetOdomPosition();
        planner->setTrackOtherAgents(true);
        // ros::spinOnce();
        // update_freq.sleep();
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
    while( std::cin.get() != '\n') {;}

    STARTED = true;

    INFO("Starting Experiment..." << std::endl);
    ros::Time begin = ros::Time::now();

    for(std::map<std::size_t, Environment *>::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
    {
      Environment* planner = iter->second;
      planner->loadPlannerInitialGoal();
    }
  }

  float startTime = (*PlannerMap_)[LOG_PLANNER]->getPlannerGlobalTime();

  while ( ros::ok() && !SAFETY_STOP )
  {
    if (CLEAR_SCREEN) {CLEAR();}

    //  **** SENSING UPDATE STEP ****
    for(std::map<std::size_t, Environment *>::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
    {
      Environment* planner = iter->second;
      planner->updateTracker();
    }

    for (std::size_t i = 0; i < (*PlannerMap_)[LOG_PLANNER]->getNumPlannerAgents(); ++i)
    {
      if ((*PlannerMap_)[LOG_PLANNER]->getAgentType(i) == INACTIVE)
        {INFO("Agent" << i << " Inactive"<< std::endl);}
      else
      {
        INFO("Agent" << i << " Pos: [" << (*PlannerMap_)[LOG_PLANNER]->getPlannerAgentPosition(i) << "]" << std::endl);
      }
    }

    // Vector2 DynGoalPos = STOP;
    //  **** PLANNER STEP ****
    if (ENABLE_PLANNER)
    {
      for(std::map<std::size_t, Environment *>::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
      {
        Environment* planner = iter->second;
        // planner->editPlannerGoal(3, planner->getPlannerAgentPosition(planner->getNumPlannerAgents()-1));
        // DynGoalPos = planner->getPlannerAgentPosition(planner->getNumPlannerAgents()-1);
        
        // Change so that when reached goals, check planner goal plan, and change goals elsewhere
        if (planner->getReachedPlannerGoal())
          {planner->setNextGoal();}
        // if (planner->getReachedPlannerGoal() && CYCLE_GOALS)
        //   {planner->cycleGoalsCounterClockwise();}

        INFO(planner->getStringActorID() << " to Goal " << planner->getPlannerGoal() << std::endl);
        // if (planner->getReachedPlannerGoal() && !CYCLE_GOALS)
        //   {planner->stopYoubot();}
        planner->doPlannerStep();
      }
    }
    else {WARN("Planner is disabled" << std::endl);}
    INFO(std::endl);


    //  **** MODEL STEP ****
    if (ENABLE_MODELLING)
    {
      bool Logged = false;
      std::map<std::size_t, std::map<std::size_t, Vector2> > possGoals;
      // std::map<std::size_t, Vector2> possGoals;
      std::map<std::size_t, std::vector <std::size_t> > modelledAgents;

      for(std::map<std::size_t, Environment *>::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
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
            if ( (*ModelMap_)[EnvID].find(AgentID) == (*ModelMap_)[EnvID].end() )
            {(*ModelMap_)[EnvID][AgentID] = new Model((*PlannerMap_)[EnvID]);}  // TODO: Check if object is destroyed
            (*ModelMap_)[EnvID][AgentID]->setupModel(AgentID, possGoals[EnvID]);
            (*ModelMap_)[EnvID][AgentID]->inferGoals(AgentID);
            modelledAgents[EnvID].push_back(AgentID);
          }
        }
      }
      if (LOG_DATA && Logged) 
      { 
        if (!ENABLE_PLANNER) {(*PlannerMap_)[LOG_PLANNER]->doPlannerStep();}
        logData(log, LOG_PLANNER, (*PlannerMap_)[LOG_PLANNER]->getPlannerGlobalTime() - startTime, modelledAgents[LOG_PLANNER], possGoals[LOG_PLANNER]);
      }
    }

    ros::spinOnce();
    update_freq.sleep();

  }
  // while ( !environment1.getReachedPlannerGoal() &&  ros::ok() && !SAFETY_STOP );
  // while ( !simulator.haveReachedGoals() && ros::ok() && !SAFETY_STOP );

  
  for(std::map<std::size_t, Environment *>::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
  {
    Environment* planner = iter->second;
    for(int i = 0; i < WIFI_ATTEMPTS; ++i) 
    {
      planner->stopYoubot();
    }
    WARN("Agent " << planner->getStringActorID() << " Stopping" << std::endl);
  }

  if (LOG_DATA){log.close();}

  if ( SAFETY_STOP )
  {
    for(std::map<std::size_t, Environment *>::iterator iter = (*PlannerMap_).begin(); iter != (*PlannerMap_).end(); ++iter)
    {
      for(int i = 0; i < WIFI_ATTEMPTS; ++i) {
        Environment* planner = iter->second;
        planner->emergencyStop();
      }
    }
    ERR("EMERGENCY STOP!" << std::endl);
    exit(1);
  }
  return 0;
}

void hrvo::SetupRobots(PlannerMapPointer* PlannerMap)
{
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
