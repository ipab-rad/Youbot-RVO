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

#ifndef HRVO_HRVO_H_
#include "HRVO.h"
#endif

#ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
#endif


#include <cmath>

#include <iostream>
#include <fstream>
#include <csignal>

// #if ROS_PUBLISHER
// #include <ros/ros.h>
// #endif


using namespace hrvo;


bool SAFETY_STOP = false;
bool STARTED = false;

void interrupt_callback(int s)
{
    if ( !STARTED )
    {
        WARN("Aborted!" << std::endl);
        exit(1);
    }
    SAFETY_STOP = true;
}

int main(int argc, char *argv[])
{
    INFO(std::endl);
    ros::init(argc, argv, "hrvo_planner");

    // ************************************************************
    //                      ENVIRONMENT SETUP
    // ************************************************************

    std::map<std::size_t, Environment *> PlannerMap_;

    Environment environment1(YOUBOT_1, START_POS1);
    PlannerMap_[1] = &environment1;
    Environment environment2(YOUBOT_2, START_POS2);
    PlannerMap_[2] = &environment2;

    std::size_t LogPlanner = 1;

    std::signal(SIGINT, interrupt_callback);

    std::ofstream log;
    if(LOG_DATA)
    {   
        const char *path="PersonWalk4.csv";
        log.open(path);
        if (log.fail())
            {ERR("Writing to log failed!" << std::endl);}
        else
            {DEBUG("Saving log on " << path << std::endl);}

        log << SIM_TIME_STEP <<","<< PlannerMap_.size() <<","<< AGENT_RADIUS << std::endl;
    }
    
    INFO("Parameters: TimeStep=" << SIM_TIME_STEP << ", NumPlanningAgents=" << PlannerMap_.size() << ", AgentRadius=" << AGENT_RADIUS << std::endl);



    bool inferFlag = false; // KEEP FALSE, disables model inference when planning agent is alone
    std::size_t inferredAgent = 1;
    std::map<std::size_t, std::size_t> simIDs;

    ros::Rate update_freq(ROS_FREQ);

    // ************************************************************
    //                      ROBOT SETUP
    // ************************************************************

    if (PERFORM_ROBOT_SETUP){

        for(std::map<std::size_t, Environment *>::iterator iter = PlannerMap_.begin(); iter != PlannerMap_.end(); ++iter)
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
                CLEAR();
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
            else if (!MANUAL_TRACKER_ASSIGNMENT && ids.size() == 1)
            {
                planner->setAgentTracker(ids[0], THIS_ROBOT);
                INFO("Automatically assigned TrackerID " << ids[0] << " for " << planner->getStringActorID() << std::endl);
            }
            else if (MANUAL_TRACKER_ASSIGNMENT)
            {
                INFO("Enter TrackerID for " << planner->getStringActorID() << ":" << std::endl);
                int TrackerID = cinInteger();
                planner->setAgentTracker(TrackerID, THIS_ROBOT);
            }

            planner->setTrackOtherAgents(true);
        }

    }

    // ************************************************************
    //                      EXPERIMENT START
    // ************************************************************

    INFO("Press enter to start Experiment");
    while( std::cin.get() != '\n') {;}

    STARTED = true;

    INFO("Starting Experiment..." << std::endl);
    ros::Time begin = ros::Time::now();
    environment1.setPlannerInitialGoal(1);
    environment2.setPlannerInitialGoal(3);

    while ( ros::ok() && !SAFETY_STOP )
    {
        CLEAR();
        if (LOG_DATA){log << ros::Time::now() - begin;}

        //  **** SENSING UPDATE STEP ****
        for(std::map<std::size_t, Environment *>::iterator iter = PlannerMap_.begin(); iter != PlannerMap_.end(); ++iter)
        {
            Environment* planner = iter->second;
            planner->updateTracker();
        }

        for (std::size_t i = 0; i < PlannerMap_[LogPlanner]->getNumPlannerAgents(); ++i)
        {
            INFO("Agent" << i << " Pos: [" << PlannerMap_[LogPlanner]->getPlannerAgentPosition(i) << "]" << std::endl);
            if (LOG_DATA){log << "," << PlannerMap_[LogPlanner]->getPlannerAgentPosition(i).getX() << "," << PlannerMap_[LogPlanner]->getPlannerAgentPosition(i).getY();}
        }
        if (LOG_DATA){log << std::endl;}
        
        //  **** PLANNER STEP ****
        for(std::map<std::size_t, Environment *>::iterator iter = PlannerMap_.begin(); iter != PlannerMap_.end(); ++iter)
        {
            Environment* planner = iter->second;

            //  **** PLANNER STEP ****
            if (planner->getReachedPlannerGoal() && CYCLE_GOALS)
                {planner->cycleGoalsCounterClockwise();}

            INFO(planner->getStringActorID() << " Goal:" << planner->getPlannerGoal() << std::endl);
            if (planner->getReachedPlannerGoal() && !CYCLE_GOALS)
                {planner->stopYoubot();}
            else
                {planner->doPlannerStep();}
        }

        //  **** MODEL STEP ****
        if (ENABLE_MODELLING)
        {
            // TODO: REPLACE WITH SPECIFIC INFERRING AGENT CHECK
            if (environment1.getNumPlannerAgents() > 1) {inferFlag = true;} else {inferFlag = false;}
        }
        if (inferFlag)
        {
            std::map<std::size_t, Vector2> possGoals;
            possGoals[0] = I_g1;
            possGoals[1] = I_g2;
            possGoals[2] = I_g3;

            simIDs = environment1.setupModel(inferredAgent, possGoals);
        
            std::size_t maxLikelihoodGoal = environment1.inferGoals(inferredAgent, simIDs);
            INFO("Agent" << inferredAgent << " is likely going to Goal" << maxLikelihoodGoal << std::endl);
            INFO(std::endl);
            if (LOG_DATA){
                log << "," << environment1.getPlannerAgentPosition(inferredAgent).getX() << "," << environment1.getPlannerAgentPosition(inferredAgent).getY();
                log << "," << environment1.getPlannerAgentVelocity(inferredAgent).getX() << "," << environment1.getPlannerAgentVelocity(inferredAgent).getY();
                log << "," << environment1.getGoalRatio(0) << "," << environment1.getGoalRatio(1) << "," << environment1.getGoalRatio(2);
            
                for (std::size_t i = 1; i < environment1.getNumPlannerAgents(); ++i)
                {
                    log << "," << environment1.getPlannerAgentPosition(i).getX() << "," << environment1.getPlannerAgentPosition(i).getY();
                    log << "," << environment1.getPlannerAgentVelocity(i).getX() << "," << environment1.getPlannerAgentVelocity(i).getY();
                }
            }
        }
        if (LOG_DATA){log << std::endl;}

        ros::spinOnce();
        update_freq.sleep();

    }
    // while ( !environment1.getReachedPlannerGoal() &&  ros::ok() && !SAFETY_STOP );
    // while ( !simulator.haveReachedGoals() && ros::ok() && !SAFETY_STOP );

    WARN("Agents Stopping" << std::endl);
    for(std::map<std::size_t, Environment *>::iterator iter = PlannerMap_.begin(); iter != PlannerMap_.end(); ++iter)
    {
        Environment* planner = iter->second;
        planner->stopYoubot();
    }

    if (LOG_DATA){log.close();}

    if ( SAFETY_STOP )
    {
        for(std::map<std::size_t, Environment *>::iterator iter = PlannerMap_.begin(); iter != PlannerMap_.end(); ++iter)
        {
            Environment* planner = iter->second;
            planner->emergencyStop();
            ERR("EMERGENCY STOP!" << std::endl);
        }
        exit(1);
    }

    return 0;
}
