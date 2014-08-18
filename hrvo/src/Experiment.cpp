/*
* Circle.cpp
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

/**
* \file   Circle.cpp
* \brief  Example with 250 agents navigating through a circular environment.
*/

#ifndef HRVO_HRVO_H_
#include "HRVO.h"
#endif

#ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
#endif

// #define ROS_PUBLISHER 0

#include <cmath>

#include <iostream>
#include <fstream>
#include <csignal>

// #if ROS_PUBLISHER
// #include <ros/ros.h>
// #endif


using namespace hrvo;

const float HRVO_PI = 3.141592654f;

bool SAFETY_STOP = false;
bool STARTED = false;

void interrupt_callback(int s)
{
    if ( !STARTED )
    {
        std::cout << "Aborted!" << std::endl;
        exit(1);
    }
    SAFETY_STOP = true;
}

int main(int argc, char *argv[])
{
    std::cout << std::endl;
    ros::init(argc, argv, "hrvo_planner");
    Environment environment1(YOUBOT_1, START_POS1);
    // Environment environment2(YOUBOT_2, START_POS2);

    std::signal(SIGINT, interrupt_callback);

    //    for (std::size_t i = 0; i < nAgents; ++i) {
    //        const Vector2 position = 200.0f * Vector2(std::cos(0.004f * i * HRVO_TWO_PI), std::sin(0.004f * i * HRVO_TWO_PI));
    //        simulator.addAgent(position, simulator.addGoal(-position));
    //    }
    const Vector2 pos1 = Vector2(-1.5f, 0.0f);
    const Vector2 pos2 = Vector2(1.5f, 0.0f);

    const Vector2 g1 = Vector2(pos2);
    const Vector2 g2 = Vector2(pos1);
    const Vector2 g3 = Vector2(-2.0f, 0.0f);
    const Vector2 I_g1 = Vector2(-6.9f, 1.5f);
    const Vector2 I_g2 = Vector2(-2.7f, 1.5f);
    const Vector2 I_g3 = Vector2(-4.5f, 3.3f);


    std::size_t goal1_1 = environment1.addPlannerGoal(g1);
    // std::size_t goal1_2 = environment1.addPlannerGoal(g2);
    // std::size_t goal2_1 = environment2.addPlannerGoal(g1);
    // std::size_t goal2_2 = environment2.addPlannerGoal(g2);
    environment1.setPlannerGoal(goal1_1);
    // environment2.setPlannerGoal(goal2_2);


    // environment1.addVirtualAgent("youbot_2", pos2, goal1_2);
    // environment1.addPedestrianAgent("pedestrian_1", pos2, goal1_2);
    // environment2.addVirtualAgent("youbot_1", pos1, goal2_1);

    // simulator.setAgentOrientation(0, 0);
    // simulator.setAgentOrientation(1, 0);

    #if HRVO_OUTPUT_TIME_AND_POSITIONS
    std::ofstream log;
    const char *path="log3.csv";
    log.open(path);
    if (log.fail())
        {std::cout << "Writing to log failed!" << std::endl;}
    else
        {std::cout << "Saving log on " << path << std::endl;}

    // log << SIM_TIME_STEP <<","<< simulator.getNumAgents() <<","<< AGENT_RADIUS << std::endl;

    #endif /* HRVO_OUTPUT_TIME_AND_POSITIONS */

    std::cout << "Parameters: TimeStep=" << SIM_TIME_STEP << ", NumAgents=" << environment1.getNumPlannerAgents() << ", AgentRadius=" << AGENT_RADIUS << std::endl;

    #if HRVO_OUTPUT_TIME_AND_POSITIONS
    ROS_INFO("enter to start:");
    while( std::cin.get() != '\n') {;}
    #endif

    STARTED = true;

    bool inferFlag = false;
    std::size_t inferredAgent = 1;
    std::map<std::size_t, std::size_t> simIDs;

    ROS_INFO("starting...");


    ros::Rate update_freq(ROS_FREQ);
    do
    {
        // std::cout << std::endl;
        // #if HRVO_OUTPUT_TIME_AND_POSITIONS
        // log << simulator.getGlobalTime();

        for (std::size_t i = 0; i < environment1.getNumPlannerAgents(); ++i)
        {
            if (environment1.getNumPlannerAgents() > 1) {inferFlag = true;} else {inferFlag = false;}
            // log << "," << simulator.getAgentPosition(i).getX() << "," << simulator.getAgentPosition(i).getY();
            std::cout << "Agent" << i << "Pos: [" << environment1.getAgentPlannerPosition(i) << "]" << std::endl;
        }
        // log << std::endl;
        // #endif /*# HRVO_OUTPUT_TIME_AND_POSITIONS */

        
        // std::map<std::size_t, float> inferredGoals;

        environment1.doPlannerStep();

        if (inferFlag)
        {
        inferredAgent = 1;
        std::map<std::size_t, Vector2> possGoals;
        possGoals[0] = I_g1;
        possGoals[1] = I_g2;
        possGoals[2] = I_g3;
        

        simIDs = environment1.setupModel(inferredAgent, possGoals);
        }


        // environment2.doPlannerStep();

        if (inferFlag)
        {
        std::size_t maxLikelihoodGoal = environment1.inferGoals(inferredAgent, simIDs);
        std::cout << "Agent" << inferredAgent << " is likely going to Goal" << maxLikelihoodGoal << std::endl;
        }

        ros::spinOnce();
        update_freq.sleep();

    }
    while ( !environment1.getReachedPlannerGoal() &&  ros::ok() && !SAFETY_STOP );
    // while ( !simulator.haveReachedGoals() && ros::ok() && !SAFETY_STOP );

    std::cout << "Agents Stopping" << std::endl;
    environment1.emergencyStop();
    // environment2.emergencyStop();

    log.close();

    if ( SAFETY_STOP )
    {
        std::cout << "EMERGENCY STOP" << std::endl;
        exit(1);
    }

    return 0;
}
