/*
* Definitions.h
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
* \file   Definitions.h
* \brief  Declares and defines internal functions.
*/

#ifndef HRVO_DEFINITIONS_H_
#define HRVO_DEFINITIONS_H_

#ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
#endif

#include <iostream>
#include <string>
#include <sstream>

namespace hrvo {
  //A sufficiently small positive float.
  const float HRVO_EPSILON = 0.00001f;
  const float HRVO_PI = 3.141592653589793f;

  // Console printing MACROS
  #define ERR(x) std::cerr << "\033[22;31;1m" << x << "\033[0m";  // RED
  #define WARN(x) std::cerr << "\033[22;33;1m" << x << "\033[0m"; // YELLOW
  #define INFO(x) std::cerr << "\033[22;37;1m" << x << "\033[0m"; // WHITE
  #define DEBUG(x) std::cerr << "\033[22;34;1m" << x << "\033[0m";// BLUE
  #define CLEAR() std::cerr << "\x1B[2J\x1B[H"; // CSI[2J clears screen, CSI[H moves the cursor to top-left corner

  // Types of agent, determine their sim param, data subscribers and publishers
  #define SIMAGENT 0
  #define PERSON 1
  #define ROBOT 2
  #define INACTIVE 3

  // ************************************************************
  //                SETUP EXPERIMENT BEFORE START
  // ************************************************************
  // Experimental setup parameters
  const bool CYCLE_GOALS = true;                  // Make Planning robot cycle between goals
  const bool PERFORM_ROBOT_SETUP = true;          // Robots move into initial positions
  const bool MANUAL_TRACKER_ASSIGNMENT = false;   // False = Automatic setup will assign last TrackerID
  const bool ONLY_ODOMETRY = false;               // Use only odometry for robots, no tracker feedback
  const bool ENABLE_MODELLING = false;            // Enable inference model
  const bool LOG_DATA = false;                    // Log data into a file
  const bool ASSIGN_TRACKER_WHEN_ALONE = false;   // When only one agent is tracked, assign tracker to robot
  const int ROS_FREQ = 10;
  const std::size_t MAX_NO_TRACKED_AGENTS = 2;    // TODO: Not working as intended

  // Inspace Workspace limits
  const bool LIMIT_WORKSPACE_VEL = true;
  const float MAX_Y = 3.5f;
  const float MIN_Y = -0.0f;
  const float MAX_X = -2.0f;
  const float MIN_X = -8.0f;

  // Simulation setup parameters
  const std::size_t THIS_ROBOT = 0;
  const float SIM_TIME_STEP = 0.1f;

  // Model setup parameters
  const float GOAL_SUM_PRIOR = 0.1f;             // Goal inference initial prior

  // Goal positions for InSpace Setup
  const Vector2 I_g1 = Vector2(-6.3f, 1.5f);
  const Vector2 I_g2 = Vector2(-3.07f, 1.5f);
  const Vector2 I_g3 = Vector2(-4.45f, 3.3f);

  // Start positions for InSpace Setup
  const Vector2 STOP = Vector2(0.0f, 0.0f);
  const Vector2 START_POS1 = Vector2(-7.4f, 1.5f);
  const Vector2 START_POS2 = Vector2(-7.4f, 2.5f);

  // Set velocities for the Youbot to advance 2 tiles Backwards/Forwards
  const Vector2 goBackVec = Vector2(-1.2f, 0.0f);
  const Vector2 goForwVec = Vector2(1.2f, 0.0f);

  // Youbot Tracking Offsets (Tracker gives feet position which is innacurate for robots when using 1 kinect)
  const Vector2 kinect1Offset = Vector2(-0.25f, 0.0f);
  const Vector2 kinect2Offset = Vector2(0.25f, 0.0f);
  const Vector2 noOffset = Vector2(0.0f, 0.0f);

  const Vector2 trackerOffset = kinect1Offset;

 
  const std::size_t VELOCITY_AVERAGE_WINDOW = 10;
  const std::size_t GOAL_INFERENCE_HISTORY = 10;


  const float NEIGHBOR_DIST = 5.0f;
  const std::size_t MAX_NEIGHBORS = 10;
  const float AGENT_RADIUS = 0.5f;
  const float GOAL_RADIUS = 0.25f;
  const float PREF_SPEED = 0.3f;
  const float PREF_PEOPLE_SPEED = 0.3f;
  const float MAX_SPEED = 0.6f;
  const float MAX_PEOPLE_SPEED = 2.0f;
  const float MAX_ACCELERATION = 1.2f;
  const float MAX_PEOPLE_ACCELERATION = 5.0f;

  // Actor name for the environment created
  enum Actor{
    YOUBOT_1 = 0,
    YOUBOT_2 ,
    YOUBOT_3 ,
    YOUBOT_4 ,
    YOUBOT_5
  };

  const char* getActorName(enum Actor actorID);

  std::string intToString(int i);

  int cinInteger();

  /**
  * \brief      Computes the square of a float.
  * \param[in]  scalar  The float to be squared.
  * \return     The square of the float.
  */
  inline float sqr(float scalar)
  {
    return scalar * scalar;
  }
}

#endif /* HRVO_DEFINITIONS_H_ */
