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
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <vector>

namespace hrvo {
  class Environment;
  class Model;
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

  // Goal pattern for planning agents
  #define GOAL_STOP 0
  #define GOAL_CYCLE_CW 1
  #define GOAL_CYCLE_CCW 2
  #define GOAL_1_2 3
  #define GOAL_2_3 4
  #define GOAL_3_1 5
  #define FOLLOW_AGENT 6

  // ************************************************************
  //                SETUP EXPERIMENT BEFORE START
  // ************************************************************

  // InSpace Goal positions
  const bool INVERT_X = true;
  const Vector2 I_g1 = Vector2(-6.3f, 1.5f);
  const Vector2 I_g2 = Vector2(-3.07f, 1.5f);
  const Vector2 I_g3 = Vector2(-4.45f, 3.3f);

  // Start positions for InSpace Setup
  const Vector2 STOP = Vector2(0.0f, 0.0f);
  const Vector2 EXIT = Vector2(0.0f, 0.0f);
  const Vector2 START_POS1 = Vector2(-7.5f, 2.0f);
  const Vector2 START_POS2 = Vector2(-7.5f, 2.0f);
  const Vector2 START_POS3 = Vector2(-9.4f, 0.0f);
  const Vector2 START_POS4 = Vector2(-9.4f, 0.5f);

  // Set velocities for the Youbot to advance 2 tiles Backwards/Forwards
  const Vector2 goBackVec = Vector2(-2.0f, 0.0f);
  const Vector2 goForwVec = Vector2(2.0f, 0.0f);

  // Youbot Tracking Offsets (Tracker gives feet position which is innacurate for robots when using 1 kinect)
  const Vector2 kinect1Offset = Vector2(-0.25f, 0.0f);
  const Vector2 kinect2Offset = Vector2(0.25f, 0.0f);
  const Vector2 noOffset = Vector2(0.0f, 0.0f);

  const Vector2 trackerOffset = noOffset;

  // Actor name for the environment created
  enum Actor{
    YOUBOT_1 = 0,
    YOUBOT_2 ,
    YOUBOT_3 ,
    YOUBOT_4 ,
    YOUBOT_5
  };

  void interrupt_callback(int s);

  const char* getActorName(enum Actor actorID);

  std::string intToString(int i);

  int cinInteger();

  // extern std::map<std::size_t, Environment *> *PlannerMapPointer_;
  // extern std::map<std::size_t, std::map<std::size_t, Model *> > *ModelMapPointer_;

  void logSetup(std::ofstream& logfile, std::map<std::size_t, Environment *> *PlannerMap, std::map<std::size_t, std::map<std::size_t, Model*> > *ModelMap);
  void logData(std::ofstream& logfile, int LOG_PLANNER, float currTime, std::vector<size_t> modelledAgents, std::map<std::size_t, Vector2> possGoals);

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
