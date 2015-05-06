/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-04
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Declaration of useful function used by the rest of the code
*/

#ifndef HRVO_DEFINITIONS_H_
#define HRVO_DEFINITIONS_H_

#include "Vector2.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <vector>

namespace hrvo {
class Environment;
class Model;
// A sufficiently small positive float.
const float HRVO_EPSILON = 0.00001f;
const float HRVO_PI = 3.141592653589793f;

// Console printing MACROS
#define ERR(x) std::cerr << "\033[22;31;1m" << x << "\033[0m";    // RED
#define WARN(x) std::cerr << "\033[22;33;1m" << x << "\033[0m";   // YELLOW
#define INFO(x) std::cerr << "\033[22;37;1m" << x << "\033[0m";   // WHITE
#define DEBUG(x) std::cerr << "\033[22;34;1m" << x << "\033[0m";  // BLUE
#define CLEAR() std::cerr << "\x1B[2J\x1B[H";
// CSI[2J clears screen, CSI[H moves the cursor to top-left corner

// Types of agent, determine their sim param, data subscribers and publishers
#define SIMAGENT 0
#define PERSON 1
#define ROBOT 2
#define INACTIVE 3

// Robot is always first agent in it's own simulation
#define THIS_ROBOT 0

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
const Vector2 I_g3 = Vector2(-4.45f, 3.0f);

// Start positions for InSpace Setup
const Vector2 STOP = Vector2(0.0f, 0.0f);
const Vector2 EXIT = Vector2(0.0f, 0.0f);
const Vector2 START_POS1 = Vector2(-8.3f, 1.5f);
const Vector2 START_POS2 = Vector2(-8.3f, 1.5f);
const Vector2 START_POS3 = Vector2(-8.3f, 1.5f);
const Vector2 START_POS4 = Vector2(-8.3f, 1.5f);

// Set velocities for the Youbot to advance 2 tiles Backwards/Forwards
const Vector2 goBackVec = Vector2(-2.0f, 0.0f);
const Vector2 goForwVec = Vector2(2.0f, 0.0f);

// Youbot Tracking Offsets (Tracker gives feet position which
//  is innacurate for robots when using 1 kinect)
const Vector2 kinect1Offset = Vector2(-0.25f, 0.0f);
const Vector2 kinect2Offset = Vector2(0.25f, 0.0f);
const Vector2 noOffset = Vector2(0.0f, 0.0f);

const Vector2 trackerOffset = noOffset;

// Actor name for the environment created
enum Actor {
  MEGATRON = 0,
  SOUNDWAVE ,
  STARSCREAM ,
  BLACKOUT ,
  THUNDERCRACKER ,
  PRIME
};

// enum GoalState{
//   PENDING = 0,
//   ACTIVE,
//   RECALLED,
//   REJECTED,
//   PREEMPTED,
//   ABORTED,
//   SUCCEEDED,
//   LOST
// };

void interrupt_callback(int s);

const char* getActorName(enum Actor actorID);

std::string intToString(int i);

int cinInteger();

// extern std::map<std::size_t, Environment *> *RobotMapPointer_;
// extern std::map<std::size_t, std::map<std::size_t,
//  Model *> > *ModelMapPointer_;

void logSetup(std::ofstream& logfile,
              std::map<std::size_t, Environment *> *RobotMap,
              std::map<std::size_t, std::map<std::size_t, Model*> > *ModelMap);
void logData(std::ofstream& logfile, int LOG_PLANNER, float currTime,
             std::vector<size_t> modelledAgents,
             std::map<std::size_t, Vector2> possGoals);

/**
* \brief      Computes the square of a float.
* \param[in]  scalar  The float to be squared.
* \return     The square of the float.
*/
inline float sqr(float scalar) {
  return scalar * scalar;
}
}  // namespace hrvo

#endif /* HRVO_DEFINITIONS_H_ */
