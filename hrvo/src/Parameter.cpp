/**
* Created by Alejandro Bordallo
* \file   Parameter.cpp
* \brief  Contains all global parameters shared between the classes.
*         These are read once at the beginning of execution from the ROS launch file.
*/

#include "Parameter.h"

// Robot Setup
bool MEGATRON_ACTIVE;
int MEGATRON_PLAN;
int MEGATRON_GOAL;
bool SOUNDWAVE_ACTIVE;
int SOUNDWAVE_PLAN;
int SOUNDWAVE_GOAL;
bool STARSCREAM_ACTIVE;
int STARSCREAM_PLAN;
int STARSCREAM_GOAL;
bool BLACKOUT_ACTIVE;
int BLACKOUT_PLAN;
int BLACKOUT_GOAL;
bool THUNDERCRACKER_ACTIVE;
int THUNDERCRACKER_PLAN;
int THUNDERCRACKER_GOAL;
bool PRIME_ACTIVE;
int PRIME_PLAN;
int PRIME_GOAL;

// Experiment Setup

// Enables robot planner, disable when only inferring / tracking
bool ENABLE_PLANNER;

bool HRVO_PLANNER;

// Robots move into initial positions
bool PERFORM_ROBOT_SETUP;

// False = Automatic setup will assign last TrackerID
bool MANUAL_TRACKER_ASSIGNMENT;

// Use only odometry for robots, no tracker feedback
bool ONLY_ODOMETRY;

// Whether AMCL is running on the robot, requires laser scanner
bool IS_AMCL_ACTIVE;

// Whether the tracker is setup to track robots as well as people
bool TRACK_ROBOTS;

bool MEGATRON_BUMPER;
bool SOUNDWAVE_BUMPER;
bool STARSCREAM_BUMPER;
bool BLACKOUT_BUMPER;
bool THUNDERCRACKER_BUMPER;

// Enable inference model
bool ENABLE_MODELLING;

// Model the planning robot as well (Start from Agent 0)
bool MODEL_OWN_ROBOT;

// Log data into a file
bool LOG_DATA;

// When only one agent is tracked, assign tracker to robot
bool ASSIGN_TRACKER_WHEN_ALONE;

// How many iterations after tracker of another agent is reassigned to robot
int TRACKER_ODOM_COMPARISONS;

// Planner frequency Hz
int ROS_FREQ;

// Clearing makes it prettier but fits less on the screen
bool CLEAR_SCREEN;

bool DISPLAY_INTENTION;

// Displays curr vs sim Vels and goal inference vs sum values
bool DISPLAY_INFERENCE_VALUES;

// Display pos, vel and goal for all simulated agents.
bool DISPLAY_SIM_AGENTS;

// TODO(Alex): Not working as intended
int MAX_NO_TRACKED_AGENTS;

int WIFI_ATTEMPTS;

// Logger Parameters
std::string LOG_NAME;
int LOG_PLANNER;

// Simulation Constants (DO NOT CHANGE)
int THIS_ROBOT = 0;
float SIM_TIME_STEP;

// Model Parameters

// Enable sampling of multiple goals over discretised space
bool GOAL_SAMPLING;
bool BIVARIATE;

// Goal inference initial prior
float GOAL_SUM_PRIOR;

// Discount of previous likelihood history
float GOAL_HISTORY_DISCOUNT;
int GOAL_INFERENCE_HISTORY;
int VELOCITY_AVERAGE_WINDOW;
bool USE_TRACKER_VELOCITIES;
float PRIOR_LAMBDA;

// Workspace limits
bool LIMIT_WORKSPACE_VEL;
float X_LIMITS[2];         // Min-Max X workspace limits
float Y_LIMITS[2];         // Min-Max X workspace limits

// Experiment Parameters
bool SAFETY_STOP = false;
bool STARTED = false;

bool INVERT_X;
// Vector2 I_g1;
// Vector2 I_g2;
// Vector2 I_g3;

bool LOADED_PARAM;

float NEIGHBOR_DIST;
int MAX_NEIGHBORS;
float AGENT_RADIUS;
float GOAL_RADIUS;
float PREF_SPEED;
float PREF_PEOPLE_SPEED;
float MAX_SPEED;
float MAX_PEOPLE_SPEED;
float MAX_ACCELERATION;
float MAX_PEOPLE_ACCELERATION;

// Acquire parameters from launch file. Default values are included just in case
void ParamInitialise() {
  std::string nn = "youbot_experiment/";  // Add node parameter path

  // ROBOT SETUP
  ros::param::param(nn + "megatronActive", MEGATRON_ACTIVE, false);
  if (MEGATRON_ACTIVE) {
    ros::param::param(nn + "megatronPlan", MEGATRON_PLAN, 0);
    ros::param::param(nn + "megatronPlan", MEGATRON_GOAL, 1);
  }
  ros::param::param(nn + "soundwaveActive", SOUNDWAVE_ACTIVE, false);
  if (SOUNDWAVE_ACTIVE) {
    ros::param::param(nn + "soundwavePlan", SOUNDWAVE_PLAN, 0);
    ros::param::param(nn + "megatronPlan", SOUNDWAVE_GOAL, 1);
  }
  ros::param::param(nn + "starscreamActive", STARSCREAM_ACTIVE, false);
  if (STARSCREAM_ACTIVE) {
    ros::param::param(nn + "starscreamPlan", STARSCREAM_PLAN, 0);
    ros::param::param(nn + "megatronPlan", STARSCREAM_GOAL, 1);
  }
  ros::param::param(nn + "blackoutActive", BLACKOUT_ACTIVE, false);
  if (BLACKOUT_ACTIVE) {
    ros::param::param(nn + "blackoutPlan", BLACKOUT_PLAN, 0);
    ros::param::param(nn + "megatronPlan", BLACKOUT_GOAL, 1);
  }
  ros::param::param(nn + "thundercrackerActive", THUNDERCRACKER_ACTIVE, false);
  if (THUNDERCRACKER_ACTIVE) {
    ros::param::param(nn + "thundercrackerPlan", THUNDERCRACKER_PLAN, 0);
    ros::param::param(nn + "megatronPlan", THUNDERCRACKER_GOAL, 1);
  }
  ros::param::param(nn + "primeActive", PRIME_ACTIVE, false);
  if (PRIME_ACTIVE) {
    ros::param::param(nn + "primePlan", PRIME_PLAN, 0);
    ros::param::param(nn + "megatronPlan", PRIME_GOAL, 1);
  }

  // EXPERIMENT SETUP
  ros::param::param(nn + "enablePlanner", ENABLE_PLANNER, false);
  ros::param::param(nn + "hrvoPlanner", HRVO_PLANNER, true);
  ros::param::param(nn + "performRobotSetup", PERFORM_ROBOT_SETUP, true);
  ros::param::param(nn + "manualTrackerAssignment", MANUAL_TRACKER_ASSIGNMENT,
                    true);
  ros::param::param(nn + "onlyOdometry", ONLY_ODOMETRY, false);
  ros::param::param(nn + "isamclactive", IS_AMCL_ACTIVE, false);
  ros::param::param(nn + "megatronBumper", MEGATRON_BUMPER, false);
  ros::param::param(nn + "soundwaveBumper", SOUNDWAVE_BUMPER, false);
  ros::param::param(nn + "starscreamBumper", STARSCREAM_BUMPER, false);
  ros::param::param(nn + "blackoutBumper", BLACKOUT_BUMPER, false);
  ros::param::param(nn + "starscreamBumper", THUNDERCRACKER_BUMPER, false);
  ros::param::param(nn + "trackRobots", TRACK_ROBOTS, false);

  ros::param::param(nn + "enableModelling", ENABLE_MODELLING, true);
  ros::param::param(nn + "modelOwnRobot", MODEL_OWN_ROBOT, false);
  ros::param::param(nn + "logData", LOG_DATA, true);
  ros::param::param(nn + "assignTrackerWhenAlone", ASSIGN_TRACKER_WHEN_ALONE,
                    false);
  ros::param::param(nn + "trackerOdomComparisons", TRACKER_ODOM_COMPARISONS, 10);
  ros::param::param(nn + "rosFreq", ROS_FREQ, 10);
  ros::param::param(nn + "clearScreen", CLEAR_SCREEN, true);
  ros::param::param(nn + "displayIntention", DISPLAY_INTENTION, true);
  ros::param::param(nn + "displayInferenceValues", DISPLAY_INFERENCE_VALUES,
                    false);
  ros::param::param(nn + "displaySimAgents", DISPLAY_SIM_AGENTS, true);
  ros::param::param(nn + "maxNoTrackedAgents", MAX_NO_TRACKED_AGENTS, 10);
  ros::param::param(nn + "wifiAttempts", WIFI_ATTEMPTS, 5);

  ros::param::param<std::string>(nn + "logName", LOG_NAME, "InSpaceDemo.csv");
  ros::param::param(nn + "logPlanner", LOG_PLANNER, 1);

  // Simulation Constant (DO NOT CHANGE)
  ros::param::param(nn + "simTimeStep", SIM_TIME_STEP, 0.1f);

  ros::param::param(nn + "goalSampling", GOAL_SAMPLING, false);
  ros::param::param(nn + "bivariate", BIVARIATE, true);
  ros::param::param(nn + "goalSumPrior", GOAL_SUM_PRIOR, 0.001f);
  ros::param::param(nn + "goalHistoryDiscount", GOAL_HISTORY_DISCOUNT, 0.5f);
  ros::param::param(nn + "goalInferenceHistory", GOAL_INFERENCE_HISTORY,
                    1 * ROS_FREQ);
  ros::param::param(nn + "velocityAverageWindow", VELOCITY_AVERAGE_WINDOW,
                    1 * ROS_FREQ);
  ros::param::param(nn + "priorLambda", PRIOR_LAMBDA, 0.5f);

  ros::param::param(nn + "limitWorkspaceVel", LIMIT_WORKSPACE_VEL, true);
  // Ros param get only works with vectors/maps, not arrays :(
  if (ros::param::has(nn + "xLimits") && ros::param::has(nn + "yLimits")) {
    std::vector<float> x_lim;
    std::vector<float> y_lim;
    ros::param::get(nn + "xLimits", x_lim);
    ros::param::get(nn + "yLimits", y_lim);
    X_LIMITS[0] = x_lim[0];   X_LIMITS[1] = x_lim[1];
    Y_LIMITS[0] = y_lim[0];   Y_LIMITS[1] = y_lim[1];
  } else {
    X_LIMITS[0] = -8.0f;   X_LIMITS[1] = -2.0f;
    Y_LIMITS[0] = 0.0f;   Y_LIMITS[1] = 3.5f;
  }

  ros::param::param(nn + "loadedParam", LOADED_PARAM, false);

  ros::param::param(nn + "neighborDist", NEIGHBOR_DIST, 5.0f);
  ros::param::param(nn + "maxNeighbors", MAX_NEIGHBORS, 10);
  ros::param::param(nn + "agentRadius", AGENT_RADIUS, 0.5f);
  ros::param::param(nn + "goalRadius", GOAL_RADIUS, 0.3f);
  ros::param::param(nn + "prefSpeed", PREF_SPEED, 0.3f);
  ros::param::param(nn + "prefPeopleSpeed", PREF_PEOPLE_SPEED, 0.3f);
  ros::param::param(nn + "maxSpeed", MAX_SPEED, 0.6f);
  ros::param::param(nn + "maxPeopleSpeed", MAX_PEOPLE_SPEED, 2.0f);
  ros::param::param(nn + "maxAcceleration", MAX_ACCELERATION, 1.2f);
  ros::param::param(nn + "maxPeopleAcceleration", MAX_PEOPLE_ACCELERATION, 5.0f);
}
