/**
* Created by Alejandro Bordallo
* \file   Parameter.h
* \brief  Contains all global parameters shared between the classes.
*         These are read once at the beginning of execution from the ROS launch file.
*/

#ifndef PARAMETER_H
#define PARAMETER_H

#include <iostream>
#include <ros/ros.h>
#include "Vector2.h"

// Robot Setup
extern bool MEGATRON_ACTIVE;
extern int MEGATRON_PLAN;
extern bool SOUNDWAVE_ACTIVE;
extern int SOUNDWAVE_PLAN;
extern bool STARSCREAM_ACTIVE;
extern int STARSCREAM_PLAN;
extern bool BLACKOUT_ACTIVE;
extern int BLACKOUT_PLAN;
extern bool THUNDERCRACKER_ACTIVE;
extern int THUNDERCRACKER_PLAN;
extern bool PRIME_ACTIVE;
extern int PRIME_PLAN;

// Experiment Setup
extern bool ENABLE_PLANNER;             // Enables robot planner, disable when only inferring / tracking
extern bool HRVO_PLANNER;
extern bool PERFORM_ROBOT_SETUP;        // Robots move into initial positions
extern bool MANUAL_TRACKER_ASSIGNMENT;  // False = Automatic setup will assign last TrackerID
extern bool ONLY_ODOMETRY;              // Use only odometry for robots, no tracker feedback
extern bool IS_AMCL_ACTIVE;             // Whether AMCL is running on the robot, requires laser scanner
extern bool TRACK_ROBOTS;               // Whether the tracker is setup to track robots as well as people

extern bool ENABLE_MODELLING;           // Enable inference model
extern bool MODEL_OWN_ROBOT;            // Model the planning robot as well (Start from Agent 0)
extern bool LOG_DATA;                   // Log data into a file
extern bool ASSIGN_TRACKER_WHEN_ALONE;  // When only one agent is tracked, assign tracker to robot
extern int TRACKER_ODOM_COMPARISONS;    // How many iterations after tracker of another agent is reassigned to robot
extern int ROS_FREQ;                    // Planner frequency Hz
extern bool CLEAR_SCREEN;               // Clearing makes it prettier but fits less on the screen
extern bool DISPLAY_INTENTION;
extern bool DISPLAY_INFERENCE_VALUES;   // Displays curr vs sim Vels and goal inference vs sum values
extern bool DISPLAY_SIM_AGENTS;         // Display pos, vel and goal for all simulated agents.
extern int MAX_NO_TRACKED_AGENTS;       // TODO: Not working as intended
extern int WIFI_ATTEMPTS;

// Logger Parameters
extern std::string LOG_NAME;            // The name given to the log file
extern int LOG_PLANNER;                 // The planner environment (robot) which the logger will save its information

// Simulation Constants (DO NOT CHANGE)
extern int THIS_ROBOT;                  // Always 0, representing first agent (own robot) of the list
extern float SIM_TIME_STEP;             // Simulation time step, affects many parts of the code

// Model Parameters
extern bool GOAL_SAMPLING;              // Enable sampling of goals over discretised space
extern bool BIVARIATE;
extern float GOAL_SUM_PRIOR;            // Goal inference initial prior
extern float GOAL_HISTORY_DISCOUNT;     // Discount of previous likelihood history
extern int GOAL_INFERENCE_HISTORY;      // 1 second window
extern int VELOCITY_AVERAGE_WINDOW;     // 1 second window
extern float PRIOR_LAMBDA;

// Agent Parameters
extern bool LIMIT_WORKSPACE_VEL;
extern float X_LIMITS[2];               // Min-Max X workspace limits
extern float Y_LIMITS[2];               // Min-Max X workspace limits

// Experiment Parameters
extern bool SAFETY_STOP;                // Safety stop flag
extern bool STARTED;                    // Robot movement started flag

extern bool INVERT_X;
// const Vector2 I_g1;
// extern Vector2 I_g2;
// extern Vector2 I_g3;
extern bool LOADED_PARAM;

extern float NEIGHBOR_DIST;
extern int MAX_NEIGHBORS;
extern float AGENT_RADIUS;
extern float GOAL_RADIUS;
extern float PREF_SPEED;
extern float PREF_PEOPLE_SPEED;
extern float MAX_SPEED;
extern float MAX_PEOPLE_SPEED;
extern float MAX_ACCELERATION;
extern float MAX_PEOPLE_ACCELERATION;

extern void ParamInitialise();

#endif
