#ifndef HRVO_EXPERIMENT_H_
#define HRVO_EXPERIMENT_H_

#include <iostream>
#include <ros/ros.h>

namespace hrvo {

  bool ENABLE_PLANNER;              // Enables robot planner, disable when only inferring / tracking
  bool PERFORM_ROBOT_SETUP;         // Robots move into initial positions
  bool MANUAL_TRACKER_ASSIGNMENT;   // False = Automatic setup will assign last TrackerID
  bool ENABLE_MODELLING;            // Enable inference model
  bool LOG_DATA;                    // Log data into a file
  int ROS_FREQ;                        // Planner frequency Hz
  bool CLEAR_SCREEN;                 // Clearing makes it prettier but fits less on the screen
  int WIFI_ATTEMPTS;

  void loadConf()
  {
    ros::param::param("enablePlanner", ENABLE_PLANNER, true);
    ros::param::param("performRobotSetup", PERFORM_ROBOT_SETUP, true);
    ros::param::param("manualTrackerAssignment", MANUAL_TRACKER_ASSIGNMENT, true);
    ros::param::param("enableModelling", ENABLE_MODELLING, true);
    ros::param::param("logData", LOG_DATA, true);
    ros::param::param("enableModelling", ENABLE_MODELLING, true);
    ros::param::param("logData", LOG_DATA, true);
    ros::param::param("rosFreq", ROS_FREQ, 10);
    ros::param::param("clearScreen", CLEAR_SCREEN, true);
    ros::param::param("wifiAttempts", WIFI_ATTEMPTS, 5);
  }

}

#endif /* HRVO_EXPERIMENT_H_ */