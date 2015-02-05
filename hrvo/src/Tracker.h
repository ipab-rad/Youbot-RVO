/**
* Created by Alejandro Bordallo
* \file   Tracker.h
* \brief  Declares the Tracker class.
*/

#ifndef HRVO_TRACKER_H_
#define HRVO_TRACKER_H_

#include <ros/ros.h>
#include "PTrackingBridge/TargetEstimations.h"
// #include "geometry_msgs/Twist.h"
// #include "std_msgs/Header.h"

// #include "Definitions.h"
#include "Environment.h"

#include "Simulator.h"

#include "Parameter.h"


namespace hrvo {
  class Environment;
  class Simulator;

  class Tracker
  {
    public:
      Tracker();
      ~Tracker();

      void updateTracker();

      std::map<int, std::size_t> getTrackerIDs();

      void receiveTrackerData(const PTrackingBridge::TargetEstimations::ConstPtr& msg);

      void setAgentTracker(int TrackerID, std::size_t AgentID);

      std::pair<float, Vector2> calculateAvgMaxSpeeds(int AgentID, Vector2 AgentVel);

      void checkExistingTrackers(std::map<int, std::size_t> ids);

      void removeInactiveTrackers();

      void updateActiveAgents(std::size_t numAgents);

      void odometryComparison();

      // Awaits robot initialisation before tracking more agents
      void setTrackOtherAgents(bool trackOtherAgents) {trackOtherAgents_ = trackOtherAgents;}

      int getRobotTrackerID()  {return robotTrackerID_;}

      void setEnvPointer(Environment *environment) {environment_ = environment;}

      void setPlannerPointer(Simulator *planner) {planner_ = planner;}


    private:

      // ROS members
      ros::NodeHandle nh_;
      ros::Subscriber Targsub;
      PTrackingBridge::TargetEstimations msg_;

      // Class pointers
      Environment *environment_;
      Simulator *planner_;

      // Private Members
      std::map<std::size_t, std::vector<Vector2> > agentVelHistory_;  // SimAgentID : VelCount : Velocity Magnitude
      std::map<std::size_t, float> maxSpeed_;
      bool trackOtherAgents_;

      int robotTrackerID_;
      std::map<int, std::size_t> trackedAgents_;            // First : Tracker ID, Second : SimAgent ID
      std::map<int, std::vector<float> > trackerCompOdom_;  // First : Tracker ID, Second : Cumulative Diff between Odometry and Tracker Position

  };

}

#endif /* HRVO_TRACKER_H_ */
