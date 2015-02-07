/*
 * Agent.h
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
 * \file   Agent.h
 * \brief  Declares the Agent class.
 */

#ifndef HRVO_AGENT_H_
#define HRVO_AGENT_H_

#include <nav_msgs/Odometry.h>

#include "Simulator.h"
#include "Vector2.h"
#include "Parameter.h"

namespace hrvo {
class Simulator;
/**
 * \class  Agent
 * \brief  An agent in the simulation.
 */
class Agent {


 private:
  /**
   * \class  Candidate
   * \brief  A candidate point.
   */
  class Candidate {
   public:
    /**
     * \brief  Constructor.
     */
    Candidate() : velocityObstacle1_(0), velocityObstacle2_(0) { }

    /**
     * \brief  The position of the candidate point.
     */
    Vector2 position_;

    /**
     * \brief  The number of the first velocity obstacle.
     */
    int velocityObstacle1_;

    /**
     * \brief  The number of the second velocity obstacle.
     */
    int velocityObstacle2_;
  };

  /**
   * \class  VelocityObstacle
   * \brief  A hybrid reciprocal velocity obstacle.
   */
  class VelocityObstacle {
   public:
    /**
     * \brief  Constructor.
     */
    VelocityObstacle() { }
    /**
     * \brief  The position of the apex of the hybrid reciprocal velocity obstacle.
     */
    Vector2 apex_;

    /**
     * \brief  The direction of the first side of the hybrid reciprocal velocity obstacle.
     */
    Vector2 side1_;

    /**
     * \brief  The direction of the second side of the hybrid reciprocal velocity obstacle.
     */
    Vector2 side2_;
  };

  explicit Agent(Simulator *simulator);
  /**
   * \brief      Constructor.
   * \param[in]  simulator  The simulation.
   */
  explicit Agent(Simulator *simulator, ros::NodeHandle &nh,
                 std::string id, int agent_type);

  /**
   * \brief      Constructor.
   * \param[in]  simulator  The simulation.
   * \param[in]  position   The starting position of this agent.
   * \param[in]  goalNo     The goal number of this agent.
   */
  Agent(Simulator *simulator, const Vector2 &position,
        std::size_t goalNo, ros::NodeHandle &nh,
        std::string id, int agent_type);

  /**
   * \brief      Constructor.
   * \param[in]  simulator          The simulation.
   * \param[in]  position           The starting position of this agent.
   * \param[in]  goalNo             The goal number of this agent.
   * \param[in]  neighborDist       The maximum neighbor distance of this agent.
   * \param[in]  maxNeighbors       The maximum neighbor count of this agent.
   * \param[in]  radius             The radius of this agent.
   * \param[in]  goalRadius         The goal radius of this agent.
   * \param[in]  prefSpeed          The preferred speed of this agent.
   * \param[in]  maxSpeed           The maximum speed of this agent.
   * \param[in]  uncertaintyOffset  The uncertainty offset of this agent.
   * \param[in]  maxAccel           The maximum acceleration of this agent.
   * \param[in]  velocity           The initial velocity of this agent.
   * \param[in]  orientation        The initial orientation (in radians) of this agent.
   */
  Agent(Simulator *simulator, const Vector2 &position,
        std::size_t goalNo, float neighborDist,
        std::size_t maxNeighbors, float radius,
        const Vector2 &velocity, float maxAccel,
        float goalRadius, float prefSpeed,
        float maxSpeed, float orientation,
#if HRVO_DIFFERENTIAL_DRIVE
        float timeToOrientation, float wheelTrack,
#endif /* HRVO_DIFFERENTIAL_DRIVE */
        float uncertaintyOffset, ros::NodeHandle& nh,
        std::string id, int agent_type);

  /**
   * \brief  Computes the neighbors of this agent.
   */
  void computeNeighbors();

  /**
   * \brief  Computes the new velocity of this agent.
   */
  void computeNewVelocity();

  /**
   * \brief  Computes the preferred velocity of this agent.
   */
  void computePreferredVelocity();

#if HRVO_DIFFERENTIAL_DRIVE
  /**
   * \brief  Computes the wheel speeds of this agent.
   */
  void computeWheelSpeeds();
#endif /* HRVO_DIFFERENTIAL_DRIVE */

  /**
   * \brief          Inserts a neighbor into the set of neighbors of this agent.
   * \param[in]      agentNo  The number of the agent to be inserted.
   * \param[in,out]  rangeSq  The squared range around this agent.
   */
  void insertNeighbor(std::size_t agentNo, float &rangeSq);

  /**
   * \brief  Updates the orientation, position, and velocity of this agent.
   */
  void update();

  void odomPosUpdate();

  void odomUpdate();

  // AGENT PARAMETERS
  // bool limit_workspace_vel;
  // float x_limits[2];       // Min-Max X workspace limits
  // float y_limits[2];        // Min-Max X workspace limits

  Simulator *const simulator_;
  Vector2 newVelocity_;
  Vector2 position_;
  Vector2 agent_sensed_position_;
  Vector2 previous_odometry_offset_;  // Odometry offset before action step
  Vector2 current_odometry_offset_;   // Odometry offset after action step
  bool odomFlag_;
  Vector2 odomPosition_;
  Vector2 prefVelocity_;
  Vector2 velocity_;
  std::size_t goalNo_;
  std::size_t maxNeighbors_;
  bool updated_;
  int agent_type_;
  float goalRadius_;
  float maxAccel_;
  float maxSpeed_;
  float neighborDist_;
  float orientation_;
  float agent_sensed_orientation_;
  float prefSpeed_;
  float radius_;
  float uncertaintyOffset_;
#if HRVO_DIFFERENTIAL_DRIVE
  float leftWheelSpeed_;
  float rightWheelSpeed_;
  float timeToOrientation_;
  float wheelTrack_;
#endif /* HRVO_DIFFERENTIAL_DRIVE */
  bool reachedGoal_;
  std::multimap<float, Candidate> candidates_;
  std::set<std::pair<float, std::size_t> > neighbors_;
  std::vector<VelocityObstacle> velocityObstacles_;

  friend class KdTree;
  friend class Simulator;
  friend class Environment;

 public:
  std::string id_, pose_topic_;
  ros::Publisher pub_;
  ros::Subscriber odom_sub_;

  /**
   * \brief  Updates the position from topic.
   */
  void updatePose(const nav_msgs::Odometry::ConstPtr &pose_msg);
  /**
   * \brief  get the pose topic.
   */
  std::string getPoseTopic();
  /**
   * \brief  set the pose topic.
   */
  void setPoseTopic(std::string pose_topic);
  /**
   * brief  set the pose subscriber.
   */
  void attachPoseSubscriber(ros::NodeHandle &nh,
                            std::string pose_topic);
  // #endif
};
}

#endif /* HRVO_AGENT_H_ */
