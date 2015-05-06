/*
* @Copyright: Copyright[2015]<Alejandro Bordallo>
* @Date:      2015-05-04
* @Email:     alex.bordallo@ed.ac.uk
* @Desc:      Defines functions for inter-robot pose/velocity comms
*/
#include "RobotComms.h"

namespace hrvo {

RobotComms::RobotComms(const Actor* nActorID, const std::string* sActorID) {
  DEBUG("Robot Comms initialising..." << std::endl);
  nActorID_ = nActorID;
  sActorID_ = sActorID;
  posePub_ = nh_.advertise<geometry_msgs::Pose>(
               "/" + *sActorID_ + "/true_pose", 1);
  robotPoses = environment_->getRobotPosesPtr();
  robotVels = environment_->getRobotVelsPtr();
}

RobotComms::~RobotComms() {
}

void RobotComms::publishPose() {
  geometry_msgs::Pose pose;
  pose.position.x = planner_->getAgentPosition(THIS_ROBOT).getX();
  pose.position.y = planner_->getAgentPosition(THIS_ROBOT).getY();
  posePub_.publish(pose);
}

void RobotComms::initRobotTrackers() {
  // DO THIS ONCE ALL ROBOTS HAVE STARTED
  // CHECK ALL EXISITING ROBOTS IN THE EXPERIMENT
  // ADD ALL ROBOTS TO THE PLANNER
  // SET UP ODOM/AMCL SUBSCRIBERS.
  startGoal_ = planner_->getAgentGoal(THIS_ROBOT);
  DEBUG(*sActorID_ << " subscribed to: ")
  if ((*nActorID_ != MEGATRON) && MEGATRON_ACTIVE) {
    std::size_t agentid = planner_->addAgent(getActorName(MEGATRON),
                          SIMAGENT, EXIT, startGoal_);
    trackedRobots_[MEGATRON] = agentid;
    robotPoseSubs[agentid] = nh_.subscribe("/youbot_1/true_pose", 1,
                                           &RobotComms::receiveRobot1Pose,
                                           this);
    robotVelSubs[agentid] = nh_.subscribe("/youbot_1/cmd_vel", 1,
                                          &RobotComms::receiveRobot1Vel,
                                          this);
    DEBUG(getActorName(MEGATRON) << " id " << agentid << ", ");
  }
  if ((*nActorID_ != SOUNDWAVE) && SOUNDWAVE_ACTIVE) {
    std::size_t agentid = planner_->addAgent(getActorName(SOUNDWAVE),
                          SIMAGENT, EXIT, startGoal_);
    trackedRobots_[SOUNDWAVE] = agentid;
    robotPoseSubs[agentid] = nh_.subscribe("/youbot_2/true_pose", 1,
                                           &RobotComms::receiveRobot2Pose,
                                           this);
    robotVelSubs[agentid] = nh_.subscribe("/youbot_2/cmd_vel", 1,
                                          &RobotComms::receiveRobot2Vel,
                                          this);
    DEBUG(getActorName(SOUNDWAVE) << " id " << agentid << ", ");
  }
  if ((*nActorID_ != STARSCREAM) && STARSCREAM_ACTIVE) {
    std::size_t agentid = planner_->addAgent(getActorName(STARSCREAM),
                          SIMAGENT, EXIT, startGoal_);
    trackedRobots_[STARSCREAM] = agentid;
    robotPoseSubs[agentid] = nh_.subscribe("/youbot_3/true_pose", 1,
                                           &RobotComms::receiveRobot3Pose,
                                           this);
    robotVelSubs[agentid] = nh_.subscribe("/youbot_3/cmd_vel", 1,
                                          &RobotComms::receiveRobot3Vel,
                                          this);
    DEBUG(getActorName(STARSCREAM) << " id " << agentid << ", ");
  }
  if ((*nActorID_ != BLACKOUT) && BLACKOUT_ACTIVE) {
    std::size_t agentid = planner_->addAgent(getActorName(BLACKOUT),
                          SIMAGENT, EXIT, startGoal_);
    trackedRobots_[BLACKOUT] = agentid;
    robotPoseSubs[agentid] = nh_.subscribe("/youbot_4/true_pose", 1,
                                           &RobotComms::receiveRobot4Pose,
                                           this);
    robotVelSubs[agentid] = nh_.subscribe("/youbot_4/cmd_vel", 1,
                                          &RobotComms::receiveRobot4Vel,
                                          this);
    DEBUG(getActorName(BLACKOUT) << " id " << agentid << ", ");
  }
  if ((*nActorID_ != THUNDERCRACKER) && THUNDERCRACKER_ACTIVE) {
    std::size_t agentid = planner_->addAgent(getActorName(THUNDERCRACKER),
                          SIMAGENT, EXIT, startGoal_);
    trackedRobots_[THUNDERCRACKER] = agentid;
    robotPoseSubs[agentid] = nh_.subscribe("/youbot_5/true_pose", 1,
                                           &RobotComms::receiveRobot5Pose,
                                           this);
    robotVelSubs[agentid] = nh_.subscribe("/youbot_5/cmd_vel", 1,
                                          &RobotComms::receiveRobot5Vel,
                                          this);
    DEBUG(getActorName(THUNDERCRACKER) << " id " << agentid << ", ");
  }
  if ((*nActorID_ != PRIME) && PRIME_ACTIVE) {
    std::size_t agentid = planner_->addAgent(getActorName(PRIME),
                          SIMAGENT, EXIT, startGoal_);
    trackedRobots_[PRIME] = agentid;
    robotPoseSubs[agentid] = nh_.subscribe("/prime/true_pose", 1,
                                           &RobotComms::receiveRobot6Pose,
                                           this);
    robotVelSubs[agentid] = nh_.subscribe("/prime/cmd_vel", 1,
                                          &RobotComms::receiveRobot6Vel,
                                          this);
    DEBUG(getActorName(PRIME)  << " id " << agentid);
  }
  DEBUG(std::endl);
  DEBUG("Multi-robot pose/vels subscriptions for " <<
        *sActorID_ << " complete" << std::endl);
}

void RobotComms::receiveRobot1Pose(const geometry_msgs::Pose& msg) {
  (*robotPoses)[trackedRobots_[MEGATRON]].setX(msg.position.x);
  (*robotPoses)[trackedRobots_[MEGATRON]].setY(msg.position.y);
}
void RobotComms::receiveRobot2Pose(const geometry_msgs::Pose& msg) {
  (*robotPoses)[trackedRobots_[SOUNDWAVE]].setX(msg.position.x);
  (*robotPoses)[trackedRobots_[SOUNDWAVE]].setY(msg.position.y);
}
void RobotComms::receiveRobot3Pose(const geometry_msgs::Pose& msg) {
  (*robotPoses)[trackedRobots_[STARSCREAM]].setX(msg.position.x);
  (*robotPoses)[trackedRobots_[STARSCREAM]].setY(msg.position.y);
}
void RobotComms::receiveRobot4Pose(const geometry_msgs::Pose& msg) {
  (*robotPoses)[trackedRobots_[BLACKOUT]].setX(msg.position.x);
  (*robotPoses)[trackedRobots_[BLACKOUT]].setY(msg.position.y);
}
void RobotComms::receiveRobot5Pose(const geometry_msgs::Pose& msg) {
  (*robotPoses)[trackedRobots_[THUNDERCRACKER]].setX(msg.position.x);
  (*robotPoses)[trackedRobots_[THUNDERCRACKER]].setY(msg.position.y);
}
void RobotComms::receiveRobot6Pose(const geometry_msgs::Pose& msg) {
  (*robotPoses)[trackedRobots_[PRIME]].setX(msg.position.x);
  (*robotPoses)[trackedRobots_[PRIME]].setY(msg.position.y);
}

void RobotComms::receiveRobot1Vel(const geometry_msgs::Twist& msg) {
  (*robotVels)[trackedRobots_[MEGATRON]].setX(msg.linear.x);
  (*robotVels)[trackedRobots_[MEGATRON]].setY(msg.linear.y);
}
void RobotComms::receiveRobot2Vel(const geometry_msgs::Twist& msg) {
  (*robotVels)[trackedRobots_[SOUNDWAVE]].setX(msg.linear.x);
  (*robotVels)[trackedRobots_[SOUNDWAVE]].setY(msg.linear.y);
}
void RobotComms::receiveRobot3Vel(const geometry_msgs::Twist& msg) {
  (*robotVels)[trackedRobots_[STARSCREAM]].setX(msg.linear.x);
  (*robotVels)[trackedRobots_[STARSCREAM]].setY(msg.linear.y);
}
void RobotComms::receiveRobot4Vel(const geometry_msgs::Twist& msg) {
  (*robotVels)[trackedRobots_[BLACKOUT]].setX(msg.linear.x);
  (*robotVels)[trackedRobots_[BLACKOUT]].setY(msg.linear.y);
}
void RobotComms::receiveRobot5Vel(const geometry_msgs::Twist& msg) {
  (*robotVels)[trackedRobots_[THUNDERCRACKER]].setX(msg.linear.x);
  (*robotVels)[trackedRobots_[THUNDERCRACKER]].setY(msg.linear.y);
}
void RobotComms::receiveRobot6Vel(const geometry_msgs::Twist& msg) {
  (*robotVels)[trackedRobots_[PRIME]].setX(msg.linear.x);
  (*robotVels)[trackedRobots_[PRIME]].setY(msg.linear.y);
}

}  // namespace hrvo
