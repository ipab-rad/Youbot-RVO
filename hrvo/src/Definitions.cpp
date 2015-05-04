/**
* Copyright[2015]<Alejandro Bordallo>
* @Author: GreatAlexander
* @Date:   2015-05-01 18:45:56
* @Last Modified by:   GreatAlexander
* @Last Modified time: 2015-05-01
*/

#include "Definitions.h"
#include "Environment.h"
#include "Model.h"

namespace hrvo {

// const bool ENABLE_PLANNER=true;
// hrvo::ros::param::get("enablePlanner", ENABLE_PLANNER);
// bool ros::param::get("enablePlanner", ENABLE_PLANNER) const;

void interrupt_callback(int s) {
  if ( !STARTED ) {
    WARN("Aborted!" << std::endl);
    ros::param::del("/youbot_experiment/loadedParam");
    exit(1);
  }
  SAFETY_STOP = true;
}

const char* getActorName(enum Actor actorID) {
  switch (actorID) {
  case 0: return "youbot_1";
  case 1: return "youbot_2";
  case 2: return "youbot_3";
  case 3: return "youbot_4";
  case 4: return "youbot_5";
  case 5: return "prime";
  default: return "ERROR";
  }
}

std::string intToString(int i) {
  std::stringstream ss;
  ss << i;
  std::string str = ss.str();
  return str;
}

int cinInteger() {
  std::string input = "";
  int myNumber = 0;

  while (true) {
    std::getline(std::cin, input);

    // This code converts from string to number safely.
    std::stringstream myStream(input);
    if (myStream >> myNumber)
    {break;}
    ERR("Invalid number, please try again" << std::endl);
  }
  return myNumber;
}

// void loadConfig()
// {
//   // bool enablePlanner;
//   // ros::param::get("enablePlanner", enablePlanner);
//   // ENABLE_PLANNER = enablePlanner;
//   // bool ENABLE_PLANNER;
//   // ros::param::get("enablePlanner", ENABLE_PLANNER);
//   INFO("LOADED" << ENABLE_PLANNER <<std::endl);
// }

std::map<std::size_t, Environment *> *RobotMapPointer_;
std::map<std::size_t, std::map<std::size_t, Model *> > *ModelMapPointer_;
// std::string LOG_NAME;

void logSetup(std::ofstream& logfile,
              std::map<std::size_t, Environment *> *RobotMap,
              std::map<std::size_t, std::map<std::size_t, Model*> > *ModelMap) {
  RobotMapPointer_ = RobotMap;
  ModelMapPointer_ = ModelMap;
  std::string folder =
    "/home/alex/Dropbox/University/PhD/Experiments/InSpace/logs/Improved/";
  std::string fullpath = folder + LOG_NAME;
  if (!logfile.is_open()) {
    const char *path = (fullpath).c_str();
    logfile.open(path);
    if (logfile.fail())
    {ERR("Writing to log failed!" << std::endl);}
    else
    {DEBUG("Saving log on " << path << std::endl);}
    // LOG HEADER
    logfile << SIM_TIME_STEP << "," << RobotMapPointer_->size()
            << "," << AGENT_RADIUS << std::endl;
  }
}

void logData(std::ofstream& logfile, int LOG_PLANNER, float currTime,
             std::vector<size_t> modelledAgents,
             std::map<std::size_t, Vector2> possGoals) {
  Environment* planner = (*RobotMapPointer_).at(LOG_PLANNER);
  std::map<std::size_t, Model *> ModelMap = (*ModelMapPointer_).at(LOG_PLANNER);

  size_t nAgents = planner->getNumPlannerAgents();

  logfile << currTime;
  logfile << "," << nAgents;

  // List number of Agents actually modelled and their IDs
  logfile << "," << modelledAgents.size();
  for (std::vector<size_t>::iterator iter = modelledAgents.begin();
       iter != modelledAgents.end(); ++iter) {
    logfile << "," << *iter;
  }

  // List number of goals and their positions
  logfile << "," << possGoals.size();
  for (std::map<std::size_t, Vector2>::iterator iter = possGoals.begin();
       iter != possGoals.end(); ++iter) {
    logfile << "," << iter->second.getX() << "," << iter->second.getY();
  }

  // Position
  for (std::vector<size_t>::iterator iter = modelledAgents.begin();
       iter != modelledAgents.end(); ++iter) {
    logfile << "," << planner->getPlannerAgentPosition(*iter).getX();
    logfile << "," << planner->getPlannerAgentPosition(*iter).getY();
  }

  // Velocity
  for (std::vector<size_t>::iterator iter = modelledAgents.begin();
       iter != modelledAgents.end(); ++iter) {
    logfile << "," << planner->getPlannerAgentVelocity(*iter).getX();
    logfile << "," << planner->getPlannerAgentVelocity(*iter).getY();
  }

  // Average / Maximum Speeds
  for (std::vector<size_t>::iterator iter = modelledAgents.begin();
       iter != modelledAgents.end(); ++iter) {
    logfile << "," << planner->getPlannerAgentAvgSpeed(*iter);
    logfile << "," << planner->getPlannerAgentMaxSpeed(*iter);
  }

  // Simulated Velocities
  for (std::vector<size_t>::iterator iter = modelledAgents.begin();
       iter != modelledAgents.end(); ++iter) {
    std::vector<Vector2> simVels_ = ModelMap[*iter]->getSimVels();
    for (std::vector<Vector2>::iterator Vel = simVels_.begin();
         Vel != simVels_.end(); ++Vel) {
      logfile << "," << (*Vel).getX();
      logfile << "," << (*Vel).getY();
    }
  }

  // Goal Likelihood
  for (std::vector<size_t>::iterator iter = modelledAgents.begin();
       iter != modelledAgents.end(); ++iter) {
    std::map<std::size_t, float> inferredGoalsSum_ =
      ModelMap[*iter]->getGoalLikelihoods();
    for (std::map<std::size_t, float>::iterator Lik = inferredGoalsSum_.begin();
         Lik != inferredGoalsSum_.end(); ++Lik) {
      logfile << "," << (*Lik).second;
    }
  }

  // Goal Ratio
  for (std::vector<size_t>::iterator iter = modelledAgents.begin();
       iter != modelledAgents.end(); ++iter) {
    std::vector<float> goalRatios_ = ModelMap[*iter]->getGoalRatios();
    for (std::vector<float>::iterator Rat = goalRatios_.begin();
         Rat != goalRatios_.end(); ++Rat) {
      logfile << "," << *Rat;
    }
  }

  logfile << std::endl;
}

}  // namespace hrvo
