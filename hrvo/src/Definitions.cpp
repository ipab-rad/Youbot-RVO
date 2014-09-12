
#ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
#endif


#ifndef HRVO_ENVIRONMENT_H_
#include "Environment.h"
#endif


namespace hrvo {

  const char* getActorName(enum Actor actorID)
    {
      switch (actorID) 
      {
        case 0: return "youbot_1";
        case 1: return "youbot_2";
        case 2: return "youbot_3";
        case 3: return "youbot_4";
        case 4: return "youbot_5";
      }
    }

  std::string intToString(int i)
  {
    std::stringstream ss;
    ss << i;
    std::string str = ss.str();
    return str;
  }

  int cinInteger()
  {
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

  std::map<std::size_t, Environment *> *PlannerMapPointer_;
  std::map<std::size_t, std::map<std::size_t, Model *> > *ModelMapPointer_;

  void logSetup(std::ofstream& logfile, std::map<std::size_t, Environment *> *PlannerMap, std::map<std::size_t, std::map<std::size_t, Model*> > *ModelMap)
  {
    const char *path = fileName.c_str();
    PlannerMapPointer_ = PlannerMap;
    ModelMapPointer_ = ModelMap;
    if (!logfile.is_open())
    {
      logfile.open(path);
      if (logfile.fail())
        {ERR("Writing to log failed!" << std::endl);}
      else
        {DEBUG("Saving log on " << path << std::endl);}
      // LOG HEADER
      logfile << SIM_TIME_STEP <<","<< PlannerMapPointer_->size() <<","<< AGENT_RADIUS << std::endl;
    }
  }

  void logData(std::ofstream& logfile, float currTime)
  {
    Environment* planner = (*PlannerMapPointer_).at(LogPlanner);
    
    size_t nAgents = planner->getNumPlannerAgents();

    logfile << currTime << ",";
    logfile << nAgents << ",";
    logfile << ",";
    // Modelled
    // Goals

    for(size_t AgentID = 0; AgentID < nAgents; ++AgentID) 
    {
    logfile << "," << planner->getPlannerAgentPosition(AgentID).getX();
    logfile << "," << planner->getPlannerAgentPosition(AgentID).getY();
    }

    for(size_t AgentID = 0; AgentID < nAgents; ++AgentID)
    {
      logfile << "," << planner->getPlannerAgentVelocity(AgentID).getX(); 
      logfile << "," << planner->getPlannerAgentVelocity(AgentID).getY();
    }

    // Simulation Velocity
    for(size_t AgentID = 0; AgentID < nAgents; ++AgentID)
    {
      logfile << "," << planner->getPlannerAgentVelocity(AgentID).getX(); 
      logfile << "," << planner->getPlannerAgentVelocity(AgentID).getY();
    }

    // Goal Inference
    for(size_t AgentID = 0; AgentID < nAgents; ++AgentID)
    {
      logfile << "," << planner->getPlannerAgentVelocity(AgentID).getX(); 
      logfile << "," << planner->getPlannerAgentVelocity(AgentID).getY();
    }
  }

}