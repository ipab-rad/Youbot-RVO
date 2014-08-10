/**
* Created by Alejandro Bordallo
* \file   Environment.cpp
* \brief  Defines the Environment class.
*/

#ifndef HRVO_ENVIRONMENT_H_
#include "Environment.h"
#endif


#ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
#endif

#ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
#endif

namespace hrvo {


  Environment::Environment()
  {
    planner_ = new Simulator();
    planner_->setTimeStep(SIM_TIME_STEP);
    planner_->setAgentDefaults(5.0f, 10, AGENT_RADIUS, GOAL_RADIUS, 0.3f, 0.6f, 0.0f, 0.6f, STOP, 0.0f);
  }
  Environment::~Environment()
  {
    delete planner_;
    planner_ = NULL;
  }
  

std::size_t addTracker()
{
;
}

std::size_t setPlannerParam()
{
 ;
}

std::size_t setSimParam(std::size_t simID)
{
;
}

std::size_t addSimulation()
{
  //simulation_ = new Simulator();
  ;
}

void Environment::emergencyStop()
{
  planner_->setAgentVelocity(0, STOP);
}





}