#ifndef HRVO_EXPERIMENT_H_
#define HRVO_EXPERIMENT_H_

#include "Environment.h"

#include "Model.h"

#include "Definitions.h"

#include "Parameter.h"

#include <cmath>
#include <csignal>
#include <fstream>

//#include "AMCLWrapper.h"

namespace hrvo {

float startTime(-1.0);

std::ofstream dataLog;

ros::Rate update_freq(ROS_FREQ);

void WaitReturn() {while( std::cin.get() != '\n') {;}}

typedef std::map<std::size_t, Environment *> PlannerMapPointer; // EnvID, EnvObject

typedef std::map<std::size_t, std::map<std::size_t, Model*> > ModelMapPointer; // EnvID, ModelID, ModelObject

void InitialiseRobots(PlannerMapPointer* PlannerMap);

void SetupLogging(PlannerMapPointer* PlannerMap, ModelMapPointer* ModelMap);

void StopRobots(PlannerMapPointer* PlannerMap);

void EStopRobots(PlannerMapPointer* PlannerMap);

void MoveIntoArea(Environment* planner);

void SelectTracker(Environment* planner);

void LoadInitialGoals(PlannerMapPointer* PlannerMap);

void SensingUpdate(PlannerMapPointer* PlannerMap);

void PrintAgentState(PlannerMapPointer* PlannerMap);

void PlannerStep(PlannerMapPointer* PlannerMap);

void ModelStep(PlannerMapPointer* PlannerMap, ModelMapPointer* ModelMap);

}

#endif /* HRVO_EXPERIMENT_H_ */