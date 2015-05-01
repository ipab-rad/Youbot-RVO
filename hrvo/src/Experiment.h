/*
* Copyright[2015]<Alejandro Bordallo>
*/

#ifndef HRVO_EXPERIMENT_H_
#define HRVO_EXPERIMENT_H_

#include "Environment.h"

#include "Model.h"

#include "Definitions.h"

#include "Parameter.h"

#include <cmath>
#include <csignal>
#include <fstream>

#include <map>
#include <vector>

#include "AMCLWrapper.h"

namespace hrvo {

float startTime(-1.0);

std::ofstream dataLog;

void WaitReturn() {while ( std::cin.get() != '\n') {;}}

// EnvID, EnvObject
typedef std::map<std::size_t, Environment *> PlannerMapPointer;

// EnvID, ModelID, ModelObject
typedef std::map<std::size_t, std::map<std::size_t, Model*> > ModelMapPointer;

void InitialiseRobots(PlannerMapPointer* PlannerMap);

void SetupLogging(PlannerMapPointer* PlannerMap, ModelMapPointer* ModelMap);

void StopRobots(PlannerMapPointer* PlannerMap);

void EStopRobots(PlannerMapPointer* PlannerMap);

void InitRobotPoses(PlannerMapPointer* PlannerMap);

void MoveIntoArea(Environment* planner);

void SelectTracker(Environment* planner);

void MoveToInitialGoal(Environment* planner);

void SensingUpdate(PlannerMapPointer* PlannerMap);

void PrintAgentState(PlannerMapPointer* PlannerMap);

void PlannerStep(PlannerMapPointer* PlannerMap);

void ModelStep(PlannerMapPointer* PlannerMap, ModelMapPointer* ModelMap);

}  // namespace hrvo

#endif /* HRVO_EXPERIMENT_H_ */
