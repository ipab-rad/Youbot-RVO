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
typedef std::map<std::size_t, Environment *> RobotMapPointer;

// EnvID, ModelID, ModelObject
typedef std::map<std::size_t, std::map<std::size_t, Model*> > ModelMapPointer;

void InitialiseRobots(RobotMapPointer* RobotMap);

void SetupLogging(RobotMapPointer* RobotMap, ModelMapPointer* ModelMap);

void StopRobots(RobotMapPointer* RobotMap);

void EStopRobots(RobotMapPointer* RobotMap);

void InitRobotPoses(RobotMapPointer* RobotMap);

void MoveIntoArea(Environment* planner);

void SelectTracker(Environment* planner);

void MoveToInitialGoal(Environment* planner);

void SensingUpdate(RobotMapPointer* RobotMap);

void PrintAgentState(RobotMapPointer* RobotMap);

void PlannerStep(RobotMapPointer* RobotMap);

void ModelStep(RobotMapPointer* RobotMap, ModelMapPointer* ModelMap);

}  // namespace hrvo

#endif /* HRVO_EXPERIMENT_H_ */
