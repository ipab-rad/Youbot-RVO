/**
* Created by Alejandro Bordallo
* \file   Model.h
* \brief  Declares the Model class.
*/

#include <vector>
#include <map>

#ifndef HRVO_SIMULATOR_H_
#include "Simulator.h"
#endif

#ifndef HRVO_AGENT_H_
#include "Agent.h"
#endif

#ifndef HRVO_VECTOR2_H_
#include "Vector2.h"
#endif

#ifndef HRVO_DEFINITIONS_H_
#include "Definitions.h"
#endif

namespace hrvo {
  class Simulator;
  class Agent;
  class Goal;
  class KdTree;

  class Model
  {
  public:
    Model();
    ~Model();

    private:

  };

}
