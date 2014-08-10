/**
* Created by Alejandro Bordallo
* \file   Environment.h
* \brief  Declares the Environment class.
*/

#ifndef HRVO_SIMULATOR_H_
#include "Simulator.h"
#endif


namespace hrvo {
  class Simulator;
  class Agent;
  class Goal;
  class KdTree;

  class Environment
  {
  public:
    Environment();
    ~Environment();

    std::size_t addTracker();
    
    std::size_t setPlannerParam();

    std::size_t setSimParam();

    std::size_t addSimulation();

  /**
  * \brief    Sends to all agents an emergency stop command.
  */
    void emergencyStop();



    private:
      friend class Simulator;
      friend class Agent;
      friend class Goal;
      friend class KdTree;

      Simulator *planner_;
      Simulator *simulation_;




  };

}
