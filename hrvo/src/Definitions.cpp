
#include "Definitions.h"

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

}