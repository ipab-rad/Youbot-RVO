
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

  std::string intToString(int i)
  {
    std::stringstream ss;
    ss << i;
    std::string str = ss.str();
    return str;
  }

void CLEAR() {
    // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
    std::cout << "\x1B[2J\x1B[H";
}

}