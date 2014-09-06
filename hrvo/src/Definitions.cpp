
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

}