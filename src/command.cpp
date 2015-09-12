
#include <sstream>
#include <stdlib.h>

#include "command.h"

namespace TeachRepeat {

  Command::Command()
  { }

  Command::Command(std::string input)
  {
    std::stringstream ss(input);
    std::string buffer;

    std::getline(ss,buffer, ',');
    timestamp = strtod(buffer.c_str(), NULL);

    std::getline(ss,buffer, ',');
    linearSpeed = strtof(buffer.c_str(), NULL);

    std::getline(ss,buffer, ',');
    angularSpeed = strtof(buffer.c_str(), NULL);
  }

  float Command::getLinearSpeed()
  {
    return linearSpeed;
  }

  float Command::getAngularSpeed()
  {
    return angularSpeed;
  }

  double Command::getTimestamp()
  {
    return timestamp;
  }
  
}
