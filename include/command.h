
#ifndef COMMAND_H
#define COMMAND_H

#include <iostream>
#include <string>

namespace TeachRepeat {

class Command{
 public:
  Command();
  Command(std::string input);
  float getLinearSpeed();
  float getAngularSpeed();
  double getTimestamp();

 private:
  double timestamp;
  float linearSpeed;
  float angularSpeed;
};

}
#endif
