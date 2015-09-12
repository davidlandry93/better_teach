
#include <iostream>
#include <vector>

#include "anchor_point.h"
#include "command.h"
#include "pose.h"

namespace TeachRepeat
{

class Map {
 public:
  Map();
  Map(std::string directory);
  std::vector<AnchorPoint>::iterator begin();
  std::vector<AnchorPoint>::iterator end();
  
 private:
  void loadCommands(std::istream& input);
  void loadPositions(std::istream& input);
  void loadAnchorPoints(std::istream& input);

  std::vector<AnchorPoint> anchorPoints;
  std::vector<Command> commands;
  std::vector<Pose> positions;
  std::string locationOnDisk;
};
  
}
