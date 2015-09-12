
#include <istream>
#include <boost/filesystem.hpp>

#include "teach_repeat_map.h"

namespace TeachRepeat
{

  Map::Map()
  { }
  
  Map::Map(std::string directory) {
    boost::filesystem::path base(directory);

    locationOnDisk = directory;

    std::ifstream commandFile((base /= "speeds.sl").string().c_str());
    loadCommands(commandFile);

    std::ifstream poseFile((base /= "positions.pl").string().c_str());
    loadPositions(poseFile);

    std::ifstream anchorPointsFile((base /= "anchorPoints.apd").string().c_str());
    loadAnchorPoints(anchorPointsFile);
  }

  std::vector<AnchorPoint>::iterator Map::begin()
  {
    return anchorPoints.begin();
  }

  std::vector<AnchorPoint>::iterator Map::end()
  {
    return anchorPoints.end();
  }
    
  void Map::loadCommands(std::istream& input)
  {
    commands.clear();
    
    std::string lineBuffer;
    while(std::getline(input, lineBuffer))
      {
	commands.push_back(Command(lineBuffer));
      }
  }
  
  void Map::loadPositions(std::istream& input)
  {
    positions.clear();
    
    std::string lineBuffer;
    while(std::getline(input, lineBuffer))
      {
	positions.push_back(Pose(lineBuffer));
      }   
  }
  
  void Map::loadAnchorPoints(std::istream& input)
  {
    anchorPoints.clear();
    
    std::string lineBuffer;
    while(std::getline(input, lineBuffer))
      {
	anchorPoints.push_back(AnchorPoint(lineBuffer));
	anchorPoints.back().loadFromDisk(locationOnDisk);
      }
  }

} //namespace TeachRepeat
