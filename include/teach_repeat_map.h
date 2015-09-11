
#include "anchor_point.h"

namespace TeachRepeat
{

class TeachRepeatMap {
 public:
  TeachRepeatMap();
  TeachRepeatMap(std::string directory);
  std::vector<AnchorPoint>::iterator begin();
  std::vector<AnchorPoint>::iterator end();
  
 private:
  std::vector<AnchorPoint> anchorPoints;

};
  
}
