#include <iostream>
#include <vector>

#include "localised_point_cloud.h"
#include "command.h"
#include "pose.h"
#include "pointmatcherservice.h"

namespace TeachRepeat {

    class Map {
    public:
        Map();
        Map(std::string directory);
        std::vector<LocalisedPointCloud>::iterator begin();
        std::vector<LocalisedPointCloud>::iterator end();

    private:
        void loadCommands(std::istream &input);
        void loadPositions(std::istream &input);
        void loadAnchorPoints(std::istream &input);
        std::vector<LocalisedPointCloud> anchorPoints;
        std::vector<Command> commands;
        std::vector<Pose> positions;
        std::string locationOnDisk;
    };

}
