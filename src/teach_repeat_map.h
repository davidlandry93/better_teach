
#ifndef TEACH_REPEAT_MAP
#define TEACH_REPEAT_MAP

#include <iostream>
#include <vector>
#include <string>
#include <list>

#include "pointmatcherservice.h"
#include "localised_point_cloud.h"
#include "command.h"
#include "pose.h"
#include "pointmatcherservice.h"

namespace TeachRepeat {

    class Map {
    public:
        Map();
        Map(std::string directory);
        Map(std::string directory, PointMatcherService<float>& service);
        std::vector<LocalisedPointCloud>::iterator begin();
        std::vector<LocalisedPointCloud>::iterator end();
        int size() const;

        void addAnchorPoint(LocalisedPointCloud& anchorPoint);
        LocalisedPointCloud getAnchorPointByIndex(int index) const;
        void removeAllAnchorsExceptIndexes(std::list<int>& indexes);

        void correctPositions(PointMatcherService<float>& pointMatcherService);
        void outputAnchorPointsMetadata(std::ostream &ostream);

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

#endif