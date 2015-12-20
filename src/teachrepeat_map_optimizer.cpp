#include "teachrepeat_map_optimizer.h"
#include "tolerance_ellipse_calculator.h"

namespace TeachRepeat {
    MapOptimizer::MapOptimizer(ToleranceEllipseCalculator<float>& toleranceEllipseCalculator) : toleranceEllipseCalculator(toleranceEllipseCalculator){

    }

    Map MapOptimizer::optimize(Map& map) {
        Map optimizedMap;

        std::vector<LocalisedPointCloud> localizablePointClouds = cloudsLocalizedByAnchorPoint(map, 0);
        std::cout << localizablePointClouds.size();

        return optimizedMap;
    }

    std::vector<LocalisedPointCloud> MapOptimizer::cloudsLocalizedByAnchorPoint(Map& map, int anchorPointIndex) {
        std::vector<LocalisedPointCloud>::iterator it = map.begin() + anchorPointIndex;
        LocalisedPointCloud anchorPoint = *it;

        it++;
        std::vector<LocalisedPointCloud> cloudsThatCanBeLocalized;
        while(toleranceEllipseCalculator.readingCanBeLocalizedByAnchorPoint(*it, anchorPoint) &&
                it < map.end()) {
            cloudsThatCanBeLocalized.push_back(*it++);
        }

        return cloudsThatCanBeLocalized;
    }
}

