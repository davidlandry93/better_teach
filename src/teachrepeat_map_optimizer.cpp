#include "teachrepeat_map_optimizer.h"
#include "tolerance_ellipse_calculator.h"
#include "localizability_graph.h"

namespace TeachRepeat {
    MapOptimizer::MapOptimizer(ToleranceEllipseCalculator<float>& toleranceEllipseCalculator) : toleranceEllipseCalculator(toleranceEllipseCalculator){

    }

    Map MapOptimizer::optimize(Map& map) {
        Map optimizedMap;
        LocalizabilityGraph localizabilityGraph(map.size());

        for(int i = 0; i < map.size(); i++) {
            std::vector<int> localizablePointClouds = cloudsLocalizedByAnchorPoint(map, i);

            for(auto pointIndex : localizablePointClouds) {
                localizabilityGraph.anchorPointLocalisesPoint(i, pointIndex);
            }

            std::cout << "Point no " << i << ": " << localizablePointClouds.size() << std::endl;
        }

        return optimizedMap;
    }

    std::vector<int> MapOptimizer::cloudsLocalizedByAnchorPoint(Map& map, int anchorPointIndex) {
        std::vector<LocalisedPointCloud>::iterator it = map.begin() + anchorPointIndex;
        LocalisedPointCloud anchorPoint = *it;

        it++;
        std::vector<int> cloudsThatCanBeLocalized;
        while(toleranceEllipseCalculator.readingCanBeLocalizedByAnchorPoint(*it, anchorPoint) &&
                it < map.end()) {
            cloudsThatCanBeLocalized.push_back(it++ - map.begin());
        }

        return cloudsThatCanBeLocalized;
    }
}
