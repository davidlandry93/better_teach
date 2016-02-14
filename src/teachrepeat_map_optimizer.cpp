#include "teachrepeat_map_optimizer.h"
#include "tolerance_ellipse_calculator.h"
#include "localizability_graph.h"

namespace TeachRepeat {
    MapOptimizer::MapOptimizer(ToleranceEllipseCalculator<float>& toleranceEllipseCalculator) : toleranceEllipseCalculator(toleranceEllipseCalculator){

    }

    void MapOptimizer::optimize(Map& map) {
        localizabilityGraph = new LocalizabilityGraph(map.size());

        for(int i = 0; i < map.size(); i++) {
            std::vector<int> localizablePointClouds = cloudsLocalizedByAnchorPoint(map, i);

            for(auto pointIndex : localizablePointClouds) {
                localizabilityGraph->anchorPointLocalizesPoint(i, pointIndex);
            }

            std::cout << "Point no " << i << ": ";
            for(auto i = localizablePointClouds.begin(); i < localizablePointClouds.end(); i++) {
                std::cout << *i << ", ";
            }
            std::cout << std::endl;
        }

        std::list<int> optimalSetOfAnchors = localizabilityGraph->optimalSetOfAnchorPoints();
        map.removeAllAnchorsExceptIndexes(optimalSetOfAnchors);
    }

    std::vector<int> MapOptimizer::cloudsLocalizedByAnchorPoint(Map& map, int anchorPointIndex) {
        std::vector<LocalisedPointCloud>::iterator it = map.begin() + anchorPointIndex;
        LocalisedPointCloud anchorPoint = *it;

        // We assume that the neighboring points are already localized.
        std::vector<int> cloudsThatCanBeLocalized;
        if(it != map.begin()) {
            cloudsThatCanBeLocalized.push_back(it - map.begin() - 1);
        }
        if(it != map.end()) {
            cloudsThatCanBeLocalized.push_back(it - map.begin() + 1);
        }

        it += 2;
        // Find points that are forward and localized.
        while(it < map.end() &&
                toleranceEllipseCalculator.readingCanBeLocalizedByAnchorPoint(*it, anchorPoint)) {
            cloudsThatCanBeLocalized.push_back(it++ - map.begin());
        }

        // Find point that are behind and localized.
        it = map.begin() + anchorPointIndex - 2;
        while(it >= map.begin() &&
                toleranceEllipseCalculator.readingCanBeLocalizedByAnchorPoint(*it, anchorPoint)) {
            cloudsThatCanBeLocalized.push_back(it-- - map.begin());
        }

        return cloudsThatCanBeLocalized;
    }

    void MapOptimizer::saveGraph(std::ostream& os) const {
        localizabilityGraph->save(os);
    }

    MapOptimizer::~MapOptimizer() {
        delete localizabilityGraph;
    }
}
