#ifndef BETTERTEACH_TEACHREPEAT_MAP_OPTIMIZER_H
#define BETTERTEACH_TEACHREPEAT_MAP_OPTIMIZER_H

#include <vector>
#include <list>
#include "teach_repeat_map.h"
#include "ellipse.h"
#include "tolerance_ellipse_calculator.h"

namespace TeachRepeat {

    class MapOptimizer {
    public:
        MapOptimizer(ToleranceEllipseCalculator<float>& toleranceEllipseCalculator);
        void optimize(Map& map);

    private:
        ToleranceEllipseCalculator<float> toleranceEllipseCalculator;

        std::vector<int> cloudsLocalizedByAnchorPoint(Map& map, int anchorPointIndex);
    };
}




#endif //BETTERTEACH_TEACHREPEAT_MAP_OPTIMIZER_H
