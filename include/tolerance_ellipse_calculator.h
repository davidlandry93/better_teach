#ifndef BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H
#define BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H

#include "pointmatcher/PointMatcher.h"
#include "localised_point_cloud.h"
#include "ellipse.h"

namespace TeachRepeat {

    template <class T>
    class ToleranceEllipseCalculator {
        typedef typename PointMatcher<T>::ICP ICP;

    public:
        ToleranceEllipseCalculator(T maxConvergenceError);
        Ellipse<T> calculate(LocalisedPointCloud reference, LocalisedPointCloud reading);

    private:
        T maxConvergenceError;
        ICP icpEngine;
    };

}

#endif //BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H
