#ifndef BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H
#define BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H

#include <math.h>
#include <string>

#include "pointmatcher/PointMatcher.h"
#include "localised_point_cloud.h"
#include "ellipse.hpp"
#include "convergence_distance_function.hpp"

namespace TeachRepeat {

    template <class T>
    class ToleranceEllipseCalculator {
        typedef typename PointMatcher<T>::ICP ICP;

    public:
        ToleranceEllipseCalculator(T maxConvergenceError, std::string icpConfigFilename);
        Ellipse<T> calculate(LocalisedPointCloud reference, LocalisedPointCloud reading);

    private:
        static const int N_SEGMENTS = 100;

        T maxConvergenceError;
        std::string icpConfigFilename;
    };


    template <class T>
    ToleranceEllipseCalculator<T>::ToleranceEllipseCalculator(T maxConvergenceError, std::string icpConfigFilename) :
            maxConvergenceError(maxConvergenceError), icpConfigFilename(icpConfigFilename) {

    }

    template <class T>
    Ellipse <T> ToleranceEllipseCalculator<T>::calculate(LocalisedPointCloud reference, LocalisedPointCloud reading) {
        ConvergenceDistanceFunction<T> f(reference, reading, icpConfigFilename);

        Ellipse<float> currentEllipse(1.0, 2.0);

        float delta = 2 * M_PI / N_SEGMENTS;
        std::vector<T> convergenceDistances();
        for(int i = 0; i < N_SEGMENTS; i++) {
            Eigen::Matrix<T,2,1> pointOnEllipse = currentEllipse.curve(i*delta);
            Eigen::Matrix<T,3,1> pointInSpace(pointOnEllipse(0), pointOnEllipse(1), 0.0);

            Transform inducedError(pointInSpace);
            T convergenceDistance = f(inducedError);

            // TODO: Actually compute something with the convergenceDistance
        }

        return currentEllipse;
    }

}

#endif //BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H
