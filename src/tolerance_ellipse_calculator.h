#ifndef BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H
#define BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H

#include <math.h>
#include <string>
#include <exception>

#include "pointmatcher/PointMatcher.h"
#include "localised_point_cloud.h"
#include "ellipse.h"
#include "convergence_distance_function.h"

namespace TeachRepeat {

    template <class T>
    class ToleranceEllipseCalculator {
        typedef typename PointMatcher<T>::ICP ICP;

    public:
        ToleranceEllipseCalculator(T maxConvergenceError, PointMatcherService<T> const& pointMatcherService);
        Ellipse<T> calculate(LocalisedPointCloud reference, LocalisedPointCloud reading);

    private:
        static const int N_SEGMENTS = 100;

        T maxConvergenceError;
        std::string icpConfigFilename;
        PointMatcherService<T> pointMatcherService;

        void printErrors(std::vector<T> errors);
    };



    template <class T>
    ToleranceEllipseCalculator<T>::ToleranceEllipseCalculator(T maxConvergenceError, PointMatcherService<T> const& pointMatcherService) :
            maxConvergenceError(maxConvergenceError), icpConfigFilename(icpConfigFilename) {
        this->pointMatcherService = pointMatcherService;
    }

    template <class T>
    Ellipse <T> ToleranceEllipseCalculator<T>::calculate(LocalisedPointCloud reference, LocalisedPointCloud reading) {
        ConvergenceDistanceFunction<T> f(reference, reading, pointMatcherService);

        Ellipse<float> currentEllipse(0.5, 1.0);

        float delta = 2 * M_PI / N_SEGMENTS;
        std::vector<T> convergenceDistances;
        for(int i = 0; i < N_SEGMENTS; i++) {
            Point<T> pointOnEllipse = currentEllipse.curve(i*delta);

            Transform inducedError(pointOnEllipse.toVector());

            T convergenceDistance = f(inducedError);
            convergenceDistances.push_back(convergenceDistance);
        }

        printErrors(convergenceDistances);

        return currentEllipse;
    }

    template <class T>
    void ToleranceEllipseCalculator<T>::printErrors(std::vector<T> errors) {
        for(int i = 0; i < errors.size(); i++) {
            std::cout << errors.at(i) << std::endl;
        }
    }
}

#endif //BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H
