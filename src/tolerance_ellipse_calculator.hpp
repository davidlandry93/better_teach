#ifndef BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H
#define BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H

#include <math.h>
#include <string>
#include <exception>

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

        void printErrors(std::vector<T> errors);
    };



    template <class T>
    ToleranceEllipseCalculator<T>::ToleranceEllipseCalculator(T maxConvergenceError, std::string icpConfigFilename) :
            maxConvergenceError(maxConvergenceError), icpConfigFilename(icpConfigFilename) {

    }

    template <class T>
    Ellipse <T> ToleranceEllipseCalculator<T>::calculate(LocalisedPointCloud reference, LocalisedPointCloud reading) {
        PointMatcherService<float> pmService;
        pmService.loadConfigFile(icpConfigFilename);

        ConvergenceDistanceFunction<T> f(reference, reading, pmService);

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
