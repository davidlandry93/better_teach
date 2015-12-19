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
        ToleranceEllipseCalculator(Ellipse<T> ellipse, T maxConvergenceError, PointMatcherService<T> const& pointMatcherService);
        bool readingCanBeLocalizedByAnchorPoint(LocalisedPointCloud& reading, LocalisedPointCloud& anchorPoint);

    private:
        static const int N_SEGMENTS = 30;

        Ellipse<T> toleranceEllipse;
        T maxConvergenceError;
        PointMatcherService<T> pointMatcherService;

        void printErrors(std::vector<T> errors);
    };



    template <class T>
    ToleranceEllipseCalculator<T>::ToleranceEllipseCalculator(Ellipse<T> ellipse, T maxConvergenceError, PointMatcherService<T> const& pointMatcherService) :
            toleranceEllipse(ellipse), maxConvergenceError(maxConvergenceError), pointMatcherService(pointMatcherService) {
        this->pointMatcherService = pointMatcherService;
    }

    template <class T>
    bool ToleranceEllipseCalculator<T>::readingCanBeLocalizedByAnchorPoint(LocalisedPointCloud& reading, LocalisedPointCloud& anchorPoint) {
        ConvergenceDistanceFunction<T> f(reading, anchorPoint, pointMatcherService);

        float delta = 2 * M_PI / N_SEGMENTS;
        std::vector<T> convergenceDistances;
        for(int i = 0; i < N_SEGMENTS; i++) {
            Point<T> pointOnEllipse = toleranceEllipse.curve(i*delta);

            Transform inducedError(pointOnEllipse.toVector());

            T convergenceDistance = f(inducedError);
            std::cout << convergenceDistance << std::endl;
            convergenceDistances.push_back(convergenceDistance);
        }

        for(auto distance : convergenceDistances) {
            if (distance > maxConvergenceError) return false;
        }

        return true;
    }

    template <class T>
    void ToleranceEllipseCalculator<T>::printErrors(std::vector<T> errors) {
        for(int i = 0; i < errors.size(); i++) {
            std::cout << errors.at(i) << std::endl;
        }
    }
}

#endif //BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H
