#ifndef BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H
#define BETTERTEACH_CONVERGENCEBASSINCALCULATOR_H

#include <math.h>
#include <string>
#include <exception>

#include "pointmatcher/PointMatcher.h"
#include "localised_point_cloud.h"
#include "ellipsoid.h"
#include "convergence_distance_function.h"

namespace TeachRepeat {

    template <class T>
    class ToleranceEllipseCalculator {
        typedef typename PointMatcher<T>::ICP ICP;

    public:
        ToleranceEllipseCalculator(Ellipsoid<T> ellipsoid, T maxConvergenceError, PointMatcherService<T> const& pointMatcherService);
        bool readingCanBeLocalizedByAnchorPoint(LocalisedPointCloud& reading, LocalisedPointCloud& anchorPoint);

    private:
        static const int N_SEGMENTS = 5;

        Ellipsoid<T> toleranceEllipsoid;
        T maxConvergenceError;
        PointMatcherService<T> pointMatcherService;

        void printErrors(std::vector<T> errors);
    };



    template <class T>
    ToleranceEllipseCalculator<T>::ToleranceEllipseCalculator(Ellipsoid<T> ellipse, T maxConvergenceError, PointMatcherService<T> const& pointMatcherService) :
            toleranceEllipsoid(ellipse), maxConvergenceError(maxConvergenceError), pointMatcherService(pointMatcherService) {
        this->pointMatcherService = pointMatcherService;
    }

    template <class T>
    bool ToleranceEllipseCalculator<T>::readingCanBeLocalizedByAnchorPoint(LocalisedPointCloud& reading, LocalisedPointCloud& anchorPoint) {
        ConvergenceDistanceFunction<T> f(reading, anchorPoint, pointMatcherService);

        float d_u = 2*M_PI / N_SEGMENTS;
        float d_v = M_PI / N_SEGMENTS;

        std::cout << "New convergence function..." << std::endl;

        std::vector<T> convergenceDistances;
        for(int i = 0; i < N_SEGMENTS; i++) {
            for(int j = 0; j < N_SEGMENTS; j++)
            {
                Point<float> pointOnEllipse = toleranceEllipsoid.standardParametrization(i*d_u, j*d_v);

                Eigen::Matrix<float,3,1>  inducedTranslationVector;
                inducedTranslationVector << pointOnEllipse.getX(), pointOnEllipse.getY(), 0.0;

                Eigen::Quaternion<float> inducedRotationQuaternion( std::cos(pointOnEllipse.getZ() / 2), 0.0, 0.0, std::sin(pointOnEllipse.getZ() / 2));

                Transform inducedError(inducedTranslationVector, inducedRotationQuaternion);

                T convergenceDistance = f(inducedError);

                if(convergenceDistance > maxConvergenceError) return false;
            }
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
