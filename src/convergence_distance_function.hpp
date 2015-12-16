#ifndef BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
#define BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H

#include <exception>
#include <string>
#include <vector>
#include <Eigen/Geometry>

#include "pointmatcher/PointMatcher.h"
#include "localised_point_cloud.h"
#include "point.hpp"
#include "transform.h"
#include "tolerance_ellipse_calculator.hpp"
#include "pointmatcherservice.h"

namespace TeachRepeat {

    template <class T>
    class ConvergenceDistanceFunction {
        typedef PointMatcher<T> PM;
        typedef typename PointMatcher<T>::DataPoints DP;
        typedef typename PointMatcher<T>::TransformationParameters TP;
        typedef typename PointMatcher<T>::ICP ICP;
        typedef typename PointMatcher<T>::Transformation Transformation;
        typedef typename PointMatcher<T>::ConvergenceError ConvergenceError;

    public:
        ConvergenceDistanceFunction(LocalisedPointCloud reference, LocalisedPointCloud reading, const PointMatcherService<float>& pointMatcherService);
        T operator()(Transform inducedError);
        bool ellipseWithinConvergenceBassin(Ellipse<T> ellipse, T maxConvergenceDistance);

    private:
        static const T ELLIPSE_SAMPLE_STEP = 0.05;

        LocalisedPointCloud reference;
        LocalisedPointCloud reading;
        Transform tFromRefToReading;
        Transform preciseReadingPosition;
        PointMatcherService<float> pointMatcherService;
    };

    template <class T>
    ConvergenceDistanceFunction<T>::ConvergenceDistanceFunction(LocalisedPointCloud reference,
                                                                LocalisedPointCloud reading, const PointMatcherService<float>& pointMatcherService) :
        reference(reference), reading(reading) {
        this->pointMatcherService = pointMatcherService;

        Eigen::Matrix<T,3,1> vectorFromRefToReading = reading.getPosition().getVector() - reference.getPosition().getVector();
        tFromRefToReading = Transform(vectorFromRefToReading);

        Transform tFromRoughEstimateToLocalisation = pointMatcherService.icp(reference, reading, tFromRefToReading);
        preciseReadingPosition = tFromRoughEstimateToLocalisation * tFromRefToReading;
    }

    template <class T>
    bool ConvergenceDistanceFunction<T>::ellipseWithinConvergenceBassin(Ellipse<T> ellipse, T maxConvergenceDistance) {
        std::vector< Point<T> > samplePoints;

        for(T theta = 0.0; theta < (T) M_PI; theta += ELLIPSE_SAMPLE_STEP) {
            Point<T> pointOnEllipse = ellipse.curve(theta);
        }
    }

    template <class T>
    T ConvergenceDistanceFunction<T>::operator()(Transform inducedError) {
        Transform preTransform = tFromRefToReading * inducedError;
        std::cout << "InducedError" << std::endl;
        std::cout << inducedError << std::endl;

        Transform icpResult = pointMatcherService.icp(reading, reference, preTransform);

        std::cout << "IcpResult" << std::endl;
        std::cout << icpResult << std::endl;

        std::cout << "---" << std::endl;

        Transform computedPositionOfReading = preTransform * icpResult;

        return (computedPositionOfReading.translationPart() - preciseReadingPosition.translationPart()).squaredNorm();
    }
}

#endif //BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
