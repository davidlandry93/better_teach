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
#include "pointmatcherservice.hpp"

namespace TeachRepeat {

    template <typename T>
    class ConvergenceDistanceFunction {

    public:
        ConvergenceDistanceFunction(LocalisedPointCloud reference, LocalisedPointCloud reading, PointMatcherService<T>& pointMatcherService);
        float operator()(Transform inducedError);

    private:
        static const T ELLIPSE_SAMPLE_STEP = 0.05;

        LocalisedPointCloud reference;
        LocalisedPointCloud reading;
        Transform tFromRefToReading;
        Transform preciseReadingPosition;
        PointMatcherService<T> pointMatcherService;
    };

    template <typename T>
    ConvergenceDistanceFunction<T>::ConvergenceDistanceFunction(LocalisedPointCloud reference,
                                                             LocalisedPointCloud reading,
                                                             PointMatcherService<T>& pointMatcherService) :
        reference(reference), reading(reading) {
        this->pointMatcherService = pointMatcherService;

        Eigen::Matrix<T,3,1> vectorFromRefToReading = reading.getPosition().getVector() - reference.getPosition().getVector();
        tFromRefToReading = Transform(vectorFromRefToReading);

        Transform tFromRoughEstimateToLocalisation = pointMatcherService.icp(reference, reading, tFromRefToReading);
        preciseReadingPosition = tFromRoughEstimateToLocalisation * tFromRefToReading;
    }

    template <typename T>
    float ConvergenceDistanceFunction<T>::operator()(Transform inducedError) {
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
