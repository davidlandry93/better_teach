#ifndef BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
#define BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H

#include <exception>
#include <string>
#include <vector>
#include <Eigen/Geometry>

#include "pointmatcher/PointMatcher.h"
#include "localised_point_cloud.h"
#include "point.h"
#include "transform.h"
#include "tolerance_ellipse_calculator.h"
#include "pointmatcherservice.h"

namespace TeachRepeat {

    template <typename T>
    class ConvergenceDistanceFunction {

    public:
        ConvergenceDistanceFunction(LocalisedPointCloud reference, LocalisedPointCloud reading, PointMatcherService<T>& pointMatcherService);
        float operator()(Transform inducedError);

    private:
        static int nCalls;

        LocalisedPointCloud reference;
        LocalisedPointCloud reading;
        Transform tFromRefToReading;
        Transform preciseReadingPosition;
        PointMatcherService<T> pointMatcherService;
    };

    template <typename T>
    int ConvergenceDistanceFunction<T>::nCalls = 0;

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
        Transform icpResult = pointMatcherService.icp(reading, reference, preTransform);

        Transform computedPositionOfReading = preTransform * icpResult;

        reference.saveToDisk("/home/david/dat/Debug/ref", std::to_string(nCalls) + ".vtk");
        reading.transform(computedPositionOfReading.inverse());
        reading.saveToDisk("/home/david/dat/Debug/result", std::to_string(nCalls++) + ".vtk");
        reading.transform(computedPositionOfReading);

        return (computedPositionOfReading.translationPart() - preciseReadingPosition.translationPart()).squaredNorm();
    }
}

#endif //BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
