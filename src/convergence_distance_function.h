#ifndef BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
#define BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H

#include <exception>
#include <string>
#include <vector>
#include <tuple>
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
        ConvergenceDistanceFunction(LocalisedPointCloud &reading, LocalisedPointCloud &reference, PointMatcherService<T>& pointMatcherService);
        std::tuple<T,T> operator()(Transform inducedError);

    private:
        static int nCalls;

        LocalisedPointCloud reference;
        LocalisedPointCloud reading;
        Transform tFromReadingToRef;
        PointMatcherService<T> pointMatcherService;
    };

    template <typename T>
    int ConvergenceDistanceFunction<T>::nCalls = 0;

    template <typename T>
    ConvergenceDistanceFunction<T>::ConvergenceDistanceFunction(LocalisedPointCloud& reading,
                                                                LocalisedPointCloud& reference,
                                                                PointMatcherService<T>& pointMatcherService) :
        reading(reading), reference(reference), pointMatcherService(pointMatcherService) {
        tFromReadingToRef = reading.getPosition().transFromPose(reference.getPosition());

        Transform tFromRoughEstimateToLocalisation = pointMatcherService.icp(reading, reference, tFromReadingToRef);
    }

    template <typename T>
    std::tuple<T,T> ConvergenceDistanceFunction<T>::operator()(Transform inducedError) {
        Transform preTransform = inducedError * tFromReadingToRef;
        Transform icpResult = pointMatcherService.icp(reading, reference, preTransform);

        // reference.saveToDisk("", "ref" + std::to_string(nCalls) + ".vtk");
        // reading.transform(icpResult);
        // reading.saveToDisk("", "res" + std::to_string(nCalls++) + ".vtk");
        // reading.transform(icpResult.inverse());

        // Print some output about how far we are from converging.
        // std::cout << "Vector " << (icpResult.translationPart() - tFromReadingToRef.translationPart()).norm();
        // std::cout << "Frobenius " << (icpResult.matrix().matrix() - tFromReadingToRef.matrix().matrix()).norm() << std::endl;

        auto result = std::make_tuple(
                (icpResult.translationPart() - tFromReadingToRef.translationPart()).norm(),
                icpResult.rotationPart().angularDistance(tFromReadingToRef.rotationPart())
        );

        std::cout << std::get<0>(result) << ", " << std::get<1>(result) << std::endl;

        return result;
    }
}

#endif //BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
