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
        ConvergenceDistanceFunction(LocalisedPointCloud &reading, LocalisedPointCloud &reference, PointMatcherService<T>& pointMatcherService);
        float operator()(Transform inducedError);
        std::vector<float> sampleForTransforms(const std::vector<Transform>& transforms);

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
    }

    template <typename T>
    std::vector<float> ConvergenceDistanceFunction<T>::sampleForTransforms(const std::vector<Transform>& inducedErrors) {
        std::vector<Transform> icpResults(inducedErrors.size());

        for(int i = 0; i < inducedErrors.size(); i++) {
            Transform preTransform = inducedErrors[i] * tFromReadingToRef;
            pointMatcherService.icp(reading, reference, preTransform, icpResults[i]);
        }

        pointMatcherService.waitForQueueEmpty();

        std::vector<float> scalarDistances(inducedErrors.size());
        for(int i = 0; i < icpResults.size(); i++) {
            scalarDistances[i] = (icpResults[i].translationPart() - tFromReadingToRef.translationPart()).squaredNorm();
        }

        return scalarDistances;
    }
}

#endif //BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
