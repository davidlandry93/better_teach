#ifndef BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
#define BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H

#include <Eigen/Geometry>

#include "pointmatcher/PointMatcher.h"
#include "localised_point_cloud.h"
#include "transform.h"

namespace TeachRepeat {

    template <class T>
    class ConvergenceDistanceFunction {
        typedef PointMatcher<T> PM;
        typedef typename PointMatcher<T>::DataPoints DP;
        typedef typename PointMatcher<T>::TransformationParameters TP;
        typedef typename PointMatcher<T>::ICP ICP;
        typedef typename PointMatcher<T>::Transformation Transformation;

    public:
        ConvergenceDistanceFunction(LocalisedPointCloud reference, LocalisedPointCloud reading,  Transform preciseReadingPosition, ICP icpEngine);
        T operator()(Transform inducedError);

    private:
        Transform do_icp(const DP& reading, const DP& ref, const Transform preTransform);

        LocalisedPointCloud reference;
        LocalisedPointCloud reading;
        Transform tFromRefToReading;
        Transform preciseReadingPosition;
        ICP icpEngine;
    };

}

#endif //BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
