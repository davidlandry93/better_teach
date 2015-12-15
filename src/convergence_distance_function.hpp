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
        ConvergenceDistanceFunction(LocalisedPointCloud reference, LocalisedPointCloud reading, std::string icpConfigFilename);
        T operator()(Transform inducedError);
        bool ellipseWithinConvergenceBassin(Ellipse<T> ellipse, T maxConvergenceDistance);

    private:
        Transform do_icp(const DP& reading, const DP& ref, const Transform preTransform);


        static const T ELLIPSE_SAMPLE_STEP = 0.05;

        LocalisedPointCloud reference;
        LocalisedPointCloud reading;
        Transform tFromRefToReading;
        Transform preciseReadingPosition;
        ICP icpEngine;
    };

    class IcpException : public std::exception {

    };

    template <class T>
    ConvergenceDistanceFunction<T>::ConvergenceDistanceFunction(LocalisedPointCloud reference,
                                                             LocalisedPointCloud reading, std::string icpConfigFilename) :
        reference(reference), reading(reading) {
        std::ifstream ifs(icpConfigFilename.c_str());

        icpEngine = typename PointMatcher<T>::ICP();
        icpEngine.loadFromYaml(ifs);

        Eigen::Matrix<T,3,1> vectorFromRefToReading = reading.getPosition().getVector() - reference.getPosition().getVector();
        tFromRefToReading = Transform(vectorFromRefToReading);

        Transform tFromRoughEstimateToLocalisation = do_icp(reference.getCloud(), reading.getCloud(), tFromRefToReading);
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

        Transform icpResult = do_icp(reading.getCloud(), reference.getCloud(), preTransform);

        std::cout << icpResult << std::endl;

        Transform computedPositionOfReading = tFromRefToReading * icpResult;

        return (computedPositionOfReading.translationPart() - preciseReadingPosition.translationPart()).squaredNorm();
    }


    template <class T>
    Transform ConvergenceDistanceFunction<T>::do_icp(const DP& reading, const DP& reference, const Transform preTransform) {

        Transformation* rigidTrans;
        rigidTrans = PointMatcher<T>::get().REG(Transformation).create("RigidTransformation");

        TP pmTransform = preTransform.pmTransform();
        if (!rigidTrans->checkParameters(pmTransform)) {
            std::cout <<
            "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis"
            << std::endl;
            rigidTrans->correctParameters(pmTransform);
        }

        DP transformedReference = rigidTrans->compute(reference, pmTransform);

        TP icpResult;
        try {
            icpResult = icpEngine(reading, transformedReference);
        } catch (ConvergenceError e) {
            std::cout << e.what() << std::endl;
            throw IcpException();
        }

        return Transform(icpResult);
    }
}

#endif //BETTERTEACH_CONVERGENCE_DISTANCE_FUNCTION_H
