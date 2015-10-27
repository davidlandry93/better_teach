#include "convergence_distance_function.h"

namespace TeachRepeat {

    template <class T>
    ConvergenceDistanceFunction<T>::ConvergenceDistanceFunction(LocalisedPointCloud reference,
                                                             LocalisedPointCloud reading,
                                                             Transform tFromRefToReading,
                                                             Transform preciseReadingPosition, ICP icpEngine) :
        reference(reference), reading(reading), tFromRefToReading(tFromRefToReading),
        preciseReadingPosition(preciseReadingPosition), icpEngine(icpEngine) {

    }

    template <class T>
    T ConvergenceDistanceFunction<T>::operator()(Transform inducedError) {
        Transform preTransform = tFromRefToReading * inducedError;
        Transform icpResult = do_icp(reading.getCloud(), reference.getCloud(), preTransform);

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

        TP icpResult = icpEngine(reading, transformedReference);

        return Transform(icpResult);
    }

}
