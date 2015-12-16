
#include "pointmatcherservice.h"

namespace TeachRepeat {

    template<typename T>
    PointMatcherService<T>::PointMatcherService() {
        icpEngine = typename PointMatcher<T>::ICP();
    }

    template<typename T>
    void PointMatcherService<T>::loadConfigFile(std::string pathToConfig) {
        std::ifstream ifs(pathToConfig.c_str());

        icpEngine.loadFromYaml(ifs);
    }

    template<typename T>
    Transform PointMatcherService<T>::icp(const LocalisedPointCloud &reading,
                                          const LocalisedPointCloud &reference,
                                          const Transform preTransform) const {
        Transformation *rigidTrans;
        rigidTrans = PointMatcher<T>::get().REG(Transformation).create("RigidTransformation");

        TP pmTransform = preTransform.pmTransform();
        if (!rigidTrans->checkParameters(pmTransform)) {
            std::cout <<
            "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis"
            << std::endl;
            rigidTrans->correctParameters(pmTransform);
        }

        DP transformedReference = rigidTrans->compute(reference.getCloud(), pmTransform);

        TP icpResult;
        try {
            icpResult = icpEngine(reading.getCloud(), transformedReference);
        } catch (ConvergenceError e) {
            std::cout << e.what() << std::endl;
            throw IcpException();
        }

        return Transform(icpResult);
    }

    template<typename T>
    void PointMatcherService<T>::savePointCloud(const TeachRepeat::LocalisedPointCloud &pointCloud,
                                             const std::string destination) const {

    }

}