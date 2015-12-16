
#include "pointmatcherservice.h"

namespace TeachRepeat {

    template <typename T>
    PointMatcherService<T>::PointMatcherService() {
        icpEngine = PointMatcher<float>::ICP();
    }

    template <typename T>
    void PointMatcherService<T>::loadConfigFile(std::string pathToConfig) {
        std::ifstream ifs(pathToConfig.c_str());

        icpEngine.loadFromYaml(ifs);
    }

    template <typename T>
    Transform PointMatcherService<T>::icp(const LocalisedPointCloud &reading, const LocalisedPointCloud &reference, const Transform preTransform) {
        TP icpResult;
        try {
            icpResult = icpEngine(reading.getCloud(), reference.getCloud(), preTransform.pmTransform());
        } catch (ConvergenceError e) {
            std::cout << e.what() << std::endl;
            throw IcpException();
        }

        return Transform(icpResult);
    }
}