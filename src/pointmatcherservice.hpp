
namespace TeachRepeat {

    template <typename T>
    PointMatcherService<T>::PointMatcherService(int nOfThreads) {
        initService(nOfThreads);
    }

    template <typename T>
    PointMatcherService<T>::PointMatcherService(const PointMatcherService& otherService) {
        initService(otherService.threadPool.size());
    }

    template <typename T>
    void PointMatcherService<T>::initService(int nOfThreads) {
        icpEngine = PointMatcher<float>::ICP();
        icpEngine.setDefault();

        boost::asio::io_service::work work(ioService);

        for(int i = 0; i < nOfThreads; i++) {
            threadPool.create_thread(boost::bind(&boost::asio::io_service::run, &ioService));
        }
    }

    template <typename T>
    void PointMatcherService<T>::loadConfigFile(std::string pathToConfig) {
        std::ifstream ifs(pathToConfig.c_str());

        icpEngine.loadFromYaml(ifs);
    }

    template <typename T>
    Transform PointMatcherService<T>::icp(const LocalisedPointCloud& reading, const LocalisedPointCloud& reference) {
        TP icpResult;
        try {
            icpResult = icpEngine(reading.getCloud(), reference.getCloud());
        } catch (ConvergenceError e) {
            std::cout << e.what() << std::endl;
            throw IcpException();
        }

        return Transform(icpResult);
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
