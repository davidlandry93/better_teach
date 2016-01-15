
namespace TeachRepeat {

    template <typename T>
    PointMatcherService<T>::PointMatcherService(int nOfThreads) : work(ioService) {
        initService(nOfThreads);
    }

    template <typename T>
    PointMatcherService<T>::PointMatcherService(const PointMatcherService& otherService) : work(ioService) {
        initService(otherService.threadPool.size());
    }

    template <typename T>
    PointMatcherService<T>::~PointMatcherService() {
        ioService.stop();
        threadPool.join_all();
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
    void PointMatcherService<T>::icp(const LocalisedPointCloud& reading, const LocalisedPointCloud& reference, Transform* result) {
        ioService.post(boost::bind(&PointMatcherService<T>::icpWorker, *this, reading, reference, result));
    }

    template <typename T>
    void PointMatcherService<T>::icp(const LocalisedPointCloud &reading, const LocalisedPointCloud &reference, const Transform preTransform, Transform* result) {
        ioService.post(boost::bind(&PointMatcherService<T>::icpWorker, *this, reading, reference, preTransform, result));
    }

    template <typename T>
    void PointMatcherService<T>::icpWorker(const LocalisedPointCloud& reading, const LocalisedPointCloud& reference, Transform* result) {
        TP icpResult;
        try {
            icpResult = icpEngine(reading.getCloud(), reference.getCloud());
        } catch (ConvergenceError e) {
            std::cout << e.what() << std::endl;
            throw IcpException();
        }

        std::cout << "Writing results";
        result->transform(Transform(icpResult));
    }

    template <typename T>
    void PointMatcherService<T>::icpWorker(const LocalisedPointCloud &reading, const LocalisedPointCloud &reference, const Transform preTransform, Transform* result) {
        TP icpResult;
        try {
            icpResult = icpEngine(reading.getCloud(), reference.getCloud(), preTransform.pmTransform());
        } catch (ConvergenceError e) {
            std::cout << e.what() << std::endl;
            throw IcpException();
        }

        std::cout << "Writing results";
        result->transform(Transform(icpResult));
    }

    template <typename T>
    void PointMatcherService<T>::waitUntilDone() {
        work.~work();
        threadPool.join_all();
    }
}
