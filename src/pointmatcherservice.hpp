
namespace TeachRepeat {

    template <typename T>
    PointMatcherService<T>::PointMatcherService(int nOfThreads) : work(nullptr) {
        initService(nOfThreads);
    }

    template <typename T>
    PointMatcherService<T>::PointMatcherService(const PointMatcherService& otherService) : work(nullptr) {
        initService(otherService.threadPool.size());
    }

    template <typename T>
    PointMatcherService<T>::~PointMatcherService() {
        delete work;
        ioService.stop();
        threadPool.join_all();
    }

    template <typename T>
    void PointMatcherService<T>::initService(int nOfThreads) {
        icpEngine = PointMatcher<float>::ICP();
        icpEngine.setDefault();

        work = new boost::asio::io_service::work(ioService);

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
        incrementJobsNotDoneCounter();

        TP icpResult;
        try {
            icpResult = icpEngine(reading.getCloud(), reference.getCloud());
        } catch (ConvergenceError e) {
            std::cout << e.what() << std::endl;
            throw IcpException();
        }
        result->transform(Transform(icpResult));

        decrementJobsNotDoneCounter();
    }

    template <typename T>
    void PointMatcherService<T>::icpWorker(const LocalisedPointCloud &reading, const LocalisedPointCloud &reference, const Transform preTransform, Transform* result) {
        incrementJobsNotDoneCounter();

        TP icpResult;
        try {
            icpResult = icpEngine(reading.getCloud(), reference.getCloud(), preTransform.pmTransform());
        } catch (ConvergenceError e) {
            std::cout << e.what() << std::endl;
            throw IcpException();
        }

        result->transform(Transform(icpResult));

        decrementJobsNotDoneCounter();
    }

    template <typename T>
    void PointMatcherService<T>::waitUntilDone() {
        delete work;
        ioService.post(boost::bind(&PointMatcherService::allJobsDoneMonitor, *this));
        threadPool.join_all();
        work = new boost::asio::io_service::work(ioService);
    }

    template <typename T>
    void PointMatcherService<T>::restart() {
        //ioService.reset();
    }

    template <typename T>
    void PointMatcherService<T>::allJobsDoneMonitor() {
        while(true) {
            jobCounterMutex.lock();
            if(jobsBeingDone == 0) {
                ioService.stop();
                jobCounterMutex.unlock();
                return;
            } else {
                jobCounterMutex.unlock();
            }
        }
    }

    template <typename T>
    void PointMatcherService<T>::incrementJobsNotDoneCounter() {
        jobCounterMutex.lock();
        jobsBeingDone++;
        std::cout << jobsBeingDone << std::endl;
        jobCounterMutex.unlock();
    }

    template <typename T>
    void PointMatcherService<T>::decrementJobsNotDoneCounter() {
        jobCounterMutex.lock();
        jobsBeingDone--;
        std::cout << jobsBeingDone << std::endl;
        jobCounterMutex.unlock();
    }
}
