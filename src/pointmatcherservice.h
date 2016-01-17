
#ifndef BETTERTEACH_POINTMATCHERSERVICE_H
#define BETTERTEACH_POINTMATCHERSERVICE_H

#include <mutex>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include "transform.h"
#include "localised_point_cloud.h"

namespace TeachRepeat {

    template <typename T>
class PointMatcherService {
    typedef PointMatcher<T> PM;
    typedef typename PointMatcher<T>::DataPoints DP;
    typedef typename PointMatcher<T>::TransformationParameters TP;
    typedef typename PointMatcher<T>::ICP ICP;
    typedef typename PointMatcher<T>::Transformation Transformation;
    typedef typename PointMatcher<T>::ConvergenceError ConvergenceError;


public:
    PointMatcherService(int nOfThreads);
    PointMatcherService(const PointMatcherService& otherService);
    ~PointMatcherService();
    void loadConfigFile(std::string pathToConfig);
    void icp(const LocalisedPointCloud& reading, const LocalisedPointCloud& reference, Transform* result);
    void icp(const LocalisedPointCloud& reading, const LocalisedPointCloud& reference, const Transform preTransform, Transform* result);
    void waitUntilDone();
    void restart();

private:
    typename PointMatcher<float>::ICP icpEngine;
    boost::asio::io_service ioService;
    boost::thread_group threadPool;
    boost::asio::io_service::work* work;
    std::mutex jobCounterMutex;
    int jobsBeingDone = 0;

    void initService(int nOfThreads);
    void icpWorker(const LocalisedPointCloud& reading, const LocalisedPointCloud& reference, Transform* result);
    void icpWorker(const LocalisedPointCloud& reading, const LocalisedPointCloud& reference, const Transform preTransform, Transform* result);
    void allJobsDoneMonitor();
    void incrementJobsNotDoneCounter();
    void decrementJobsNotDoneCounter();
};

class IcpException : public std::exception {

};

}

#include "pointmatcherservice.hpp"

#endif
