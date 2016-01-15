
#ifndef BETTERTEACH_POINTMATCHERSERVICE_H
#define BETTERTEACH_POINTMATCHERSERVICE_H

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

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
    void loadConfigFile(std::string pathToConfig);
    Transform icp(const LocalisedPointCloud& reading, const LocalisedPointCloud& reference);
    Transform icp(const LocalisedPointCloud& reading, const LocalisedPointCloud& reference, const Transform preTransform);

private:
    typename PointMatcher<float>::ICP icpEngine;
    boost::asio::io_service ioService;
    boost::thread_group threadPool;

    void initService(int nOfThreads);
};

class IcpException : public std::exception {

};

}

#include "pointmatcherservice.hpp"

#endif
