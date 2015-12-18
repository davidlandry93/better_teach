
#include "gtest/gtest.h"
#include "pointmatcherservice.h"
#include "localised_point_cloud.h"

using namespace TeachRepeat;

TEST(pointMatcherServiceTest, initialTransformationTest) {
    LocalisedPointCloud reading = LocalisedPointCloud("aPointCloud.vtk", Pose::origin());
    reading.loadFromDisk("res/");

    LocalisedPointCloud reference = LocalisedPointCloud("aPointCloud.vtk", Pose::origin());
    reading.loadFromDisk("res/");

    PointMatcherService<float> service;

    Eigen::Vector3f translation;
    translation << 0.5, 0.0, 0.0;
    Transform preTransform = Transform(translation);
    Transform icpResult = service.icp(reading, reference, preTransform);

    ASSERT_TRUE(icpResult.isApproxEqual(preTransform.inverse(), 0.01));
}