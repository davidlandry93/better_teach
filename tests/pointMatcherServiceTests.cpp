
#include "gtest/gtest.h"
#include "pointmatcherservice.h"
#include "localised_point_cloud.h"

using namespace TeachRepeat;

class PointMatcherServiceTest : public ::testing::Test {
protected:
    LocalisedPointCloud cloud1 = LocalisedPointCloud("aPointCloud.vtk", Pose::origin());
    LocalisedPointCloud cloud2 = LocalisedPointCloud("aPointCloud.vtk", Pose::origin());

    PointMatcherService<float>* service = new PointMatcherService<float>(4);

    virtual void SetUp() {
        cloud1.loadFromDisk("res/");
        cloud2.loadFromDisk("res/");
    }

    virtual void TearDown() {
        delete service;
    }
};

TEST_F(PointMatcherServiceTest, sameCloudTest) {
    Transform icpResult;
    service->icp(cloud1, cloud2, icpResult);
    service->waitUntilDone();

    ASSERT_TRUE(icpResult.isApproxEqual(Transform::identity(), 0.01));
}

TEST_F(PointMatcherServiceTest, initialGuessTest) {
    Eigen::Vector3f translation;
    translation << 1.0, 0.0, 0.0;
    Transform transform = Transform(translation);

    cloud1.transform(transform);

    Transform icpResult;
    service->icp(cloud2, cloud1, transform, icpResult);
    service->waitUntilDone();

    ASSERT_TRUE(icpResult.isApproxEqual(transform, 0.01));
}