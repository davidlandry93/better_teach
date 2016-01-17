#include "gtest/gtest.h"
#include "ellipsoid.h"

using namespace TeachRepeat;

class EllipsoidTest : public ::testing::Test {

protected:
    Ellipsoid<float>* ellipsoid;

    virtual void SetUp() {
        ellipsoid = new Ellipsoid<float>(1.0, 1.0, 1.0);
    }

    virtual void TearDown() {
        delete ellipsoid;
    }
};

TEST_F(EllipsoidTest, parametrizationTest) {
    Point<float> point = ellipsoid->stereographicParametrization(0.0, 0.0);

    ASSERT_FLOAT_EQ(1.0, point.getX());
    ASSERT_FLOAT_EQ(0.0, point.getY());
    ASSERT_FLOAT_EQ(0.0, point.getZ());

    point = ellipsoid->stereographicParametrization(1.0, 1.0);
}