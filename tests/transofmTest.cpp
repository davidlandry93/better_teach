#include "gtest/gtest.h"
#include "transform.h"

using namespace TeachRepeat;

class TransformTest : public ::testing::Test {

protected:
    Transform* transform;

    virtual void SetUp() {
        transform = new Transform();
    }

    virtual void TearDown() {
        delete transform;
    }
};

TEST_F(TransformTest, TranslationThenRotationTest) {
    Eigen::Vector3f translationVector(1.0,1.0,1.0);
    Transform translation(translationVector);

    Eigen::Quaternionf rotationQuaternion(1.0,0.0,0.0,0.0);
    Transform rotation(rotationQuaternion);

    transform->transform(translation);
    transform->transform(rotation);

    Transform reference(translationVector, rotationQuaternion);

    ASSERT_TRUE(transform->isApproxEqual(reference, 0.00001));
}