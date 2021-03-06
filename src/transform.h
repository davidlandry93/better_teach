#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <iostream>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "pointmatcher/PointMatcher.h"

namespace TeachRepeat {

    class Transform {
    public:
        Transform();
        Transform(Eigen::Quaternionf rotation);
        Transform(Eigen::Vector3f translation);
        Transform(Eigen::Vector3f translation, Eigen::Quaternionf rotation);
        Transform(Eigen::Vector4f translation);
        Transform(const PointMatcher<float>::TransformationParameters pmTransform);
        Transform(const Eigen::Affine3f eigenTransform);

        PointMatcher<float>::TransformationParameters pmTransform() const;
        Eigen::Quaternionf rotationPart();
        Eigen::Vector3f translationPart();
        Eigen::Affine3f matrix();
        Transform inverse();
        bool isApproxEqual(const Transform otherTransform, float epsilon) const;
        void transform(const Transform& otherTransform);

        static std::string quatToString(Eigen::Quaternionf quat);
        friend std::ostream &operator<<(std::ostream &out, Transform &t);
        friend Transform operator*(Transform lhs, const Transform &rhs);

        static Transform identity();

    private:
        Eigen::Affine3f mTransform;
    };
} // Namespace TeachRepeat.

#endif
