
#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <iostream>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "pointmatcher/PointMatcher.h"

namespace TeachRepeat
{
  
  class Transform
  {
  public:
    Transform();
    Transform(Eigen::Quaternionf rotation);
    Transform(Eigen::Vector3f translation);
    Transform(Eigen::Vector3f translation, Eigen::Quaternionf rotation);
    Transform(const PointMatcher<float>::TransformationParameters pmTransform); 
    Transform(const Eigen::Affine3f eigenTransform);

    PointMatcher<float>::TransformationParameters pmTransform();
    Eigen::Quaternionf rotationPart();
    Eigen::Vector3f translationPart();

    static std::string quatToString(Eigen::Quaternionf quat);
    friend std::ostream& operator<<(std::ostream& out, Transform& t);

    friend Transform operator*(const Transform& lhs, const Transform& rhs);

  private:
    Eigen::Affine3f mTransform;
  };
} // Namespace TeachRepeat.

#endif
