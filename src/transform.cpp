
#include "transform.h"

namespace TeachRepeat
{

  typedef PointMatcher<float> PM;

  Transform::Transform() : mTransform(Eigen::Affine3f::Identity())
  {

  }
  
  Transform::Transform(Eigen::Quaternionf rotation) :
    mTransform(Eigen::Affine3f(rotation))
  {
  }

  Transform::Transform(Eigen::Vector3f translation) :
    mTransform(Eigen::Affine3f(Eigen::Translation3f(translation)))
  {
  }

  
  Transform::Transform(Eigen::Vector3f translation, Eigen::Quaternionf rotation)
  {
    Transform();

    mTransform.translate(translation);
    mTransform.rotate(rotation);
  }

  Transform::Transform(const PM::TransformationParameters pmTransform) :
    mTransform(Eigen::Affine3f(Eigen::Matrix4f(pmTransform.template cast<float>())))
  {
  }

  Transform::Transform(const Eigen::Affine3f eigenTransform) :
    mTransform(eigenTransform)
  {
  }

  Eigen::Quaternionf Transform::rotationPart()
  {
    return Eigen::Quaternionf(mTransform.rotation());
  }
  
  Eigen::Vector3f Transform::translationPart()
  {
    return Eigen::Vector3f(mTransform.translation());
  }
  
  std::ostream& operator<<(std::ostream& out, Transform& t)
  {
    out << "Translation." << std::endl << t.translationPart() << std::endl;
    out << "Rotation." << std::endl << Transform::quatToString(t.rotationPart()) << std::endl;
    return out;
  }

  std::string Transform::quatToString(Eigen::Quaternionf quat)
  {
    std::stringstream ss;
    ss << "[x: " << quat.x() << ", y: " << quat.y() << ", z: " << quat.z()
       << ", w: " << quat.w() << "]" << std::endl;
    return ss.str();
  }

  PointMatcher<float>::TransformationParameters Transform::pmTransform() const
  {
    PointMatcher<float>::TransformationParameters pmTransform =
      PointMatcher<float>::TransformationParameters::Identity(4,4);

    for(int i=0; i < 4; i++)
      {
        for(int j=0; j < 4; j++)
          {
            pmTransform(i,j) = mTransform(i,j);
          }
      }

    return pmTransform;
  }

  Transform operator*(Transform lhs, const Transform& rhs)
  {
    return Transform(lhs.mTransform * rhs.mTransform);
  }

  Transform Transform::inverse()
  {
    return Transform(mTransform.inverse());
  }

} // Namespace TeachRepeat.
