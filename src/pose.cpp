
#include <vector>

#include "pose.h"

namespace TeachRepeat
{

  Pose::Pose()
  {

  }

  Pose::Pose(Eigen::Vector3f vector, Eigen::Quaternionf quaternion) :
    mVector(vector), mRotation(quaternion)
  {

  }


  Pose::Pose(std::string input)
  {
    std::stringstream ss(input);
    std::string buffer;
    std::vector<double> lineValues;

    for(int i = 0; i < 7; i++)
      {
        std::getline(ss, buffer, ',');
        lineValues.push_back(strtod(buffer.c_str(), NULL));
      }

    mVector = Eigen::Vector3f(lineValues[0], lineValues[1], lineValues[2]);
    mRotation = Eigen::Quaternionf(lineValues[3], lineValues[4], lineValues[5], lineValues[6]);
  }

  Eigen::Vector3f Pose::getVector() const
  {
    return mVector;
  }

  void Pose::setVector(Eigen::Vector3f vector)
  {
    mVector = vector;
  }

  void Pose::setRotation(Eigen::Quaternionf quaternion)
  {
    mRotation = quaternion;
  }

  Eigen::Quaternionf Pose::getRotation() const
  {
    return mRotation;
  }

  std::ostream& operator<<(std::ostream& out, Pose& pose)
  {
    out << pose.mVector << "[" << pose.mRotation.x() << "," <<
      pose.mRotation.y() << "," << pose.mRotation.z() << "," <<
      pose.mRotation.w() << "]" << std::endl;
    return out;
  }

  
  Transform Pose::transFromPose(const Pose& otherPose) const
  {
    Eigen::Vector3f translationVector =
      mVector - otherPose.getVector();
  
    Eigen::Quaternionf rotationQuaternion =
      mRotation * otherPose.getRotation().conjugate();
    
    return Transform(translationVector, rotationQuaternion);
  }


}
