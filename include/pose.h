
#ifndef POSE_H
#define POSE_H

#include <iostream>
#include <Eigen/Geometry>

namespace TeachRepeat
{

class Pose {
 public:
  Pose();
  Pose(Eigen::Vector3f, Eigen::Quaternionf);
  Pose(std::string input);
  friend std::ostream& operator<<(std::ostream& out, Pose& pose);

  void setVector(Eigen::Vector3f vector);
  Eigen::Vector3f getVector();
  void setRotation(Eigen::Quaternionf quaternion);
  Eigen::Quaternionf getRotation();

 private:
  Eigen::Vector3f mVector;
  Eigen::Quaternionf mRotation;
};

}

#endif
