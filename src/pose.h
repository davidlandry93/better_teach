#ifndef POSE_H
#define POSE_H

#include <iostream>
#include <Eigen/Geometry>

#include "transform.h"

namespace TeachRepeat {

    class Pose {
    public:
        Pose();
        Pose(Eigen::Vector3f, Eigen::Quaternionf);
        Pose(std::string input);
        Transform transFromPose(const Pose &otherPose) const;
        void transform(Transform transform);
        friend std::ostream &operator<<(std::ostream &out, Pose &pose);
        void setVector(Eigen::Vector3f vector);
        Eigen::Vector3f getVector() const;
        void setRotation(Eigen::Quaternionf quaternion);
        Eigen::Quaternionf getRotation() const;

        static Pose origin();

    private:
        Eigen::Vector3f mVector;
        Eigen::Quaternionf mRotation;
    };

}

#endif
