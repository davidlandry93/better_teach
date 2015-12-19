#include <vector>

#include "pose.h"

namespace TeachRepeat {

    Pose::Pose() {

    }

    Pose::Pose(Eigen::Vector3f vector, Eigen::Quaternionf quaternion) :
            mVector(vector), mRotation(quaternion) {

    }


    Pose::Pose(std::string input) {
        std::stringstream ss(input);
        std::string buffer;
        std::vector<double> lineValues;

        for (int i = 0; i < 7; i++) {
            std::getline(ss, buffer, ',');
            lineValues.push_back(strtod(buffer.c_str(), NULL));
        }

        mVector = Eigen::Vector3f(lineValues[0], lineValues[1], lineValues[2]);
        mRotation = Eigen::Quaternionf(lineValues[3], lineValues[4], lineValues[5], lineValues[6]);
    }

    Eigen::Vector3f Pose::getVector() const {
        return mVector;
    }

    void Pose::setVector(Eigen::Vector3f vector) {
        mVector = vector;
    }

    void Pose::setRotation(Eigen::Quaternionf quaternion) {
        mRotation = quaternion;
    }

    Eigen::Quaternionf Pose::getRotation() const {
        return mRotation;
    }

    std::ostream& operator<<(std::ostream &out, Pose &pose) {
        out << pose.mVector[0] << ","
            << pose.mVector[1] << ","
            << pose.mVector[2] << ","
            << pose.mRotation.x() << ","
            << pose.mRotation.y() << ","
            << pose.mRotation.z() << ","
            << pose.mRotation.w();
        return out;
    }

    Transform Pose::transFromPose(const Pose &otherPose) const {
        Eigen::Vector3f translationVector =
                mVector - otherPose.getVector();

        Eigen::Quaternionf rotationQuaternion =
                mRotation * otherPose.getRotation().conjugate();

        Transform returnValue(translationVector);

        return returnValue;
    }

    Pose Pose::origin() {
        Eigen::Vector3f originVector(0.0, 0.0, 0.0);
        Eigen::Quaternionf originQuaternion(1.0, 0.0, 0.0, 0.0);

        return Pose(originVector, originQuaternion);
    }

    void Pose::transform(Transform transform) {
        mVector = mVector + transform.translationPart();
        mRotation = mRotation * transform.rotationPart();
    }
}
