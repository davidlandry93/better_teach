
#ifndef ANCHOR_POINT_H
#define ANCHOR_POINT_H

#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Geometry>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/IO.h>

#include "pose.h"

namespace TeachRepeat
{
class LocalisedPointCloud {

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Parameters Parameters;
    typedef PM::Transformation Transformation;
  
private:
    const static std::string POINT_CLOUD_FRAME;

    std::string mAnchorPointName;
    DP mPointCloud;
    Pose mPosition;

public:
    LocalisedPointCloud(std::string& anchorPointName, Pose position);
    LocalisedPointCloud(std::string& anchorPointName, Pose position, DP cloud);
    LocalisedPointCloud(std::string& anchorPointEntry);
    LocalisedPointCloud();
    ~LocalisedPointCloud();

    PointMatcher<float>::DataPoints getCloud() const;
    void loadFromDisk(std::string directory="");
    void saveToDisk(std::string directory="") const;
    void saveToDisk(std::string directory, std::string filename) const;

    friend std::ostream& operator<<(std::ostream& out, LocalisedPointCloud& ap);

    std::string name() const;
    Pose getPosition() const;

    void transform(const Transform t);
};

} // namespace TeachRepeat
#endif
