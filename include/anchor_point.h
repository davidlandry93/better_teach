
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
class AnchorPoint {

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Parameters Parameters;
  
private:
    const static std::string POINT_CLOUD_FRAME;

    std::string mAnchorPointName;
    DP mPointCloud;
    Pose mPosition;

public:
    AnchorPoint(std::string& anchorPointName, Pose position);
    AnchorPoint(std::string& anchorPointName, Pose position, DP cloud);
    AnchorPoint(std::string& anchorPointEntry);
    AnchorPoint();
    ~AnchorPoint();

    DP getCloud() const;
    void loadFromDisk(std::string directory="");
    void saveToDisk(std::string directory="");

    friend std::ostream& operator<<(std::ostream& out, AnchorPoint& ap);

    std::string name() const;
    Pose getPosition() const;
};

} // namespace TeachRepeat
#endif
