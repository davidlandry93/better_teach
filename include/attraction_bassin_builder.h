
#ifndef ATTRACTION_BASSIN_BUILDER_H
#define ATTRACTION_BASSIN_BUILDER_H

#include <fstream>
#include <exception>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "localised_point_cloud.h"

namespace TeachRepeat
{

  class AttractionBassinBuilder
  {
  public:
    AttractionBassinBuilder(const LocalisedPointCloud& reference, const LocalisedPointCloud& reading);
    void setReading(const LocalisedPointCloud& newReading);
    void setIcpConfigFile(const std::string icpConfigFilename);
    Eigen::MatrixXf build(float fromX, float toX, float fromY, float toY, int resX, int resY);

  private:
    LocalisedPointCloud mReference;
    LocalisedPointCloud mReading;
    Transform mTransFromReferenceToReading;
    Transform mRoughEstimate;
    PointMatcher<float>::ICP mIcpEngine;

    Transform do_icp(const PointMatcher<float>::DataPoints& reading, const PointMatcher<float>::DataPoints& anchorPoint, Transform preTransform);
  };

} // Namespace TeachRepeat.

#endif
