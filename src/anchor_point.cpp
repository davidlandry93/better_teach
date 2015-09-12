
#include <boost/filesystem.hpp>

#include "anchor_point.h"

namespace TeachRepeat
{

  AnchorPoint::AnchorPoint() : mPointCloud()
  { }

  AnchorPoint::AnchorPoint(std::string& anchorPointName, Pose position) :
    mAnchorPointName(anchorPointName), mPointCloud(), mPosition(position)
  { }

  AnchorPoint::AnchorPoint(std::string& anchorPointName, Pose position, DP cloud) :
    mAnchorPointName(anchorPointName), mPointCloud(cloud), mPosition(position)
  { }


  // Builds an anchor point from a string, as in the format outputted by the << operator.
  AnchorPoint::AnchorPoint(std::string& anchorPointEntry)
  {
    std::stringstream ss(anchorPointEntry);
    std::string buffer;
    
    std::getline(ss,buffer,',');
    std::string filename(buffer);

    std::getline(ss,buffer);

    Pose pose(buffer);

    mAnchorPointName = filename;
    mPosition = pose;
  }

  AnchorPoint::~AnchorPoint()
  {

  }

  PointMatcher<float>::DataPoints AnchorPoint::getCloud() const
  {
    return mPointCloud;
  }


  void AnchorPoint::loadFromDisk(std::string directory)
  {
    std::string filename = directory == "" ?
      mAnchorPointName :
      (boost::filesystem::path(directory) / mAnchorPointName).string();
      
      mPointCloud = PointMatcherIO<float>::loadVTK(filename);
  }

  void AnchorPoint::saveToDisk(std::string directory) const
  {

    std::string filename = directory == "" ?
      mAnchorPointName :
      (boost::filesystem::path(directory) / mAnchorPointName).string();
      
      mPointCloud.save(filename);
  }

  std::ostream& operator<<(std::ostream& out, AnchorPoint& ap)
  {
    out << ap.mAnchorPointName << "," << ap.mPosition;
    return out;
  }


  std::string AnchorPoint::name() const
  {
    return mAnchorPointName;
  }

  Pose AnchorPoint::getPosition() const
  {
    return mPosition;
  }

} // namespace TeachRepeat
