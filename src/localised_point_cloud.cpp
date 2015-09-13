
#include <boost/filesystem.hpp>

#include "localised_point_cloud.h"

namespace TeachRepeat
{

  LocalisedPointCloud::LocalisedPointCloud() : mPointCloud()
  { }

  LocalisedPointCloud::LocalisedPointCloud(std::string& anchorPointName, Pose position) :
    mAnchorPointName(anchorPointName), mPointCloud(), mPosition(position)
  { }

  LocalisedPointCloud::LocalisedPointCloud(std::string& anchorPointName, Pose position, DP cloud) :
    mAnchorPointName(anchorPointName), mPointCloud(cloud), mPosition(position)
  { }


  // Builds an anchor point from a string, as in the format outputted by the << operator.
  LocalisedPointCloud::LocalisedPointCloud(std::string& anchorPointEntry)
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

  LocalisedPointCloud::~LocalisedPointCloud()
  {

  }

  PointMatcher<float>::DataPoints LocalisedPointCloud::getCloud() const
  {
    return mPointCloud;
  }


  void LocalisedPointCloud::loadFromDisk(std::string directory)
  {
    std::string filename = directory == "" ?
      mAnchorPointName :
      (boost::filesystem::path(directory) / mAnchorPointName).string();
      
      mPointCloud = PointMatcherIO<float>::loadVTK(filename);
  }

  void LocalisedPointCloud::saveToDisk(std::string directory) const
  {

    std::string filename = directory == "" ?
      mAnchorPointName :
      (boost::filesystem::path(directory) / mAnchorPointName).string();
      
      mPointCloud.save(filename);
  }

  std::ostream& operator<<(std::ostream& out, LocalisedPointCloud& ap)
  {
    out << ap.mAnchorPointName << "," << ap.mPosition;
    return out;
  }


  std::string LocalisedPointCloud::name() const
  {
    return mAnchorPointName;
  }

  Pose LocalisedPointCloud::getPosition() const
  {
    return mPosition;
  }

} // namespace TeachRepeat
