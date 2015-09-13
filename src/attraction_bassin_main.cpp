
#include <iostream>
#include <string>

#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "pointmatcher/PointMatcher.h"

#include "localised_point_cloud.h"
#include "transform.h"
#include "teach_repeat_map.h"

using namespace TeachRepeat;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

Transform do_icp(DP& reading, DP& anchorPoint, Transform T)
{
  PM::ICP icp;
  icp.setDefault();

  PM::Transformation* rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  if (!rigidTrans->checkParameters(T.pmTransform())) {
    std::cout << "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis"
              << std::endl;
  }

  DP transformedAnchorPoint = rigidTrans->compute(anchorPoint, T.pmTransform());

  PM::TransformationParameters icpResult = icp(reading, transformedAnchorPoint);

  return Transform(icpResult);
}

Transform transformFromApToAp(const LocalisedPointCloud& anchorPoint,
                              const LocalisedPointCloud& reading)
{
  Transform roughEstimate =
    reading.getPosition().transFromPose(anchorPoint.getPosition());

  DP readingPointCloud = reading.getCloud();
  DP anchorPointCloud = anchorPoint.getCloud();

  Transform icpResult =
    do_icp(readingPointCloud, anchorPointCloud, roughEstimate);

  std::cout << icpResult << std::endl;

  // This is the best estimate we have of the actual movement the robot
  // would have to do to go from the anchor point to the reading.
  Transform evenBetterEstimate = icpResult * roughEstimate;
  return evenBetterEstimate;
}

void convergenceBassin(const LocalisedPointCloud& anchorPoint,
		       const LocalisedPointCloud& reading,
		       int gridLength,
		       float delta,
		       Eigen::MatrixXf& output)
{
  Transform referenceTransform = transformFromApToAp(anchorPoint, reading);
}


int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::options_description desc("Options");

  desc.add_options()
    ("map,m", po::value< std::string >(),
     "The location of the Teach Repeat map to handle")
    ("help,h", "Produce a help message");
   
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if(vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  if(!vm.count("map")) {
    std::cout << "No teach and repeat map was given" << std::endl;
    return 1;
  }

  std::cout << "Using map " << vm["map"].as< std::string >() << std::endl;

  std::cout << "Loading map in memory..." << std::endl;
  Map map(vm["map"].as< std::string >());

  std::vector<LocalisedPointCloud>::iterator cursor = map.begin();

  for(int i = 0; i < 5; i++)
    {
      cursor++;
    }

  LocalisedPointCloud anchorPoint = *cursor;

  for(int i=0; i < 5; i++)
    {
      cursor++;
    }

  LocalisedPointCloud reading = *cursor;

  Eigen::MatrixXf convergenceData;
  convergenceBassin(anchorPoint, reading, 5.0, 0.2, convergenceData);
  
  return 0;
}
