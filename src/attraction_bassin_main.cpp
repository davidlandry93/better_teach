
#include <iostream>
#include <string>

#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "pointmatcher/PointMatcher.h"

#include "anchor_point.h"
#include "transform.h"
#include "teach_repeat_map.h"

using namespace TeachRepeat;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

std::string quatToString(Eigen::Quaternionf quat)
{
  std::stringstream ss;
  ss << "[x: " << quat.x() << ", y: " << quat.y() << ", z: " << quat.z() << ", w: " << quat.w() << "]" << std::endl;
  return ss.str();
}

void printHumanReadableTransform(const Eigen::Affine3f transform)
{
  Eigen::Affine3f::ConstTranslationPart translationPart = transform.translation();
  Eigen::Affine3f::LinearMatrixType rotationPart = transform.rotation();

  Eigen::Quaternionf quat(rotationPart);
  quat.normalize();

  std::cout << "Translation." << std::endl << translationPart << std::endl;
  std::cout << "Rotation." << std::endl << quatToString(quat) << std::endl;
}

PM::TransformationParameters eigenMatrixToDim(const PM::TransformationParameters& matrix, int dimp1)
{
  typedef PM::TransformationParameters M;
  assert(matrix.rows() == matrix.cols());
  assert((matrix.rows() == 3) || (matrix.rows() == 4));
  assert((dimp1 == 3) || (dimp1 == 4));
		
  if (matrix.rows() == dimp1)
    return matrix;
		
  M out(M::Identity(dimp1,dimp1));
  out.topLeftCorner(2,2) = matrix.topLeftCorner(2,2);
  out.topRightCorner(2,1) = matrix.topRightCorner(2,1);
  return out;
}

Eigen::Affine3f eigenTransormOfPmTransform(const PM::TransformationParameters pmTransform)
{
  Eigen::Affine3f eigenMat(Eigen::Matrix4f(pmTransform.template cast<float>()));
  return eigenMat;
}

void pmTransformOfEigenTransform(const Eigen::Transform<float,3,Eigen::Affine>& eigenTransform, PM::TransformationParameters& pmTransform)
{
  pmTransform = PM::TransformationParameters::Identity(4,4);

  for(int i=0; i < 4; i++)
    {
      for(int j=0; j < 4; j++)
	{
	  pmTransform(i,j) = eigenTransform(i,j);
	}
    }
}

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

Transform transformFromApToAp(const AnchorPoint& anchorPoint,
                              const AnchorPoint& reading)
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

void convergenceBassin(const AnchorPoint& anchorPoint,
		       const AnchorPoint& reading,
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

  std::vector<AnchorPoint>::iterator cursor = map.begin();

  for(int i = 0; i < 5; i++)
    {
      cursor++;
    }

  AnchorPoint anchorPoint = *cursor;

  for(int i=0; i < 5; i++)
    {
      cursor++;
    }

  AnchorPoint reading = *cursor;

  Eigen::MatrixXf convergenceData;
  convergenceBassin(anchorPoint, reading, 5.0, 0.2, convergenceData);
  
  return 0;
}
