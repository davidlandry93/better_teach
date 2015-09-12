
#include <iostream>
#include <string>

#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "anchor_point.h"
#include "teach_repeat_map.h"

using namespace TeachRepeat;

void convergenceBassin(AnchorPoint& anchorPoint,
		       AnchorPoint& reading,
		       int gridLength,
		       float delta,
		       Eigen::MatrixXf& output)
{
  Eigen::Transform<float,3,Eigen::Affine> bestEstimate =
    reading.getPosition().transFromPose(anchorPoint.getPosition());

  std::cout << bestEstimate.matrix() << std::endl;
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
