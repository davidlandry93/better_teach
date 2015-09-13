
#include <fstream>
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
#include "attraction_bassin_builder.h"

using namespace TeachRepeat;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;


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


  AttractionBassinBuilder builder(anchorPoint, reading);
  float convergenceMapFromX = reading.getPosition().getVector()(0) - 2.0;
  float convergenceMapToX = reading.getPosition().getVector()(0) + 2.0;
  float convergenceMapFromY = reading.getPosition().getVector()(1) - 2.0;
  float convergenceMapToY = reading.getPosition().getVector()(1) + 2.0;
  
  Eigen::MatrixXf convergenceData = builder.build(convergenceMapFromX,
                                                  convergenceMapToX,
                                                  convergenceMapFromY,
                                                  convergenceMapToY,
                                                  10,
                                                  10);
  std::ofstream outputFile("output.csv");
  outputFile << convergenceData;
  outputFile.close();

  return 0;
}
