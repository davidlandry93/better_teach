
#include <fstream>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>

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
    ("config,c", po::value< std::string >(), "The config file to use")
    ("help,h", "Produce a help message");
   
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if(vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  std::string configFile;
  if(!vm.count("config"))
    {
      std::cout << "No config file specified" << std::endl;
      return 1;
    }
  else
    {
      configFile = vm["config"].as< std::string >();
    }
  YAML::Node config = YAML::LoadFile(configFile.c_str());

  std::cout << "Loading map in memory..." << std::endl;
  Map map(config["map"].as< std::string >());

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
  float convergenceMapFromX = reading.getPosition().getVector()(0) - 5.0;
  float convergenceMapToX = reading.getPosition().getVector()(0) + 5.0;
  float convergenceMapFromY = reading.getPosition().getVector()(1) - 3.0;
  float convergenceMapToY = reading.getPosition().getVector()(1) + 3.0;
  
  Eigen::MatrixXf convergenceData = builder.build(convergenceMapFromX,
                                                  convergenceMapToX,
                                                  convergenceMapFromY,
                                                  convergenceMapToY,
                                                  30,
                                                  30);

  Eigen::IOFormat CsvFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");

  std::ofstream outputFile(config["output"].as< std::string >().c_str());
  outputFile << convergenceData.format(CsvFormat);
  outputFile.close();

  return 0;
}
