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

int main(int argc, char **argv) {
    namespace po = boost::program_options;
    po::options_description desc("Options");

    desc.add_options()
            ("map,m", po::value<std::string>(), "The map to use.")
            ("output,o", po::value<std::string>(), "The name of the output file.")
            ("icp,i", po::value<std::string>(), "ICP config file.")
            ("reading,r", po::value<int>(), "The index of the reading.")
            ("anchor,a", po::value<int>(), "The index of the anchor point.")
            ("help,h", "Produce a help message");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    std::string mapPath;
    if (!vm.count("map")) {
        std::cout << "No map specified" << std::endl;
        return 1;
    }
    else {
        mapPath = vm["map"].as<std::string>();
    }

    std::string output;
    if(!vm.count("output")) {
        std::cout << "No output specified" << std::endl;

    } else {
        output = vm["output"].as<std::string>();
    }


    std::cout << "Loading map in memory..." << std::endl;
    Map map(mapPath);

    std::vector<LocalisedPointCloud>::iterator cursor = map.begin();

    cursor = map.begin() + vm["anchor"].as<int>();
    LocalisedPointCloud anchorPoint = *cursor;


    cursor = map.begin() + vm["reading"].as<int>();
    LocalisedPointCloud reading = *cursor;

    AttractionBassinBuilder builder(anchorPoint, reading);
    if (vm.count("icp")) {
        std::string icpConfigPath = vm["icp"].as<std::string>();
        builder.setIcpConfigFile(icpConfigPath);
    }

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

    Eigen::IOFormat CsvFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "",
                              "");

    std::ofstream outputFile(output);
    outputFile << convergenceData.format(CsvFormat);
    outputFile.close();

    return 0;
}
