
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include "pointmatcher/PointMatcher.h"

#include "teach_repeat_map.h"
#include "tolerance_ellipse_calculator.h"
#include "teachrepeat_map_optimizer.h"

using namespace TeachRepeat;

int main(int argc, char** argv) {
    const float MAX_ERROR_TO_CONVERGE = 0.05;

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

    PointMatcherService<float> pmService;
    pmService.loadConfigFile(config["icp_config"].as< std::string >());

    Map map(config["map"].as< std::string >(), pmService);

    std::ofstream ofs;
    ofs.open("correctedAnchorPointsPositions.apd");
    map.outputAnchorPointsMetadata(ofs);
    ofs.close();

    std::vector<LocalisedPointCloud>::iterator cursor = map.begin();

    for(int i = 0; i < 1; i++)
    {
        cursor++;
    }

    LocalisedPointCloud anchorPoint = *cursor;

    for(int i=0; i < 1; i++)
    {
        cursor++;
    }

    LocalisedPointCloud reading = *cursor;

    Ellipse<float> toleranceEllipse = Ellipse<float>(0.3, 0.3);


    ToleranceEllipseCalculator<float> toleranceEllipseCalculator(toleranceEllipse, MAX_ERROR_TO_CONVERGE, pmService);
    MapOptimizer optimizer(toleranceEllipseCalculator);

    Map optimizedMap = optimizer.optimize(map);

    return 0;
}