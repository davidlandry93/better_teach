
#include <boost/program_options.hpp>

#include "pointmatcher/PointMatcher.h"

#include "teach_repeat_map.h"
#include "tolerance_ellipse_calculator.h"
#include "teachrepeat_map_optimizer.h"

using namespace TeachRepeat;

namespace po = boost::program_options;

bool validate_user_input(po::variables_map vm) {
    if(!vm.count("map"))
    {
        std::cout << "No map specified" << std::endl;
        return false;
    }

    if(!vm.count("semimajor") || !vm.count("semiminor")) {
        std::cout << "Axes lengts must be defined" << std::endl;
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    const float MAX_ERROR_TO_CONVERGE = 0.05;



    po::options_description desc("Options");

    desc.add_options()
            ("semimajor,a", po::value< float >(), "The length of the semimajor axis")
            ("semiminor,b", po::value< float >(), "The lenght of the semiminor axis")
            ("icp,i", po::value< std::string >(), "Path to the ICP config file")
            ("map,m", po::value< std::string >(), "Path to the teach and repeat map")
            ("help,h", "Produce a help message");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    if(!validate_user_input(vm)) {
        return 1;
    }

    PointMatcherService<float> pmService;
    if(vm.count("icp")) {
        pmService.loadConfigFile(vm["icp"].as< std::string >());
    }

    std::cout << "Initializing map..." << std::endl;
    Map map(vm["map"].as< std::string >(), pmService);

    std::ofstream ofs;
    ofs.open("correctedAnchorPointsPositions.apd");
    map.outputAnchorPointsMetadata(ofs);
    ofs.close();

    Ellipse<float> toleranceEllipse = Ellipse<float>(0.4, 0.4);

    ToleranceEllipseCalculator<float> toleranceEllipseCalculator(toleranceEllipse, MAX_ERROR_TO_CONVERGE, pmService);
    MapOptimizer optimizer(toleranceEllipseCalculator);

    optimizer.optimize(map);

    ofs.open("optimizedMap.csv");
    map.outputAnchorPointsMetadata(ofs);
    ofs.close();

    return 0;
}