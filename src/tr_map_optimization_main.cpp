
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "pointmatcher/PointMatcher.h"

#include "teach_repeat_map.h"
#include "tolerance_ellipse_calculator.h"
#include "teachrepeat_map_optimizer.h"
#include "ellipsoid.h"

using namespace TeachRepeat;

namespace po = boost::program_options;
namespace pt = boost::posix_time;

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
            ("semimajor,a", po::value< float >(), "The tolerance to forward error")
            ("semiminor,b", po::value< float >(), "The tolerance to lateral error")
            ("rotation,c", po::value< float >(), "To tolerance to rotation")
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

    std::cout << "=== Teach and Repeat map optimization ===" << std::endl;
    std::cout << "Began on " << pt::to_iso_extended_string(pt::second_clock::local_time()) << std::endl;
    std::cout << "Parameters" << std::endl;
    std::cout << "semimajor (a): " << vm["semimajor"].as<float>() << std::endl;
    std::cout << "semiminor (b): " << vm["semiminor"].as<float>() << std::endl;
    std::cout << "rotation (c): " << vm["rotation"].as<float>() << std::endl;
    std::cout << "epsilon: " << MAX_ERROR_TO_CONVERGE << std::endl;

    std::cout << "Initializing map..." << std::endl;
    Map map(vm["map"].as< std::string >(), pmService);

    std::ofstream ofs;
    ofs.open("correctedAnchorPointsPositions.apd");
    map.outputAnchorPointsMetadata(ofs);
    ofs.close();

    Ellipsoid<float> toleranceEllipsoid = Ellipsoid<float>(vm["semimajor"].as<float>(), vm["semiminor"].as<float>(), vm["rotation"].as<float>());

    ToleranceEllipseCalculator<float> toleranceEllipseCalculator(toleranceEllipsoid, MAX_ERROR_TO_CONVERGE, pmService);
    MapOptimizer optimizer(toleranceEllipseCalculator);

    std::cout << "Map initialization ended on " << pt::to_iso_extended_string(pt::second_clock::local_time())<< std::endl;
    std::cout << "Optimizing map..." << std::endl;
    optimizer.optimize(map);

    std::cout << "Writing results..." << std::endl;

    ofs.open("optimizedMap.csv");
    map.outputAnchorPointsMetadata(ofs);
    ofs.close();

    ofs.open("localizabilityGraph.lgf");
    optimizer.saveGraph(ofs);
    ofs.close();

    std::cout << "Optimization ended on " << pt::to_iso_extended_string(pt::second_clock::local_time())<< std::endl;

    return 0;
}