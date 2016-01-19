
#include <iostream>
#include <boost/program_options.hpp>
#include "pointmatcherservice.h"
#include "teach_repeat_map.h"

using namespace TeachRepeat;
namespace po = boost::program_options;

static const int N_SAMLE_PER_PAIR = 100;

int main(int argc, char** argv) {
    po::options_description desc("Options");

    desc.add_options()
            ("map,m", po::value< std::string >(), "The map to use to test the variance of the ICP.")
            ("icpconfig,i", po::value< std::string >(), "The Yaml config file to be used with libpointmatcher.")
            ("help,h", "Produce a help message.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    std::cout << "Loading map from disk..." << std::endl;
    Map map(vm["map"].as< std::string >());
    std::cout << "Done." << std::endl;

    PointMatcherService pmService;
    pmService.loadConfigFile(vm["icpconfig"].as< std::string >());

    for(auto it = map.begin(); it < it + 10; it += 2) {
        for(int i = 0; i < N_SAMLE_PER_PAIR; i++) {

        }
    }

    return 0;
}