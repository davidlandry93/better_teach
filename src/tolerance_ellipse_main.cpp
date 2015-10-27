
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include "teach_repeat_map.h"
#include "tolerance_ellipse_calculator.h"

using namespace TeachRepeat;

int main(int argc, char** argv) {

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

    return 0;
}