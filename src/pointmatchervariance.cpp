
#include <iostream>
#include <boost/program_options.hpp>
#include "pointmatcherservice.h"
#include "teach_repeat_map.h"

using namespace TeachRepeat;
namespace po = boost::program_options;

static const int N_SAMLE_PER_PAIR = 100;

float variance(const std::vector<float>& samples) {
    float sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    float mean = sum / samples.size();

    float acc = 0;
    for(auto it = samples.begin(); it != samples.end(); it++) {
        acc += (*it - mean) * (*it - mean);
    }

    return acc;
}

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

    PointMatcherService<float> pmService;
    pmService.loadConfigFile(vm["icpconfig"].as< std::string >());

    std::vector<float> variances;
    for(auto it = map.begin(); it < it + 10; it += 2) {
        std::vector<float> values;
        for(int i = 0; i < N_SAMLE_PER_PAIR; i++) {
            Transform tFromReadingToReference = it->getPosition().transFromPose(map.begin()->getPosition());
            Transform icpResult = pmService.icp(*it, *map.begin(), tFromReadingToReference);

            float new_value = (icpResult.matrix().matrix() - tFromReadingToReference.matrix().matrix()).norm();
            std::cout << new_value << std::endl;
            values.push_back(new_value);
        }
        variances.push_back(variance(values));
        std::cout << "Variance: " << variance(values) << std::endl;
    }

    return 0;
}