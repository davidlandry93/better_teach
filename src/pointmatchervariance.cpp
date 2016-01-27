
#include <iostream>
#include <boost/program_options.hpp>
#include "pointmatcherservice.h"
#include "teach_repeat_map.h"

using namespace TeachRepeat;
namespace po = boost::program_options;

static const int N_SAMLE_PER_PAIR = 50;

float variance(const std::vector<float>& samples) {
    float sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    float mean = sum / samples.size();

    float acc = 0;
    for(auto it = samples.begin(); it != samples.end(); it++) {
        acc += (*it - mean) * (*it - mean);
    }

    return acc / samples.size();
}

int main(int argc, char** argv) {
    po::options_description desc("Options");

    desc.add_options()
            ("map,m", po::value< std::string >(), "The map to use to test the variance of the ICP.")
            ("icpconfig,i", po::value< std::string >(), "The Yaml config file to be used with libpointmatcher.")
            ("first,f", po::value< int >(), "The index of the point cloud to use as reference.")
            ("number,n", po::value< int >(), "Number of clouds to test.")
            ("help,h", "Produce a help message.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    Map map(vm["map"].as< std::string >());

    PointMatcherService<float> pmService;
    pmService.loadConfigFile(vm["icpconfig"].as< std::string >());

    int firstIndex = vm["first"].as< int >();

    std::cout << "distance,stddeviation,samplesize" << std::endl;

    std::vector<float> variances;

    auto firstCloud = map.begin() + firstIndex;

    for(auto it = firstCloud; it < firstCloud + vm["number"].as< int >(); it += 1) {
        std::vector<float> values;
        Transform tFromReadingToReference = it->getPosition().transFromPose(firstCloud->getPosition());

        for(int i = 0; i < N_SAMLE_PER_PAIR; i++) {
            Transform icpResult = pmService.icp(*it, *map.begin(), tFromReadingToReference);

            float new_value = (icpResult.matrix().matrix() - tFromReadingToReference.matrix().matrix()).norm();
            values.push_back(new_value);
        }

        std::cout << tFromReadingToReference.translationPart().norm() << ",";
        std::cout << std::sqrt(variance(values)) << ",";
        std::cout << N_SAMLE_PER_PAIR << std::endl;
    }

    return 0;
}