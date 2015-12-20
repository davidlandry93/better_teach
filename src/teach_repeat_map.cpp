#include <istream>
#include <boost/filesystem.hpp>
#include <bits/basic_string.h>

#include "teach_repeat_map.h"

namespace TeachRepeat {

    Map::Map() { }

    Map::Map(std::string directory, PointMatcherService<float> & service)
    : Map(directory) {
        correctPositions(service);
    }

    Map::Map(std::string directory) {
        boost::filesystem::path base(directory);

        locationOnDisk = directory;

        std::ifstream commandFile((base / "speeds.sl").string().c_str());
        loadCommands(commandFile);

        std::ifstream poseFile((base / "positions.pl").string().c_str());
        loadPositions(poseFile);

        std::ifstream anchorPointsFile((base / "anchorPoints.apd").string().c_str());
        loadAnchorPoints(anchorPointsFile);
    }

    std::vector<LocalisedPointCloud>::iterator Map::begin() {
        return anchorPoints.begin();
    }

    std::vector<LocalisedPointCloud>::iterator Map::end() {
        return anchorPoints.end();
    }

    void Map::loadCommands(std::istream &input) {
        commands.clear();

        std::string lineBuffer;
        while (std::getline(input, lineBuffer)) {
            commands.push_back(Command(lineBuffer));
        }
    }

    void Map::loadPositions(std::istream &input) {
        positions.clear();

        std::string lineBuffer;
        while (std::getline(input, lineBuffer)) {
            positions.push_back(Pose(lineBuffer));
        }
    }

    void Map::loadAnchorPoints(std::istream &input) {
        anchorPoints.clear();

        std::string lineBuffer;
        while (std::getline(input, lineBuffer)) {
            anchorPoints.push_back(LocalisedPointCloud(lineBuffer));
            anchorPoints.back().loadFromDisk(locationOnDisk);
        }
    }

    void Map::correctPositions(PointMatcherService<float> & pointMatcherService) {
        std::vector<Pose> correctedPoses;
        correctedPoses.push_back(Pose::origin());

        for(int i = 1; i < anchorPoints.size(); ++i) {
            auto it = anchorPoints.begin() + i;

            Pose previousOdometryEstimate = (it - 1)->getPosition();
            Pose currentOdometryEstimate = it->getPosition();
            Transform initialGuess = currentOdometryEstimate.transFromPose(previousOdometryEstimate);

            Transform icpResult = pointMatcherService.icp(*it, *(it - 1), initialGuess);
            
            Transform originToPreviousPose = correctedPoses[i-1].transFromPose(Pose::origin());
            Transform tFromOriginToCurrent = icpResult * originToPreviousPose;

            Pose poseOfCurrent = Pose::origin();
            poseOfCurrent.transform(tFromOriginToCurrent);
            correctedPoses.push_back(poseOfCurrent);
        }

        for(int i = 0; i < anchorPoints.size(); ++i) {
            anchorPoints[i].setPosition(correctedPoses[i]);
        }

    }

    void Map::outputAnchorPointsMetadata(std::ostream &ostream) {
        for(auto anchorPoint : anchorPoints) {
            ostream << anchorPoint << std::endl;
        }
    }


} //namespace TeachRepeat
