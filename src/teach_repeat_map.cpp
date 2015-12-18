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

            Transform originToPreviousPose = correctedPoses[i-1].transFromPose(Pose::origin());
            Transform originToCurrentPose = it->getPosition().transFromPose(Pose::origin());

            Transform icpResult = pointMatcherService.icp(*it, *(it - 1));
            
            (it - 1)->saveToDisk("", std::to_string(i) + "ref.vtk");

            Transform tFromOriginToCurrent = icpResult * originToPreviousPose;

            it->transform(icpResult);
            it->saveToDisk("", std::to_string(i) + "res.vtk");
            it->transform(icpResult.inverse());

            Pose poseOfCurrent = Pose::origin();
            poseOfCurrent.transform(tFromOriginToCurrent);
            correctedPoses.push_back(poseOfCurrent);
        }

        for(auto anchor : anchorPoints)
            std::cout << anchor.getPosition().getVector() << std::endl << std::endl;

        for(auto pose : correctedPoses)
            std::cout << pose.getVector() << std::endl << std::endl;

        for(int i = 0; i < anchorPoints.size(); ++i) {
            anchorPoints[i].setPosition(correctedPoses[i]);
        }
    }

    Transform Map::tFromCloudToCloud(int firstCloudIndex, int secondCloudIndex) {
        return anchorPoints[secondCloudIndex].getPosition().transFromPose(anchorPoints[firstCloudIndex].getPosition());
    }

    void Map::writeMapListToStream(std::ostream& stream) const {
        for(auto anchorPoint : anchorPoints) {
            stream << anchorPoint <<  std::endl;
        }
    }

} //namespace TeachRepeat
