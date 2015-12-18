#include <boost/filesystem.hpp>

#include "localised_point_cloud.h"

namespace TeachRepeat {

    LocalisedPointCloud::LocalisedPointCloud() : mPointCloud() { }

    LocalisedPointCloud::LocalisedPointCloud(std::string anchorPointName, Pose position) :
            mAnchorPointName(anchorPointName), mPointCloud(), mPosition(position) { }

    LocalisedPointCloud::LocalisedPointCloud(std::string anchorPointName, Pose position, DP cloud)
            :
            mAnchorPointName(anchorPointName), mPointCloud(cloud), mPosition(position) { }


    // Builds an anchor point from a string, as in the format outputted by the << operator.
    LocalisedPointCloud::LocalisedPointCloud(std::string anchorPointEntry) {
        std::stringstream ss(anchorPointEntry);
        std::string buffer;

        std::getline(ss, buffer, ',');
        std::string filename(buffer);

        std::getline(ss, buffer);

        Pose pose(buffer);

        mAnchorPointName = filename;
        mPosition = pose;
    }

    LocalisedPointCloud::~LocalisedPointCloud() {

    }

    PointMatcher<float>::DataPoints LocalisedPointCloud::getCloud() const {
        return mPointCloud;
    }


    void LocalisedPointCloud::loadFromDisk(std::string directory) {
        std::string filename = directory == "" ?
                               mAnchorPointName :
                               (boost::filesystem::path(directory) / mAnchorPointName).string();

        mPointCloud = PointMatcherIO<float>::loadVTK(filename);
    }

    void LocalisedPointCloud::saveToDisk(std::string directory) const {
        saveToDisk(directory, mAnchorPointName);
    }

    void LocalisedPointCloud::saveToDisk(std::string directory, std::string filename) const {
        boost::filesystem::path basepath(directory), filepath(filename), fullpath;

        fullpath = directory.empty() ? filepath : basepath / filepath;

        mPointCloud.save(fullpath.generic_string());
    }

    std::ostream &operator<<(std::ostream &out, LocalisedPointCloud &ap) {
        out << ap.mAnchorPointName << "," << ap.mPosition;
        return out;
    }


    std::string LocalisedPointCloud::name() const {
        return mAnchorPointName;
    }

    Pose LocalisedPointCloud::getPosition() const {
        return mPosition;
    }

    void LocalisedPointCloud::setPosition(Pose const& pose) {
        mPosition = pose;
    }

    void LocalisedPointCloud::transform(const Transform t) {
        Transformation *rigidTrans;
        rigidTrans = PointMatcher<float>::get().REG(Transformation).create("RigidTransformation");

        if (!rigidTrans->checkParameters(t.pmTransform())) {
            std::cout <<
            "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis"
            << std::endl;
            rigidTrans->correctParameters(t.pmTransform());
        }

        mPointCloud = rigidTrans->compute(mPointCloud, t.pmTransform());
    }
} // namespace TeachRepeat
