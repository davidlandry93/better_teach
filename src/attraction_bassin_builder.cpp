#include "pointmatcher/PointMatcher.h"

#include "attraction_bassin_builder.h"
#include "transform.h"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

namespace TeachRepeat {
    AttractionBassinBuilder::AttractionBassinBuilder(const LocalisedPointCloud &reference,
                                                     const LocalisedPointCloud &reading) :
            mReference(reference), mReading(reading), mIcpEngine(PM::ICP()) {
        // Init the icp engine.
        mIcpEngine.setDefault();

        // Find the most accurate transformation from the reference to the reading.
        mRoughEstimate =
                reading.getPosition().transFromPose(reference.getPosition());

        DP readingPointCloud = reading.getCloud();
        DP referencePointCloud = reference.getCloud();

        Transform icpResult =
                do_icp(readingPointCloud, referencePointCloud, mRoughEstimate);

        // This is the best estimate we have of the actual movement the robot
        // would have to do to go from the anchor point to the reading.
        Transform evenBetterEstimate = icpResult * mRoughEstimate;
        mTransFromReferenceToReading = evenBetterEstimate;
    }

    void AttractionBassinBuilder::setReading(const LocalisedPointCloud &newReading) {
        mReading = newReading;
    }

    void AttractionBassinBuilder::setIcpConfigFile(const std::string icpConfigFilename) {
        std::ifstream ifs(icpConfigFilename.c_str());

        if (ifs) {
            mIcpEngine.loadFromYaml(ifs);
        } else {
            std::cout << "Could not load the specified icp config file" << std::endl;
        }
    }

    Eigen::MatrixXf AttractionBassinBuilder::build(float fromX, float toX, float fromY, float toY,
                                                   int resX, int resY) {
        // To make the map more intuitive to read, the columns represent x coordinates,
        // the rows represent y coordinates.
        Eigen::MatrixXf attractionMap(resX, resY);

        float deltaX = (toX - fromX) / resX;
        float deltaY = (toY - fromY) / resY;

        for (int i = 0; i < resX; i++) {
            for (int j = 0; j < resY; j++) {
                Eigen::Vector3f preciseReadingLocation =
                        mReference.getPosition().getVector() +
                        mTransFromReferenceToReading.translationPart();

                Eigen::Vector3f gridPointPosition(fromX + deltaX * i, fromY + deltaY * j, 0.0);
                Eigen::Vector3f inducedError = gridPointPosition - preciseReadingLocation;

                Transform preTransform = mRoughEstimate * Transform(inducedError);

                Transform icpResult = do_icp(mReading.getCloud(), mReference.getCloud(),
                                             preTransform);

                Eigen::Vector3f icpConvergenceLocation =
                        gridPointPosition - icpResult.translationPart();


                int nOfPoints = resX * resY;
                int nOfPointsDone = i * resX + j;
                std::cout << "[" << nOfPointsDone << "/" << nOfPoints << "] " <<
                    (icpConvergenceLocation - preciseReadingLocation).squaredNorm() <<
                    std::endl;

                attractionMap(i, j) = (icpConvergenceLocation -
                                       preciseReadingLocation).squaredNorm();
            }
        }

        return attractionMap;
    }

    Transform AttractionBassinBuilder::do_icp(const DP &reading, const DP &anchorPoint,
                                              Transform preTransform) {
        PM::Transformation *rigidTrans;
        rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

        PM::TransformationParameters pmTransform = preTransform.pmTransform();
        if (!rigidTrans->checkParameters(pmTransform)) {
            std::cout <<
            "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis"
            << std::endl;
            rigidTrans->correctParameters(pmTransform);
        }

        DP transformedReference = rigidTrans->compute(anchorPoint, pmTransform);

        PM::TransformationParameters icpResult = mIcpEngine(reading, transformedReference);

        return Transform(icpResult);
    }
}

