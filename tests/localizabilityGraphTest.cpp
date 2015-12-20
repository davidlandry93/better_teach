#include <localizability_graph.h>
#include "gtest/gtest.h"
#include "pointmatcherservice.h"

using namespace TeachRepeat;

class LocalizabilityGraphTest : public ::testing::Test {
protected:
    LocalizabilityGraph graph;

    virtual void SetUp() {
        graph = LocalizabilityGraph(3);

        graph.anchorPointLocalizesPoint(0, 1);
        graph.anchorPointLocalizesPoint(0, 2);
        graph.anchorPointLocalizesPoint(1, 2);
    }

    virtual void TearDown() {

    }
};

TEST_F(LocalizabilityGraphTest, optimalSetTest) {
    std::vector<int> optimalSet = graph.optimalSetOfAnchorPoints();

    ASSERT_EQ(0, optimalSet[0]);
    ASSERT_EQ(1, optimalSet.size());
}