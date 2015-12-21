#include <localizability_graph.h>
#include "gtest/gtest.h"
#include "pointmatcherservice.h"

using namespace TeachRepeat;

class LocalizabilityGraphTest : public ::testing::Test {

protected:
    LocalizabilityGraph* graph;

    virtual void SetUp() {
        graph = new LocalizabilityGraph(5);

        graph->anchorPointLocalizesPoint(0, 1);
        graph->anchorPointLocalizesPoint(0, 2);
        graph->anchorPointLocalizesPoint(2, 3);
        graph->anchorPointLocalizesPoint(2, 4);
    }

    virtual void TearDown() {
        delete graph;
    }
};

TEST_F(LocalizabilityGraphTest, optimalSetTest) {
    std::list<int> optimalSet = graph->optimalSetOfAnchorPoints();

    ASSERT_EQ(2, optimalSet.size());

    ASSERT_EQ(0, optimalSet.front());
    optimalSet.pop_front();

    ASSERT_EQ(2, optimalSet.front());
    optimalSet.pop_front();
}