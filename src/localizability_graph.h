#ifndef BETTERTEACH_LOCALIZABILITYGRAPH_H
#define BETTERTEACH_LOCALIZABILITYGRAPH_H

#include <boost/graph/adjacency_list.hpp>

namespace TeachRepeat {
    class LocalizabilityGraph {
        typedef boost::adjacency_list<boost::setS, boost::vecS, boost::directedS> Graph;

    public:
        LocalizabilityGraph(int nOfPoints);
        void anchorPointLocalisesPoint(int anchorPointIndex, int pointIndex);
        std::vector<int> optimalSetOfAnchorPoints() const;

    private:
        Graph graph;
    };
}




#endif //BETTERTEACH_LOCALIZABILITYGRAPH_H
