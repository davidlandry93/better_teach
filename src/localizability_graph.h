#ifndef BETTERTEACH_LOCALIZABILITYGRAPH_H
#define BETTERTEACH_LOCALIZABILITYGRAPH_H

#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>

namespace TeachRepeat {
    class LocalizabilityGraph {
        typedef lemon::ListDigraph Graph;
        typedef Graph::Node Node;
        typedef Graph::Arc Arc;
        typedef Graph::ArcMap<int> LengthMap;
        typedef Graph::NodeMap<int> IndexMap;

    public:
        LocalizabilityGraph();
        LocalizabilityGraph(int nOfPoints);
        void anchorPointLocalizesPoint(int anchorPointIndex, int pointIndex);
        std::list<int> optimalSetOfAnchorPoints() const;

    private:
        Graph graph;
        IndexMap indexMap;
        std::vector< Node > nodes;
    };
}

#endif //BETTERTEACH_LOCALIZABILITYGRAPH_H
