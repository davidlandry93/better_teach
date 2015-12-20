#include <iostream>
#include "localizability_graph.h"

TeachRepeat::LocalizabilityGraph::LocalizabilityGraph() : LocalizabilityGraph(0) {

}

TeachRepeat::LocalizabilityGraph::LocalizabilityGraph(int nOfPoints) :
    graph() {

    if(nOfPoints > 0) {
        source = boost::add_vertex(0, graph);
    }

    for(int i = 1; i < nOfPoints; i++) {
        boost::add_vertex(i, graph);
    }
}

void TeachRepeat::LocalizabilityGraph::anchorPointLocalizesPoint(int anchorPointIndex,
                                                                 int pointIndex) {
    VertexIterator begin, end;
    boost::tie(begin, end) = boost::vertices(graph);

    Vertex anchorPoint = *(begin + anchorPointIndex);
    Vertex point = *(begin + pointIndex);

    boost::add_edge(anchorPoint, point, 1, graph); // Every edge has a weight of one.
}

std::vector<int> TeachRepeat::LocalizabilityGraph::optimalSetOfAnchorPoints() const {
    int nVerticesOfGraph = boost::num_vertices(graph);
    std::vector<Vertex> predecessors(nVerticesOfGraph);
    std::vector<Weight> distancesFromSource(nVerticesOfGraph);

    IndexMap indexMap = boost::get(boost::vertex_index, graph);
    PredecessorMap predecessorMap(&predecessors[0], indexMap);
    DistanceMap distanceMap(&distancesFromSource[0], indexMap);

    boost::dijkstra_shortest_paths(graph, source, boost::distance_map(distanceMap).predecessor_map(predecessorMap));

    std::cout << "distances and parents" << std::endl;

    NameMap nameMap = boost::get(boost::vertex_name, graph);
    BGL_FORALL_VERTICES(v, graph, Graph)
    {
        std::cout << "distance(" << nameMap[source] << ", " << nameMap[v] << ") = " << distanceMap[v] << ", ";
        std::cout << "predecessor(" << nameMap[v] << ") = " << nameMap[predecessorMap[v]] << std::endl;
    }

    return std::vector<int>();
}


