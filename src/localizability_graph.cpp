#include <iostream>
#include "localizability_graph.h"

namespace TeachRepeat {
    LocalizabilityGraph::LocalizabilityGraph() : LocalizabilityGraph(0) {

    }

    LocalizabilityGraph::LocalizabilityGraph(int nOfPoints) :
            graph(), indexMap(graph) {

        for(int i = 0; i < nOfPoints; i++) {
            Node addedNode = graph.addNode();
            indexMap[addedNode] = i;
            nodes.push_back(addedNode);
        }
    }

    void LocalizabilityGraph::anchorPointLocalizesPoint(int anchorPointIndex,
                                                                     int pointIndex) {
        graph.addArc(nodes[anchorPointIndex], nodes[pointIndex]);
    }

    std::list<int> LocalizabilityGraph::optimalSetOfAnchorPoints() const {
        LengthMap distances(graph);

        for(Graph::ArcIt it(graph); it != lemon::INVALID; ++it) {
            distances[it] = 1;
        }

        lemon::Dijkstra<Graph, LengthMap> dijkstra(graph, distances);
        dijkstra.run(nodes[0]);

        std::cout << "Cost: " << dijkstra.dist(nodes[nodes.size() - 1]) << std::endl;

        std::list<int> optimalSetOfAnchors;
        Node lastNode = nodes[nodes.size() - 1];
        for(Node currentNode = dijkstra.predNode(lastNode); currentNode != nodes[0]; currentNode = dijkstra.predNode(currentNode)) {
            if(currentNode != lemon::INVALID) {
                std::cout << indexMap[currentNode] << std::endl;
                optimalSetOfAnchors.push_front(indexMap[currentNode]);
            } else {
                std::cout << "node was not reached" << std::endl;
            }
        }
        optimalSetOfAnchors.push_front(0);  // The first node is always part of the optimal set.

        return optimalSetOfAnchors;
    }
}




