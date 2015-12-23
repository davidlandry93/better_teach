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

        for(int i = 0; i < nodes.size() - 1; i++) {
            graph.addArc(nodes[i], nodes[i+1]);
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

        std::list<int> optimalSetOfAnchors;
        Node lastNode = nodes[nodes.size() - 1];
        for(Node currentNode = dijkstra.predNode(lastNode); currentNode != nodes[0]; currentNode = dijkstra.predNode(currentNode)) {
            if(currentNode != lemon::INVALID) {
                optimalSetOfAnchors.push_front(indexMap[currentNode]);
            } else {
                std::cout << "node was not reached" << std::endl;
                return std::list<int>();
            }
        }
        optimalSetOfAnchors.push_front(0);  // The first node is always part of the optimal set.

        return optimalSetOfAnchors;
    }

    void LocalizabilityGraph::save(std::ostream& os) const {
        lemon::DigraphWriter<Graph> writer(graph, os);
        writer.nodeMap("id", indexMap);
        writer.run();
    }
}




