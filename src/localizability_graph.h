#ifndef BETTERTEACH_LOCALIZABILITYGRAPH_H
#define BETTERTEACH_LOCALIZABILITYGRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/directed_graph.hpp>

namespace TeachRepeat {
    class LocalizabilityGraph {
        typedef int Weight;

        typedef boost::directed_graph<> Graph;
        typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;

        typedef boost::property_map< Graph, boost::vertex_index_t>::type IndexMap;
        typedef boost::property_map< Graph, boost::vertex_name_t >::type NameMap;

        typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > PredecessorMap;
        typedef boost::iterator_property_map < Weight*, IndexMap, Weight, Weight& > DistanceMap;

    public:
        LocalizabilityGraph();
        LocalizabilityGraph(int nOfPoints);
        void anchorPointLocalizesPoint(int anchorPointIndex, int pointIndex);
        std::vector<int> optimalSetOfAnchorPoints() const;

    private:
        Graph graph;
        Vertex source;
    };
}




#endif //BETTERTEACH_LOCALIZABILITYGRAPH_H
