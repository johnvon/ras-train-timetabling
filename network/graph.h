#ifndef GRAPH_H
#define GRAPH_H

#include <memory>

#include <network/arc.h>
#include <network/node.h>
#include <network/train.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
using namespace boost;

typedef adjacency_list<listS, listS, bidirectionalS, std::shared_ptr<Node>, std::shared_ptr<Arc>> BoostGraph;

typedef graph_traits<BoostGraph>::vertex_iterator vit;
typedef graph_traits<BoostGraph>::edge_iterator eit;
typedef graph_traits<BoostGraph>::in_edge_iterator ieit;
typedef graph_traits<BoostGraph>::out_edge_iterator oeit;

typedef graph_traits<BoostGraph>::vertex_descriptor Vertex;
typedef graph_traits<BoostGraph>::edge_descriptor Edge;

typedef vector<Edge> Path;

class Graph {
public:
    BoostGraph              graph;
    std::shared_ptr<Train>  train;
    
    Graph(const BoostGraph graph, std::shared_ptr<Train> train) : graph(graph), train(train) {}
    pair<bool, Vertex> get_vertex(const std::shared_ptr<Junction> j, const int t);
};

#endif