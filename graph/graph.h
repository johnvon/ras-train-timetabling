#ifndef GRAPH_H
#define GRAPH_H

#include <graph/node.h>
#include <preprocessing/data.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <memory>
#include <utility>

using namespace boost;

typedef adjacency_list<listS, listS, bidirectionalS, Node> graph_t;
typedef graph_traits<graph_t>::vertex_descriptor vertex_t;
typedef graph_traits<graph_t>::edge_descriptor edge_t;
typedef graph_traits<graph_t>::vertex_iterator vi_t;
typedef graph_traits<graph_t>::edge_iterator ei_t;
typedef graph_traits<graph_t>::out_edge_iterator oei_t;
typedef graph_traits<graph_t>::in_edge_iterator iei_t;

class Graph {
    std::shared_ptr<const Data> d;
    
    bool mow(const std::shared_ptr<const Segment> s, const int t) const;
    std::pair<bool, vertex_t> sigma() const;
    std::pair<bool, vertex_t> tau() const;
    
public:
    graph_t g;
    const Train tr;
    
    Graph(const std::shared_ptr<const Data> d, const Train tr);
    std::pair<bool, vertex_t> vertex_for(const std::shared_ptr<const Segment> s, const int t) const;
};

#endif