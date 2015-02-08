#ifndef TRAIN_GRAPH_H
#define TRAIN_GRAPH_H

#include <graph/data.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

using namespace boost;

struct train_graph {
    struct node {
        int segment;
        int time_interval;
        
        node() {}
        node(int segment, int time_interval) : segment{segment}, time_interval{time_interval} {}
    };
    
    using graph_t = adjacency_list<listS, listS, directedS, node>;
    using vertex_t = graph_traits<graph_t>::vertex_descriptor;
    using edge_t = graph_traits<graph_t>::edge_descriptor;
    using vi_t = graph_traits<graph_t>::vertex_iterator;
    using ei_t = graph_traits<graph_t>::edge_iterator;
    
    const data& d;
    int         train_number; // The train this graph is for
    graph_t     g;
    
    train_graph(const data& d, int train_number);
};

#endif