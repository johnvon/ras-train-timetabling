#ifndef GRAPH_CPP
#define GRAPH_CPP

#include <utility>
using std::make_pair;

#include <network/graph.h>

pair<bool, Vertex> Graph::get_vertex(const std::shared_ptr<Junction> j, const int t) {
    pair<vit, vit> vp;
    for(vp = vertices(graph); vp.first != vp.second; ++vp.first) {
        Node n = *graph[*vp.first];
        if(n.junction == j && n.time_interval == t) {
            return make_pair(true, *vp.first);
        }
    }
    
    return make_pair(false, Vertex());
}

#endif