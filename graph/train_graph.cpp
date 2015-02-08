#include <graph/train_graph.h>

#include <iostream>

train_graph::train_graph(const data& d, int train_number) : d{d} {
    for(auto s = 0; s < d.ns + 2; s++) {
        for(auto t = 0; t < d.ni + 2; t++) {
            if(d.v[train_number][s][t]) {
                auto v = add_vertex(g);
                g[v] = node(s, t);
            }
        }
    }
    
    vi_t vi, vi_end, vj, vj_end;
    
    for(std::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
        auto s1 = g[*vi].segment;
        auto t1 = g[*vi].time_interval;
        
        for(std::tie(vj, vj_end) = vertices(g); vj != vj_end; ++vj) {
            auto s2 = g[*vj].segment;
            auto t2 = g[*vj].time_interval;
            
            if(t2 == t1 + 1 && d.adj[train_number][s1][t1][s2]) {
                add_edge(*vi, *vj, g);
            }
        }
        
        assert(out_degree(*vi, g) == (unsigned int) d.n_out[train_number][s1][t1]);
    }
}