#ifndef GRAPH_GENERATOR_CPP
#define GRAPH_GENERATOR_CPP

#include <memory>
using std::make_shared;

#include <network/graph.h>
#include <preprocessing/data.h>
#include <preprocessing/graph_generator.h>

std::shared_ptr<Graph> GraphGenerator::create_graph(const Data& data, std::shared_ptr<Train> train) {
    std::shared_ptr<Graph> g = make_shared<Graph>(BoostGraph(), train);
    
    /* Create vertices */
    for(std::shared_ptr<Junction> j : data.junctions) {
        for(int t = 1; t <= data.num_times; ++t) {
            Vertex v = add_vertex(g->graph);
            g->graph[v] = make_shared<Node>(j, t);
        }
    }
}

#endif