#ifndef GRAPH_GENERATOR_H
#define GRAPH_GENERATOR_H

#include <memory>

#include <network/graph.h>
#include <network/junction.h>
#include <network/train.h>
#include <network/track.h>
#include <preprocessing/data.h>

namespace GraphGenerator {
    std::shared_ptr<Graph> create_graph(const Data& data, std::shared_ptr<Train> train);
    void clean_graph(std::shared_ptr<Graph> g);
    void create_arc(std::shared_ptr<Junction> j1, const int t1, std::shared_ptr<Junction> j2, const int t2, std::shared_ptr<Graph> g, const int id, std::shared_ptr<Track> track);
    bool is_in_maintenance(std::shared_ptr<Track> track, const int t1, const int t2, const vector<Mow>& mow);
}

#endif