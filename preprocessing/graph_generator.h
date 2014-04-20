#ifndef GRAPH_GENERATOR_H
#define GRAPH_GENERATOR_H

#include <memory>

#include <network/train.h>
#include <preprocessing/data.h>

namespace GraphGenerator {
    std::shared_ptr<Graph> create_graph(const Data& data, std::shared_ptr<Train> train);
}

#endif