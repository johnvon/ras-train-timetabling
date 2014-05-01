#ifndef MAIN_CPP
#define MAIN_CPP

#include <vector>
using std::vector;
#include <memory>

#include <network/graph.h>
#include <network/train.h>
#include <preprocessing/data.h>
#include <preprocessing/graph_generator.h>

int main() {
    Data data("data/toy.json");
    data.print();
    
    vector<std::shared_ptr<Graph>> graphs;
    for(const std::shared_ptr<Train> t : data.trains) {
        graphs.push_back(GraphGenerator::create_graph(data, t));
    }
    
    return 0; 
}

#endif