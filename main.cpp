#ifndef MAIN_CPP
#define MAIN_CPP

#include <graph/graph.h>
#include <preprocessing/data.h>
#include <solver/mip_solver.h>

#include <iostream>
#include <memory>
#include <vector>

int main(int argc, char* argv[]) {
    if(argc != 2) {
        std::cout << "Wrong number of arguments" << std::endl;
        return -1;
    }
    
    std::shared_ptr<const Data> d {std::make_shared<const Data>(argv[1])};
    std::vector<std::shared_ptr<const Graph>> graphs;
    
    for(const Train& tr : d->trains) {
        graphs.push_back(std::make_shared<const Graph>(d, tr));
    }
    
    MipSolver msolv {d, graphs};
    
    msolv.solve();
    
    return 0;
}

#endif