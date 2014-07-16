#ifndef MIP_SOLVER_H
#define MIP_SOLVER_H

#include <graph/graph.h>
#include <preprocessing/data.h>

#include <memory>
#include <vector>

class MipSolver {
    std::shared_ptr<const Data> d;
    std::vector<std::shared_ptr<const Graph>> graphs;
    
public:
    MipSolver(std::shared_ptr<const Data> d, std::vector<std::shared_ptr<const Graph>> graphs) : d{d}, graphs{graphs} {}
    void solve() const;
};

#endif