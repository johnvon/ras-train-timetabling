#ifndef SEQUENTIAL_SOLVER_H
#define SEQUENTIAL_SOLVER_H

#include <data/array.h>
#include <data/data.h>
#include <data/graph.h>
#include <data/path.h>

#include <boost/optional.hpp>

/*! \brief This class represents a solver that schedules the trains sequentially */
struct sequential_solver {
    const data& d;
    
    /*! Schedule the trains one by one */
    auto solve_sequentially() -> boost::optional<bv<path>>;
    
    /*! Remove all arcs incompatible with the paths */
    auto constrain_graph_by_paths(graph& gr, const bv<path>& paths) -> void;
    
    /*! Basic constructor */
    sequential_solver(const data& d) : d{d} {}
    
private:
    
    auto fix_path_for(graph& gr, const path& p) -> void;
    auto remove_incompatible(graph& gr, unsigned int j, const path& p) -> void;
};

#endif
