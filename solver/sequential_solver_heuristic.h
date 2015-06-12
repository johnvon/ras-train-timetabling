/*
 * sequential_solver_heuristic.h
 *
 *  Created on: 07/mag/2015
 *      Author: simone
 */

#ifndef SOLVER_SEQUENTIAL_SOLVER_HEURISTIC_H_
#define SOLVER_SEQUENTIAL_SOLVER_HEURISTIC_H_
#include <data/array.h>
#include <data/data.h>
#include <data/graph.h>
#include <data/path.h>

#include <boost/optional.hpp>

struct sequential_solver_heuristic{
public:
	data& d;

	/*! Basic constructor */
	sequential_solver_heuristic(data& d): d{d} {}

	/*! Schedule the trains one by one heuristically */
	auto solve_sequentially() -> boost::optional<bv<path>>;

    /*! Remove all arcs incompatible with the paths */
    auto constrain_graph_by_paths(graph& gr, const bv<path>& paths) -> void;

private:

    auto fix_path_for(graph& gr, const path& p) -> void;
    auto remove_incompatible(graph& gr, unsigned int j, const path& p) -> void;

};

#endif /* SOLVER_SEQUENTIAL_SOLVER_HEURISTIC_H_ */
