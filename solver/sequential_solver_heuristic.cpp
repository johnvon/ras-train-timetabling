/*
 * sequential_solver_heuristic.cpp
 *
 *  Created on: 07/mag/2015
 *      Author: simone
 */

#include <data/array.h>
#include <data/data.h>
#include <data/graph.h>
#include <data/path.h>

#include <boost/optional.hpp>

#include "sequential_solver_heuristic.h"


auto sequential_solver_heuristic::solve_sequentially() -> boost::optional<bv<path>> {

	bv<path> paths;
	auto s = solver_heuristic(d,0); //crea e invoca il solver
	auto p_sol = s.solve();

	paths.push_back(p_sol);
	return paths;
}
