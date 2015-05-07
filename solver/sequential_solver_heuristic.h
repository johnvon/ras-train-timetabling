/*
 * sequential_solver_heuristic.h
 *
 *  Created on: 07/mag/2015
 *      Author: simone
 */

#ifndef SOLVER_SEQUENTIAL_SOLVER_HEURISTIC_H_
#define SOLVER_SEQUENTIAL_SOLVER_HEURISTIC_H_

#include "sequential_solver.h"

struct sequential_solver_heuristic: virtual public sequential_solver {
public:
	/*! Schedule the trains one by one heuristically */
	    auto solve_sequentially() -> boost::optional<bv<path>>;
};

#endif /* SOLVER_SEQUENTIAL_SOLVER_HEURISTIC_H_ */
