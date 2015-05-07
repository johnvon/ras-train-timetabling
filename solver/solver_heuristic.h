/*
 * solver_heuristic.h
 *
 *  Created on: 07/mag/2015
 *      Author: giacomo
 */

#ifndef SOLVER_SOLVER_HEURISTIC_H_
#define SOLVER_SOLVER_HEURISTIC_H_
#include <data/array.h>
#include <data/data.h>
#include <data/path.h>

#include <boost/optional.hpp>
struct solver_heuristic {
public:
	auto solve() -> boost::optional<bv<path>>;
	solver_heuristic();
	virtual ~solver_heuristic();
};

#endif /* SOLVER_SOLVER_HEURISTIC_H_ */
