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
#include <set>

#include <boost/optional.hpp>

using node = path::node;

struct solver_heuristic {
public:
	data& d;
	unsigned int train;

	auto solve() -> boost::optional<bv<path>>;
	solver_heuristic(data& d, auto train): d{d}, train{train} {}

private:
	auto dijkstra_time_expanded(graph * gr, trains * trn, auto start, auto src_segment, auto dst_segment);
};

#endif /* SOLVER_SOLVER_HEURISTIC_H_ */
