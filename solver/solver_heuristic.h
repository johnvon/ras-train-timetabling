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
#include <iostream>

#include <boost/optional.hpp>

using node = path::node;

struct solver_heuristic {
public:
	data& d;
	unsigned int train;

	auto solve() -> path;
	solver_heuristic(data& d, unsigned int train): d{d}, train{train} {}

private:
	auto dijkstra_extra_greedy(graph * gr, trains * trn, auto start, auto src_segments, auto dst_segments) -> bv<node>;

	/* Returns the sequence of nodes (segment, instant) of the schedule for "train" with free network */
	auto simple_single_scheduler() -> bv<node>;

	/* Returns the time to wait in case the segment is under maintenance (0 if free) with free network*/
	auto mow_wait_time(unsigned int seg, unsigned int now) -> unsigned int;

	/* Insert the nodes relative to the wait on current segment (in case the next one is under maintenance) and to reach
	 * the end of the next segment with free network
	 */
	auto wait_and_travel(unsigned int here, unsigned int next, unsigned int& now, bv<node> &seq, bool &finished)->void;

	/* Returns the next segment (policy: main preferred > siding > crossover > main unpreferred) */
	auto inline choose_next(unsigned int here, unsigned int now) -> unsigned int;
};

#endif /* SOLVER_SOLVER_HEURISTIC_H_ */
