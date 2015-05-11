/*
 * solver_heuristic.cpp
 *
 *  Created on: 07/mag/2015
 *      Author: giacomo
 */

#include "solver_heuristic.h"

auto solve(){

	//path p = path(d, train);
	graph * gr = &d.gr;
	trains * trn = &d.trn;
	auto sigma = trn->orig_ext[train];
	auto tau = trn->dest_ext[train];

	return dijkstra_time_expanded(gr, trn, src, dst);

}

auto dijkstra_time_expanded(graph * gr, trains * trn, auto start, auto src, auto dst) {
	unsigned int inf = 0xFFFFFFFF - 1;
	uint_vector dist = uint_vector(gr->n_nodes[train], inf);
	uint_vector previous = uint_vect(gr->n_nodes[train], scr);

	dist[scr] = 0;

	for (const auto& next : gr->delta[train][src]) {
		dist[next] = gr->costs[train][src][start][next];
	}

	std::set<unsigned int> S;
	while (true) {

	}
}

