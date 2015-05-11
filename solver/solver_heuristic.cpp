/*
 * solver_heuristic.cpp
 *
 *  Created on: 07/mag/2015
 *      Author: giacomo
 */

#include "solver_heuristic.h"

using uint_pair = std::pair<unsigned int, unsigned int>;

auto solve(){

	//path p = path(d, train);
	graph * gr = &d.gr;
	trains * trn = &d.trn;
	auto sigma = trn->orig_ext[train];
	auto tau = trn->dest_ext[train];

	return dijkstra_time_expanded(gr, trn, src, dst);

}

auto dijkstra_time_expanded(graph * gr, trains * trn, auto start, auto src_segment, auto dst_segment) {
	unsigned int inf = 0xFFFFFFFF - 1;
	bv<bv<uint_pair>> previous = bv(gr->v[train].size(), bv(gr->v[train][start].size(), std::make_pair(src_segment, start)));

	std::set<uint_pair> S;
	S.insert(std::make_pair(src_segment, start));

	unsigned int now = start;
	bool dst_found = false;
	while (!dst_found) {
		for(const uint_pair& explored : S){
			for (const unsigned int& connected : gr->delta[train][explored.first]){
				if(gr->adj[train][explored.first][explored.second][connected] && S.find(std::make_pair(connected, explored.second+1)!=S.end())
				if(S.find(connected)==S.end()){
					S.insert(connected);
					previous[connected]=explored;
				}
			}
		}

	}
}

