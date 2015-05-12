/*
 * solver_heuristic.cpp
 *
 *  Created on: 07/mag/2015
 *      Author: giacomo
 */

#include <solver/solver_heuristic.h>

using uint_pair = std::pair<unsigned int, unsigned int>;

auto solve()-> boost::optional<bv<path>>{
	graph * gr = &d.gr;
	trains * trn = &d.trn;
	auto sigma = trn->orig_ext[train];
	auto tau = trn->dest_ext[train];

	return dijkstra_time_expanded(gr, trn, trn->entry_time, src, dst);

}

auto dijkstra_time_expanded(graph * gr, trains * trn, auto start, auto src_segment, auto dst_segment) {
	unsigned int inf = 0xFFFFFFFF - 1;
	bv<bv<uint_pair>> previous = bv(gr->v[train].size(), bv(gr->v[train][src_segment].size(), std::make_pair(src_segment, start)));

	std::set<uint_pair> S;
	S.insert(std::make_pair(src_segment, start));

	unsigned int end;
	bool dst_found = false;

	while (!dst_found) {
		for(const uint_pair& explored : S){
			for (const unsigned int& connected : gr->delta[train][explored.first]){
				if(gr->adj[train][explored.first][explored.second][connected] && S.find(std::make_pair(connected, explored.second+1))!=S.end()){
					previous[connected][explored.second+1]=explored;
					S.insert(std::make_pair(connected, explored.second+1));
					if(connected==dst_segment){
						end=explored.second+1;
						dst_found=true;
						break;
					}
				}
			}
			if(dst_found)
				break;
		}

	}
	path p =

}

