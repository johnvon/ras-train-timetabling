/*
 * solver_heuristic.cpp
 *
 *  Created on: 07/mag/2015
 *      Author: giacomo
 */

#include <solver/solver_heuristic.h>
#include <data/path.h>

auto solve()-> boost::optional<bv<path>>{
	graph * gr = &d.gr;
	trains * trn = &d.trn;
	auto sigma = trn->orig_ext[train];
	auto tau = trn->dest_ext[train];

	return dijkstra_extra_greedy(gr, trn, trn->entry_time, src, dst);

}

auto dijkstra_extra_greedy(graph * gr, trains * trn, auto start, auto src_segment, auto dst_segment) {
	unsigned int inf = 0xFFFFFFFF - 1;

	bv<bv<node>> previous = bv(gr->v[train].size(), bv(gr->v[train][src_segment].size(), node(src_segment, start)));

	std::set<node> S;
	S.insert(node(src_segment, start));

	unsigned int end;
	bool dst_found = false;
	bool broken = false;

	while (!dst_found || !broken) {
		broken = true;
		for(const node& explored : S){
			for (const unsigned int& connected : gr->delta[train][explored.seg]){
				if(gr->adj[train][explored.seg][explored.t][connected] && S.find(node(connected, explored.t+1))!=S.end()){
					broken = false;
					previous[connected][explored.t+1]=explored;
					S.insert(node(connected, explored.t+1));
					if(connected==dst_segment){
						end=explored.t+1;
						dst_found=true;
						break;
					}
				}
			}
			if(dst_found)
				break;
		}

	}
	bv<node> shortest;
	if(!broken){
		node actual = node(dst_segment,end);
		unsigned int next_seg;
		double cost = 0;

		while (actual == node(src_segment,start)) {
			next_seg = actual.seg;
			actual = previous[actual.seg][actual.t];
			cost += gr->costs[train][actual.seg][actual.t][next_seg];
			shortest.push_back(actual);
		}
		shortest.erase(shortest.begin()+shortest.size()-1);
		cost -= gr->costs[train][src_segment][start][next_seg];

		shortest=std::reverse(shortest.begin(), shortest.end());

	}
	return shortest;	//restituisce il percorso come vettore di nodi, vuoto se non siamo riusciti a schedularlo
}

