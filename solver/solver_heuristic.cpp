/*
 * solver_heuristic.cpp
 *
 *  Created on: 07/mag/2015
 *      Author: giacomo
 */

#include <solver/solver_heuristic.h>
#include <data/path.h>

auto solver_heuristic::solve()-> path{
	graph * gr = &d.gr;
	trains * trn = &d.trn;

	uint_matrix_2d segments;	//Contains the ordered list of segments corresponding to the nodes to be visited (origin, SAs, destination)

	segments.push_back(trn->orig_segs[train]);
	for(auto i = 0u; i< trn->sa_num[train]; i++){
		segments.push_back(trn->sa_segs[train][i]);
	}
	segments.push_back(trn->dest_segs[train]);

	unsigned int now = trn->entry_time[train];

	//The path is built concatenating the partial paths that connect the nodes to be visited
	bv<node> node_seq;

	for(auto i=0u; i<trn->sa_num+1; i++){

		/* We are given a junction to be visited.
		 * If we use a segment that enters in that junction (according to train's direction) we simplify the search function
		 * since we don't have to choose a segment among the ones exiting the junction.
		 * Of course all segments exiting the junction will have the same inverse_delta set, i.e. all the segments entering
		 * the junction.
		 * We also need to verify that this entering segment is available at the desired time
		 */

		/* This should never fail because:
		 * if it's a schedule adherence we came from a sure existing arc
		 * if it's at the origin we have the sigma segment and its arcs that are never deleted
		 */
		unsigned int src;
		for(const unsigned int& prev : gr->bar_inverse_delta[train][segments[i][0]]){
			if(gr->v[train][prev][now-1]){
				src = prev;
				break;
			}
		}

		/*TODO maybe this segment is occupied when we finish and we have to wait, but another
		 * segment could be free
		 */
		unsigned int dst = gr->bar_delta[train][segments[i+1][0]][0];

		bv<node> greedy_result= dijkstra_extra_greedy(gr, trn, now-1, src, dst);
		now = now + greedy_result.size();
		for(const node& n : greedy_result){
			node_seq.push_back(n);
		}
	}

	return path(d, train, node_seq, node_seq.size());

}

auto solver_heuristic::dijkstra_extra_greedy(graph * gr, trains * trn, auto start, auto src_segment, auto dst_segment)-> bv<node> {
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

auto solver_heuristic::simple_single_scheduler(graph * gr, trains * trn, auto start, auto src_segments, auto dst_segments) -> bv<node>{
	auto segs = trn->orig_segs[train];
	auto now = trn->entry_time[train];
	bv<node> seq;
	seq.push_back(node(0,now));
	std::pair<unsigned int, unsigned int> best = std::make_pair(0,5);;
	for(const unsigned int& s : segs){
		if(d.seg.type[s]>='0' || d.seg.type[s]<='2'){
			if(!d.net.unpreferred[train][s]){
				best = std::make_pair(s,0);
				break;
			}
			else{
				if(best.second>1)
					best = std::make_pair(s,1);
			}
		}
		else{
			if(d.seg.type[s]=='S'){
				if(best.second>2)
					best = std::make_pair(s,2);
			}
		}
	}
	assert(best.second<3);
	unsigned int t = segment_time(best.first, now); //Da qui
	seq.push_back(node(best.first,now));
	bool finished = false;
	while(!finished){

	}
}
auto solver_heuristic::segment_time(auto seg, auto now) -> unsigned int{
	unsigned int t = now;
	while(t<=now+d.net.min_travel_time[train][seg]){
		if(d.mnt.is_mow[seg][t]){
			while(d.mnt.is_mow[seg][t]){
				t++;
			}
			break;
		}
		t++;
	}
	return t;
}

