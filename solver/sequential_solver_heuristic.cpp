/*
 * sequential_solver_heuristic.cpp
 *
 *  Created on: 07/mag/2015
 *      Author: simone
 */

#include "sequential_solver_heuristic.h"

auto solve_sequentially() -> boost::optional<bv<path>> {
	auto trains_to_schedule = uint_vector();
	    auto paths = bv<path>();

	    for(auto i = 0u; i < d.nt; i++) {
	        paths.push_back(path(d, i));
	    }

	    for(auto i = 0u; i < d.nt; i++) {
	        auto local_d = d;

	        trains_to_schedule.push_back(i);
	        paths.at(i).make_empty();

	        std::cout << "SEQUENTIAL_SOLVER >> Trains to schedule: ";
	        std::copy(trains_to_schedule.begin(), trains_to_schedule.end(), std::ostream_iterator<unsigned int>(std::cout, " "));
	        std::cout << std::endl;

	        local_d.gr.only_trains(trains_to_schedule, d.nt, d.ns, d.ni);
	        constrain_graph_by_paths(local_d.gr, paths);

	        auto s = solver(local_d);
	        auto p_sol = s.solve();

	        if(p_sol) {
	            paths = *p_sol;
	            std::cout << "SEQUENTIAL_SOLVER >> Paths: "<< std::endl;
	            for(const auto& p : paths) {
	                if(!p.is_dummy()) {
	                    std::cout << "\tTrain " << p.train << ", cost: " << p.cost << std::endl;
	                }
	            }
	        } else {
	            if(i > 0u) {
	                std::cout << "SEQUENTIAL_SOLVER >> Scheduled trains: ";
	                std::copy(trains_to_schedule.begin(), trains_to_schedule.end() - 1, std::ostream_iterator<unsigned int>(std::cout, " "));
	                std::cout << "- Could not schedule train: " << i << std::endl;
	            } else {
	                std::cout << "SEQUENTIAL SOLVER >> Could not schedule any train!" << std::endl;
	            }
	            break;
	        }
	    }

	    // All paths now have a pointer to dead local_d objects, we replace them with pointers to d
	    for(auto& p : paths) {
	        p.d = &d;
	    }

	    return paths;
}
