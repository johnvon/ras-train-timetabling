#ifndef PARAMS_H
#define PARAMS_H

#include <string>

struct params {
    struct cplex_params {
        int threads;
        int time_limit;
        
        cplex_params() {}
        cplex_params(int threads, int time_limit) : threads{threads}, time_limit{time_limit} {}
    };
    
    struct model_params {
        bool alternative_min_travel_time_cst;
        bool max_travel_time_cst;
    
        model_params() {}
        model_params(bool amin_cst, bool max_cst) : alternative_min_travel_time_cst{amin_cst}, max_travel_time_cst{max_cst} {}
    };

    struct heuristics_params {
        bool simplified_objective_function;
        bool corridor;
        int corridor_minutes_around_ideal;
        double corridor_pct_around_ideal;
        bool sparsification;
        int sparsification_keepall_range;
    
        heuristics_params() {}
        heuristics_params(bool simp_obj, bool corridor, int corridor_minutes_around_ideal, double corridor_pct_around_ideal, bool sparsification, int sparsification_keepall_range) : simplified_objective_function{simp_obj}, corridor{corridor}, corridor_minutes_around_ideal{corridor_minutes_around_ideal}, corridor_pct_around_ideal{corridor_pct_around_ideal}, sparsification{sparsification}, sparsification_keepall_range{sparsification_keepall_range} {}
    };
    
    std::string results_file;
    cplex_params cplex;
    model_params model;
    heuristics_params heuristics;
    
    params(std::string file_name);
};

#endif